#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Base class for loitering synchronization methods.

This class contains all the functions necessary to implement any loitering synchronization method.
A StateSharing node is initialized and used to gather the state of current or other drones.
A control loop with a timer is used to calculate and set the speed of the drone. 
The function publish_speed is then used to give the command to the drone.

To use this as a parent for a new synchronization method:
    pass params and publishing_enabled to it using super().__init__(name, params, publishing_enabled)
    Call initialize_ros() right after to initialize
    redefine unpack_method_params() to include any new params necessary
    redefine control_loop() to contain your algorithm. Make sure to call publish_speeds() inside it. 
    Examples of how to access the yaw from the stateSharing node is in the control_loop() below.

note: publishing_enabled relates to the stateSharing node. If True, a topic called /state would be published every
      STATE_PUBLISHING_DT seconds containing the state of the drone. Not all methods need this, so it is False by default.
"""

import rclpy
from rclpy.node import Node
from time import sleep
import random

from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import MissionResult
from lrs_msgs.msg import GoCommand



from .state_sharing_node import StateSharing
from .state_sharing_node_with_yaw_phase import StateSharing_withOffset
from .ideal_angle import *


class LoiteringSyncBase(Node):
    def __init__(self, name, params, publishing_enabled=False):
        super().__init__(name)
        self.unpack_params(params)
        self.unpack_method_params(params)
        ident = self.get_namespace() # Gets the id of the drone
        if ident[-2].isdigit():
            self.ident_key = ident[-2:]
            self.ident = self.DRONE_IDS[self.ident_key] # Set the id to be the drone number
        else:
            self.ident_key = ident[-1]
            self.ident = self.DRONE_IDS[self.ident_key]
        if self.RANDOM_SEED != -1:
            random.seed(self.ident*self.RANDOM_SEED)
     
        # define stateSharing node:
        if self.PHASE_OFFSET_ENABLE:
            self.state_sharing = StateSharing_withOffset(self.STATE_PUBLISHING_ENABLED, self.STATE_PUBLISHING_DT, 1/self.CONTROL_DELAY, self.DDS_FLAG, self.ESTIMATION_FLAG, self.ESTIMATION_METHOD, self.PHASE_OFFSET[self.ident-1])
        else:
            self.state_sharing = StateSharing(self.STATE_PUBLISHING_ENABLED, self.STATE_PUBLISHING_DT, 1/self.CONTROL_DELAY, self.DDS_FLAG, self.ESTIMATION_METHOD)

        self.mission_start = False
        print("Initiating loitering sync node")

        if self.DDS_FLAG:
            qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
            vehicle_command_topic = "fmu/in/vehicle_command"
            vehicle_control_mode_topic = "fmu/out/vehicle_control_mode"
            mission_result_topic = "fmu/out/mission_result"
        else:
            qos_policy = 10
            vehicle_command_topic = "fmu/vehicle_command/in"
            vehicle_control_mode_topic = "fmu/vehicle_control_mode/out"
            mission_result_topic = "fmu/mission_result/out"

        # A timer for the control loop
        self.control_timer_ = self.create_timer(
            self.CONTROL_DELAY,
            self.control_loop
        )

        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand,
            vehicle_command_topic,
            qos_policy,
        )
        self.vehicle_control_mode_sub_ = self.create_subscription(
            VehicleControlMode,
            vehicle_control_mode_topic,
            self.vehicle_control_mode_callback,
            qos_policy,
        )
        self.mission_result_sub_ = self.create_subscription(
            MissionResult,
            mission_result_topic,
            self.mission_result_callback,
            qos_policy,
        )

        self.go_command_sub_ = self.create_subscription(
            GoCommand,
            "/go_command",
            self.go_command_callback,
            qos_policy,
        )

        # a publisher for go command
        self.go_command_pub_ = self.create_publisher(
            GoCommand,
            "/go_command",
            10,
        )

        self.loitering = False
        self.synchronized = False
        self.synchronized_counter = 0
        self.timestamp_ = 0
        self.print_counter = 0
        self.shutdown = False
                        
        # initialize the ros node # commented out for the other methods to initialize properly
        # print("Executing ros nodes")
        # self.initialize_ros()


    # initialize ros node
    def initialize_ros(self):
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self)
        # Add stateSharing node 
        executor.add_node(self.state_sharing)
        while not self.shutdown:
            executor.spin_once()

    # A function that publishes the go command for the drones
    def publish_go_command(self, command, source):
        for i in range(3):
            msg = GoCommand()
            msg.timestamp = 0
            msg.id = source
            msg.command = command
            self.go_command_pub_.publish(msg)
            self.get_logger().info('Publishing go command!')
            sleep(0.2)

    # unpack basic parameters needed for all sync methods
    def unpack_params(self, params):
        self.STATE_PUBLISHING_ENABLED = params['STATE_PUBLISHING_ENABLED']
        self.SIMULATION = params['SIMULATION']
        self.SPEEDS = params['SPEEDS']
        self.ACCEPTANCE_RANGE = params['ACCEPTANCE_RANGE']
        self.CONTROL_DELAY = params['CONTROL_DELAY']
        self.TAKEOFF_DELAY = params['TAKEOFF_DELAY']
        self.TAKEOFF_DELAY_RANDOM = params['TAKEOFF_DELAY_RANDOM']
        self.RANDOM_SEED = params['RANDOM_SEED']
        self.STATE_PUBLISHING_DT = params['STATE_PUBLISHING_DT']
        self.DDS_FLAG = params['DDS_FLAG']
        self.DRONE_IDS = params['DRONE_IDS']
        self.ESTIMATION_METHOD = params["ESTIMATION_METHOD"]
        self.ESTIMATION_FLAG = params["ESTIMATION_FLAG"]
        self.TRAJECTORY_FLAG = params["TRAJECTORY_FLAG"]
        self.TRAJECTORY_DT = params["TRAJECTORY_DT"]
        self.chosen_method = params['chosen_method']
        self.LAST_WAYPOINT = params["LAST_WAYPOINT"]

        self.PHASE_OFFSET_ENABLE = params['PHASE_OFFSET_ENABLE']
        self.PHASE_OFFSET = params['PHASE_OFFSET']
        self.BACKYARD = params['BACKYARD']

    # unpack parameters needed for a specific sync method
    def unpack_method_params(self, params):
        pass

    # Function that checks if waypoint 2 was reached in the mission.
    # If it was, that means the agent is in FW mode and we should start formation control
    def mission_result_callback(self, msg):
        if msg.seq_current == 2 and not self.loitering:
            self.get_logger().info(f"Loitering started")
            self.loitering = True
        elif msg.seq_current != 2 and self.loitering:
            self.get_logger().info(f"Loitering finished (mission)")
            self.loitering = False


    # Function that receives a go command from the main computer when ready to move to next waypoint
    def go_command_callback(self, msg):
        # checks the command came from the main computer
        print("received command callback ", msg)
        if msg.id == 0:
            if msg.command == 0 and self.chosen_method != 'mean': # destroy the node
                self.shutdown = True
                self.get_logger().info(f"Return to Mission waypoint 2")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_MISSION_START, param1=1, param2=self.LAST_WAYPOINT+1)
            elif msg.command == 1: # Start trajectory command
                # must move to next waypoint.
                # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_JUMP, param1=5, param2=0)
                # Hacky way to do the switch in mission waypoint. The first number is the first item in the mission sequene
                # it starts the mission from that item.
                self.get_logger().info(f"Go to Mission waypoint 4. Starting trajectory Following.")
                if self.ident == 1:
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_MISSION_START, param1=3, param2=self.LAST_WAYPOINT+1)
                    sleep(self.TRAJECTORY_DT) # wait for the difference duration between drones
                    self.publish_go_command(1, self.ident)
            elif msg.command == 2: # restart loitering sync command
                # must move to next waypoint.
                # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_JUMP, param1=5, param2=0)
                # Hacky way to do the switch in mission waypoint. The first number is the first item in the mission sequene
                # it starts the mission from that item.
                self.get_logger().info(f"Return to Mission waypoint 2")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_MISSION_START, param1=1, param2=self.LAST_WAYPOINT+1)
            elif msg.command == 3: # turn on trajectory following synchronization
                self.get_logger().info(f"Trajectory Flag on")
                self.TRAJECTORY_FLAG = True
            elif msg.command == 4: # turn off trajectory following synchronization
                self.get_logger().info(f"Trajectory Flag off")
                self.TRAJECTORY_FLAG = False
            elif msg.command == 5: # Go to landing infinite
                self.get_logger().info(f"Abort everything. Going to loitering sync infinite loitering.")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_MISSION_START, param1=1, param2=self.LAST_WAYPOINT+1)
            elif msg.command == 7: # Turn off trajectory flag. # 6 taken as command to shutdown metrics node
                self.get_logger().info(f"Trajectory Flag off")
                self.TRAJECTORY_FLAG = False
            elif msg.command > 1000 and msg.command < 2000: # Go to waypoint number in the first 3 digits
                waypoint_num = msg.command - 1000 - 1 # the waypoint number is 1 less than the number seen in QGC
                self.get_logger().info(f"Go to waypoint {waypoint_num}.")
                if waypoint_num > self.LAST_WAYPOINT: # max on the final loitering of the mission
                    waypoint_num = self.LAST_WAYPOINT
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_MISSION_START, param1=waypoint_num, param2=self.LAST_WAYPOINT+1)
        elif (msg.command == 1) and (msg.id == (self.ident - 1)):
            self.get_logger().info(f"Go to Mission waypoint 4. Starting trajectory Following.")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_MISSION_START, param1=3, param2=self.LAST_WAYPOINT+1)
            sleep(self.TRAJECTORY_DT) # wait for the difference duration between drones
            self.publish_go_command(1, self.ident)


    # Function that applies the formation principles and calculates the commands for the agent
    def control_loop(self):
        # timestamp used in vehicle command publisher. Good to distinguish different commads.
        self.timestamp_ = self.state_sharing.behavior.me.timestamp_

        if (not self.mission_start):
            self.get_logger().info(f"Starting mission")
            self.mission_start = True
            self.publish_mission_start()

        if self.loitering or self.BACKYARD:
            self.print_counter += 1
            # example of getting my gps coords from state sharing node.
            lon = self.state_sharing.behavior.me.lon
            lat = self.state_sharing.behavior.me.lat
            alt = self.state_sharing.behavior.me.alt
            my_coords = [lon, lat, alt]
            if self.print_counter == 10:
                self.get_logger().info(f"My current position is: {lon}, {lat}, {alt}")
                print('My current position is: ', lon, lat, alt)

            # Example of gettings gps coordinates of all other agents
            agents = self.state_sharing.behavior.agents
            coord_list = []
            # Get all the coordinates from the agents including me
            for key in agents:
                # keep yaw in 0-2pi range
                lon = agents[key].lon
                lat = agents[key].lat
                alt = agents[key].alt
                coord_list.append([lon,lat,alt])
            
            # add my coordinates to the list
            coord_list.append(my_coords)
            
            if self.print_counter == 10:
                print("Agent coords are:")
                print(coord_list)
                self.print_counter = 0    
            # You can get q or roll, pitch, yaw using the same method. 

            # example of giving the drone a speed command
            self.publish_speed(17.0)
        

    # Checks if we switch to offboard mode
    # if switched, stop synchronized loitering
    # OK initially, but should be removed when we implement synchronized trajectories
    def vehicle_control_mode_callback(self, msg):
        if msg.flag_control_offboard_enabled and self.loitering:
            self.get_logger().info(f"Loitering finished (offboard)")
            self.loitering = False

    # General function for publishing vehicle commands
    # Used to publish speed commands
    def publish_vehicle_command(
            self,
            command,
            param1,
            param2,
            param3 = None,
            param4 = None,
            param5 = None,
            param6 = None,
            param7 = None
    ):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        if param3:
            msg.param3 = float(param3)
        if param4:
            msg.param4 = float(param4)
        if param5:
            msg.param5 = float(param5)
        if param6:
            msg.param6 = float(param6)
        if param7:
            msg.param7 = float(param7)
        msg.command = command  # command ID
        msg.target_system = 0  # system which should execute the command
        msg.target_component = 0  # component to execute the command, 0 for all
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        self.vehicle_command_publisher_.publish(msg)

    # Function to publish the new speed commands
    def publish_speed(self, speed):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_CHANGE_SPEED, param1=0,
                                     param2=speed, param3=-1)

    # Funtion to start mission plan
    def publish_mission_start(self):
        if self.SIMULATION: 
            if self.TAKEOFF_DELAY_RANDOM:
                # Randomly delay takeoff between drones. Delay between 0 and 60 seconds
                delay = 60 * random.random()
            else:
                # Fixed delay take off of each drone by self.TAKEOFF_DELAY
                delay = float(self.ident)*self.TAKEOFF_DELAY

            print("TAKEOFF DELAY IS: ", delay)
            sleep(delay)
            # COMMENTED OUT FOR NOW!!! Remove later TODO!!!
            # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_MISSION_START, param1=0, param2=3)
        else:
            print("Please ensure simulation flag is True for auto start feature to work.")
