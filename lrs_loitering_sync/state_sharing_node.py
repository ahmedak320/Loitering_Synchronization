#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
A State Sharing Ros2 Node that subscribes to a group of messages to create and share a state with other agents on the network.
The goal of this is to provide a building block that others can easily integrate into their swarming and multi-robot projects.
This code assumes the use of PX4 messages and ROS2, as well as the use of the custom StateSharingMsg provided.

The shared state contains frame id, gps coordinates, local position, euler angles and quaternion angles

To use this in your application, look into the provided example in loitering_sync_Ahmed.py
"""
import numpy as np

import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleGlobalPosition

# imports for local sync
from lrs_msgs.msg import StateSharingMsg
from math import atan2, asin, pi
from .estimator import Estimation
import time

# # A function that converts quaternion angles to euler
# # Gotten from chatgpt, verified
def quaternion_to_euler(qw, qx, qy, qz):
    roll = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    pitch = asin(2*(qw*qy - qz*qx))
    yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return roll, pitch, yaw


# A class that contains the state definition of the agent
# It contains x, y, z and roll, pitch, yaw and lon, lat, alt as global gps coordinates
# Also contains quaternion angles and the id of the agent as well as a check for initialization
class Agent:
    def __init__(self, ident):
        self.ident = ident
        self.timestamp_ = 0
        # local position
        self.local_position_x = None
        self.local_position_y = None
        self.local_position_z = None
        # local orientation
        self.q = None # quaternion array
        self.roll = None
        self.pitch = None
        self.yaw = None
        # global gps coordinates
        self.lon = None
        self.lat = None
        self.alt = None

        # Initialization Checkers
        self.initialized = False
        self.initialized_local_pos = False
        self.initialized_q = False
        self.initialized_angles = False
        self.initialized_global_pos = False

    # update local position
    def update_position(self, x, y, z):
        self.local_position_x = x
        self.local_position_y = y
        self.local_position_z = z
        if not self.initialized:
            self.initialized_local_pos = True
            self.update_initialized()

    # update quaternion angles
    def update_quaternion(self, q):
        self.q = q
        if not self.initialized:
            self.initialized_q = True
            self.update_initialized()

    # update cartesian angles
    def update_angles(self, roll=0, pitch=0, yaw=0):
        self.roll = (roll + pi) % (2 * pi) - pi
        self.pitch = (pitch + pi) % (2 * pi) - pi
        self.yaw = (yaw + pi) % (2 * pi) - pi
        if not self.initialized:
            self.initialized_angles = True
            self.update_initialized()

    # update global coordinates
    def update_global(self, lon, lat, alt):
        self.lon = lon
        self.lat = lat
        self.alt = alt
        if not self.initialized:
            self.initialized_global_pos = True
            self.update_initialized()

    def update_initialized(self):
        self.initialized = self.initialized_local_pos and self.initialized_q and self.initialized_angles and self.initialized_global_pos

# A class that creates the agent list and has a function to update the agents
class Behavior:
    def __init__(self, self_ident, publishing_timestep, control_frequency, ESTIMATION_FLAG=True, ESTIMATION_METHOD='foh'):
        self.agents = {}
        self.me = Agent(self_ident)

        if ESTIMATION_FLAG:
            self.estimator =  Estimation(
                state_sharing_freq=publishing_timestep,
                control_loop_freq=control_frequency,
                estimation_method=ESTIMATION_METHOD
            )
        self.ESTIMATION_FLAG = ESTIMATION_FLAG
        
    # Update the agents state in the agents list
    # Also update self saved state
    def update_agent(self, agent_ident, lon, lat, alt, yaw, x=0, y=0, z=0, roll=0, pitch=0, q=[]):

        if agent_ident not in self.agents:
            self.agents[agent_ident] = Agent(agent_ident)
        self.agents[agent_ident].update_global(lon, lat, alt)
        self.agents[agent_ident].update_position(x, y, z)
        self.agents[agent_ident].update_angles(roll, pitch, yaw)
        self.agents[agent_ident].update_quaternion(q)

        if self.ESTIMATION_FLAG:
            self.update_estimation(agent_ident=agent_ident)

    def update_estimation(self, agent_ident):
        self.estimator.update_estimations(self.agents[agent_ident])
    
    def get_estimated_states(self):
        return [self.estimator.get_state(agent) for agent in self.agents.values()]

    def get_estimated_state_agent(self, agent):
        return self.estimator.get_state(agent)
    

# A class that subscribes and publishes to the states of the agents
# The user would only need to use this class to get the list of states for all the agents
class StateSharing(Node):
    def __init__(self, publishing_enabled=True, publishing_timestep=1.0, control_frequency=5, DDS_FLAG=True, ESTIMATION_FLAG=False, ESTIMATION_METHOD='foh'):
        super().__init__("StateSharing"+str(int(time.time())))
        ident = self.get_namespace() # Gets the id of the drone
        if ident[-2].isdigit():
            self.ident = ident[-2:] # Set the id to be the drone number
        else:
            self.ident = ident[-1]
        print("State sharing id is: ", self.ident)
        # self.ident = ident[-2:] # Set the id to be the drone number
        self.behavior = Behavior(self.ident, publishing_timestep, control_frequency, ESTIMATION_FLAG, ESTIMATION_METHOD)
        self.publishing_enabled = publishing_enabled

        if DDS_FLAG:
            qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
            vehicle_local_position_topic = "fmu/out/vehicle_local_position"
            vehicle_global_position_topic = "fmu/out/vehicle_global_position"
            vehicle_odometry_topic = "fmu/out/vehicle_odometry"
        else:
            qos_policy = 10
            vehicle_local_position_topic = "fmu/vehicle_local_position/out"
            vehicle_global_position_topic = "fmu/vehicle_global_position/out"
            vehicle_odometry_topic = "fmu/vehicle_odometry/out"

        # a subscriber for my state from vehiclelocalposition message
        self.vehicle_local_position_subscriber_ = self.create_subscription(
            VehicleLocalPosition,
            vehicle_local_position_topic,
            self.vehicle_local_position_callback,
            qos_policy,
        )

        # a subscriber for my state from vehicleglobalposition message
        self.vehicle_global_position_subscriber_ = self.create_subscription(
            VehicleGlobalPosition,
            vehicle_global_position_topic,
            self.vehicle_global_position_callback,
            qos_policy,
        )

        # a subscriber for my state from odometry
        self.vehicle_odometry_subscriber_ = self.create_subscription(
            VehicleOdometry,
            vehicle_odometry_topic,
            self.vehicle_odometry_callback,
            qos_policy,
        )

        # a subscriber for the agent states message
        self.state_sub_ = self.create_subscription(
            StateSharingMsg,
            "/state",
            self.state_callback,
            qos_policy,
        )
        # a publisher for the agent state message
        self.state_pub_ = self.create_publisher(
            StateSharingMsg,
            "/state",
            qos_policy,
        )
                
        if self.publishing_enabled:
            # A timer for the state publishing loop
            self.state_timer_ = self.create_timer(
                publishing_timestep,
                self.state_publishing_loop
            )


    # Callback function for the vehicle local position subscriber
    # Updates my state local position and yaw angle
    def vehicle_local_position_callback(self, msg):
        self.behavior.me.update_position(msg.x, msg.y, msg.z)
        # yaw = msg.heading

    # Callback function for the vehicle global position subscriber
    # Updates my state lon lat and alt
    def vehicle_global_position_callback(self, msg):
        self.behavior.me.update_global(msg.lon, msg.lat, msg.alt)

    # Callback function for the vehicle odometry subscriber
    # Updates my state quaternion angles
    def vehicle_odometry_callback(self, msg):
        self.behavior.me.timestamp_ = msg.timestamp
        self.behavior.me.update_quaternion(msg.q)
        # Get roll, pitch, yaw from quaternions and update angles
        roll, pitch, yaw = quaternion_to_euler(msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        self.behavior.me.update_angles(roll, pitch, yaw)

    # callback function for the state subscriber
    # It reads received StateSharingMsg messages and updates the agents list
    def state_callback(self, state):
        # If received self state, skip
        if state.frame_id == self.ident:
            return

        # update the agent
        # print(f"updating agent {state.header.frame_id}")
        self.behavior.update_agent(
            state.frame_id,
            state.global_position_lon,      # gps lon
            state.global_position_lat,      # gps lat
            state.global_position_alt,      # gps alt
            state.yaw,                      # yaw angle in rad
            state.local_position_x,         # local x
            state.local_position_y,         # local y
            state.local_position_z,         # local z
            state.roll,                     # roll
            state.pitch,                    # pitch
            state.q                         # quaternion angles
        )

    # A function that publishes the state of the agent
    def publish_state(self):
        # self.get_logger().info(f"IN PUBLISH STATE!!!")
        if (self.behavior.me.initialized):
            # Create the message and publish it
            msg = StateSharingMsg()
            msg.timestamp = self.behavior.me.timestamp_
            msg.frame_id = self.ident
            msg.global_position_lon = self.behavior.me.lon
            msg.global_position_lat =  self.behavior.me.lat
            msg.global_position_alt = self.behavior.me.alt
            msg.yaw = self.behavior.me.yaw
            msg.roll = self.behavior.me.roll
            msg.pitch = self.behavior.me.pitch
            msg.local_position_x = self.behavior.me.local_position_x
            msg.local_position_y = self.behavior.me.local_position_y
            msg.local_position_z = self.behavior.me.local_position_z
            msg.q = self.behavior.me.q

            self.state_pub_.publish(msg)
        else:
            print("position not initialized, not publishing state")
            return False

    # A loop connected to the publishing timer
    # publishes the state for the agent at time intervals
    def state_publishing_loop(self):
        self.publish_state()
