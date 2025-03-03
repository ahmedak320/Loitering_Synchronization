#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Yaw-N synchronization method.
This method uses minimal state sharing between the drones to achieve synchronization.
We divide the circle into N angles. If N=1 angles=[0] If N=2 angles=[0,180], If N=3 angles=[0,120,240] etc...
Whenever a drone reaches a yaw around those angles, it sends a pulse to the other drones.
Each drone compares its yaw to that angle whenever a pulse is received and changes speeds to get closer to it.
Eventually, they synchronize with each other.

To prevent overshooting, the change in speed is limited by a SPEED_COUNTER_LIMIT.
When the counter reaches the limit, the speed is reset back to cruise speed.
This duration is calculated by SPEED_COUNTER_LIMIT * CONTROL_DELAY in seconds. 

YAW0_RANGE is the yaw range where the yaw is considered close enough to 0 radians to send the pulse.
We will be using it to get the range for all angle points.
A flag prevents multiple pulses to be sent whenever a drone is close to an angle.
"""

# imports for local sync
from lrs_msgs.msg import StateSharingMsg
from .loitering_sync_base_class import LoiteringSyncBase
import rclpy
from math import pi

# Function that gets the shortest difference between two angles
def shortest_angle_difference(angle1, angle2):
    # Compute the difference between the angles
    diff = angle2 - angle1

    # Adjust the difference to be within -π to π range
    diff = (diff + pi) % (2 * pi) - pi

    # Ensure the result is always the shortest difference
    if diff > pi:
        diff -= 2 * pi
    elif diff < -pi:
        diff += 2 * pi

    return diff

class YAW_N_Loitering(LoiteringSyncBase):
    def __init__(self, name, params):#, publishing_enabled=False):
        super().__init__(name, params, publishing_enabled=False)
        self.yaw = 0.0
        self.speed = self.SPEEDS['cruise']
        self.speed_counter = 0
        self.pulse_sent = False
        self.mission_start = False
        # max speed counter is duration to go half a circle at cruise speed divided by the control time step
        self.SPEED_COUNTER_MAX = int((pi*self.LOITER_RADIUS/self.SPEEDS['cruise'])/self.CONTROL_DELAY)
        # get N_YAW pusling angles
        self.pulsing_angles = []
        for i in range(self.N_YAW):
            self.pulsing_angles.append(i*2*pi/self.N_YAW)

        if self.DDS_FLAG:
            qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        else:
            qos_policy = 10

         # a subscriber for the agent states message
        self.state_sub_ = self.create_subscription(
            StateSharingMsg,
            "/state",
            self.state_callback,
            qos_policy
        )

        # initialize the ros node
        self.initialize_ros()

    # unpack parameters needed for this specific sync method
    def unpack_method_params(self, params):
        self.YAW0_RANGE = params['YAW0_RANGE']
        self.SPEED_COUNTER_LIMIT = params['SPEED_COUNTER_LIMIT']
        self.N_YAW = params['N_YAW']
        self.LOITER_RADIUS = params['LOITER_RADIUS']
        self.SPEED_COUNTER_BASE = params['SPEED_COUNTER_BASE']
        self.LEADER_DRONE = params['LEADER_DRONE']
    
    # function to notify others that we reached 0 yaw
    def publish_n_yaw(self):
        # if yaw is at one of the pulsing angles, send a pulse to others
        min_distance = 2*pi
        closest_pulsing_angle = 0
        for angle in self.pulsing_angles:
            distance = shortest_angle_difference(self.yaw, angle)
            if abs(distance) < abs(min_distance):
                min_distance = distance
                closest_pulsing_angle = angle

        if abs(min_distance) < self.YAW0_RANGE:
            if not self.pulse_sent:
                pulse_msg = StateSharingMsg()
                pulse_msg.frame_id = str(self.ident)
                pulse_msg.yaw = closest_pulsing_angle
                self.state_sharing.state_pub_.publish(pulse_msg)
                self.pulse_sent = True
        else:
            self.pulse_sent = False

    # callback function for the state subscriber
    # It reads when a pulse is received and updates the speed
    def state_callback(self, state):
        if self.loitering or self.BACKYARD:
            # If received self state, skip
            if int(state.frame_id) == self.ident:
                return

            distance = shortest_angle_difference(self.yaw, state.yaw)
            if abs(distance) < self.ACCEPTANCE_RANGE/2.0:
                return
            else:
                if distance < 0:
                    self.speed = self.SPEEDS['low']
                else:
                    self.speed = self.SPEEDS['high']
                # if we are close to the target, make speed reset happen faster
                self.SPEED_COUNTER_LIMIT = int((abs(distance)/pi)*self.SPEED_COUNTER_MAX) + self.SPEED_COUNTER_BASE
                self.speed_counter = 0
            self.get_logger().info(f"Drone: {state.frame_id} is at 0 yaw. Speed is: {self.speed}")

    # Function that applies the synchronization principles and calculates the commands for the agent
    def control_loop(self):
        # timestamp used in vehicle command publisher. Good to distinguish different commads.
        self.print_counter += 1
        self.timestamp_ = self.state_sharing.behavior.me.timestamp_

        # starts mission if SIMULATION parameter is true
        if (not self.mission_start) :
            self.mission_start = True
            self.publish_mission_start()

        if self.loitering or self.BACKYARD:
            # check if yaw reached one of the pulsing angles and publish it
            self.yaw = self.state_sharing.behavior.me.yaw
            if self.ident == self.LEADER_DRONE:
                self.publish_n_yaw()

            # speed reset timer
            if self.speed != self.SPEEDS['cruise']:
                self.speed_counter +=1
                if self.speed_counter > self.SPEED_COUNTER_LIMIT:
                    self.speed_counter = 0
                    self.speed = self.SPEEDS['cruise']
            else:
                self.speed_counter = 0

            self.publish_speed(self.speed)
        if self.print_counter >= 100:
            self.get_logger().info(f"setting speed: {self.speed}")
            self.print_counter = 0
