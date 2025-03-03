#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Yaw-0 synchronization method. With speed steps the increase/decrease with each pulse
This method uses minimal state sharing between the drones to achieve synchronization.
Whenever a drone reaches a yaw of 0 radians, it sends a pulse to the other drones.
Each drone compares its yaw to 0 radians whenever a pulse is received and changes speeds to get closer to it.
Eventually, they synchronize with each other.

To prevent overshooting, the change in speed is limited by a SPEED_COUNTER_LIMIT.
When the counter reaches the limit, the speed is reset back to cruise speed.
This duration is calculated by SPEED_COUNTER_LIMIT * CONTROL_DELAY in seconds. 

YAW0_RANGE is the yaw range where the yaw is considered close enough to 0 radians to send the pulse.
A flag prevents multiple pulses to be sent whenever a drone is close to 0.

Does not work with 10 drones.
"""

# imports for local sync
from lrs_msgs.msg import StateSharingMsg
from .loitering_sync_base_class import LoiteringSyncBase
import rclpy
import time

class RAMPYAW0Loitering(LoiteringSyncBase):
    def __init__(self, name, params):#, publishing_enabled=False):
        super().__init__(name, params, publishing_enabled=False)
        self.yaw = 0.0
        self.speed = self.SPEEDS['cruise']
        self.speed_counter = 0
        self.yaw0_reached = False
        self.mission_start = False
        self.pulse_timestamp = [-1]*self.NUM_DRONES
        self.pulse_magnitude = [0]*self.NUM_DRONES
        self.SPEED_STEP = 0.5

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
        self.NUM_DRONES = params['NUM_DRONES']
    
    # function to notify others that we reached 0 yaw
    def publish_0yaw(self):
        # if yaw is 0, send a pulse to others
        if abs(self.yaw) < self.YAW0_RANGE:
            if not self.yaw0_reached:
                msg_reached0 = StateSharingMsg()
                msg_reached0.frame_id = str(self.ident)
                self.state_sharing.state_pub_.publish(msg_reached0)
                self.yaw0_reached = True
        else:
            self.yaw0_reached = False

    # callback function for the state subscriber
    # It reads when a pulse is received and updates the speed
    def state_callback(self, state):
        # If received self state, skip
        if int(state.frame_id) == self.ident:
            return
        if abs(self.yaw) < self.ACCEPTANCE_RANGE/2.0:
            self.speed = self.SPEEDS['cruise'] + sum(self.pulse_magnitude)
            return
        else:
            drone_id = int(state.frame_id) - 1
            self.pulse_timestamp[drone_id] = time.time()
            if self.yaw > 0:
                self.pulse_magnitude[drone_id] = -self.SPEED_STEP
            else:
                self.pulse_magnitude[drone_id] = self.SPEED_STEP

            self.speed = self.SPEEDS['cruise'] + sum(self.pulse_magnitude)
            self.get_logger().info(f"Drone: {state.frame_id} is at 0 yaw. Speed is: {self.speed}")

    # Function that applies the synchronization principles and calculates the commands for the agent
    def control_loop(self):
        # timestamp used in vehicle command publisher. Good to distinguish different commads.
        self.timestamp_ = self.state_sharing.behavior.me.timestamp_

        if (not self.mission_start) :
            self.mission_start = True
            self.publish_mission_start()

        if self.loitering or self.BACKYARD:
            current_time = time.time()
            # check if yaw reached 0 and publish it
            self.yaw = self.state_sharing.behavior.me.yaw
            self.publish_0yaw()

            # speed reset loop
            reset_duration = self.SPEED_COUNTER_LIMIT*self.CONTROL_DELAY
            for i in range(len(self.pulse_timestamp)):
                if ((current_time - self.pulse_timestamp[i]) > reset_duration) and (self.pulse_timestamp[i] > 0):
                    self.pulse_timestamp[i] = -1
                    self.pulse_magnitude[i] = 0.0

            self.speed = self.SPEEDS['cruise'] + sum(self.pulse_magnitude)
            # include speed saturation
            if self.speed > self.SPEEDS['high']:
                self.speed = self.SPEEDS['high']
            elif self.speed < self.SPEEDS['low']:
                self.speed = self.SPEEDS['low']



            speed = self.speed
            self.publish_speed(speed)
            self.print_counter += 1
            if self.print_counter >= 100:
                self.get_logger().info(f"setting speed: {self.speed}")
                self.print_counter = 0
