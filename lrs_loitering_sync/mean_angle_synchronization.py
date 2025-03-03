#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Mean Angle synchronization method.
This method uses stateSharing to receive the yaw of all the drones, then calculates
the mean angle as the meeting point between them to synchronize.

The states are shared every STATE_PUBLISHING_DT and the control loop gathers all the updated yaws
and passes them to get_speed_ideal() which returns the ideal speed for the current drone to reach
the mean angle point. 
"""

from .loitering_sync_base_class import LoiteringSyncBase
from math import pi, atan2
import numpy as np
from .go_command_generator import Go_Commander
import rclpy

# Function that finds the mean angle 
def get_ideal_mean_angle(yaws):
    yaws = np.array(yaws)
    x = sum(np.cos(yaws))/len(yaws)
    y = sum(np.sin(yaws))/len(yaws)
    return atan2(y,x)

# gets the minimum arc distance between the two angles
# also returns a check of whether drone 1 needs to speed up or slow down
# Steps: we convert the angles to 0-2pi. we check angle1-angle2. 
#        If negative and abs is more than pi: drone1 speed up false. distance is 2pi - abs(diff). 
#        If negative and abs is less than pi: drone1 speed up true. distance is abs(diff)
#        If positive and less than pi. drone1 speed up false. distance is diff
#        If positive and more than pi. drone1 speed up true. distance is 2pi - diff
#       optimize the conditions by grouping more than pi vs less than pi condition first.
#        If abs is more than pi and negative: drone1 speed up false. distance is 2pi - abs(diff). 
#        If abs more than pi and positive. drone1 speed up true. distance is 2pi - diff
#        If abs is less than pi and negative: drone1 speed up true. distance is abs(diff)
#        If abs less than pi and positive. drone1 speed up false. distance is diff
def min_angle_distance(angle1, angle2):
    drone1_speedup = True
    # change angle range to 0-2pi
    angle1 %= 2*pi
    angle2 %= 2*pi

    diff = angle1 - angle2
    if abs(diff) < pi:
        distance = abs(diff)
        if diff > 0:
            drone1_speedup = False
    else:
        distance = 2*pi - abs(diff)
        if diff < 0:
            drone1_speedup = False
       
    return distance, drone1_speedup

# Function that returns the speed to be used for the drone
def get_speed_ideal(yaw_list, SPEEDS, ACCEPTANCE_RANGE):
    my_yaw = yaw_list[-1]
    phase = get_ideal_mean_angle(yaw_list)
    distance, drone_speedup = min_angle_distance(my_yaw, phase)
    # divide acceptance range by 2 to allow for +/- range from the middle angle
    if distance < ACCEPTANCE_RANGE/2.0:
        speed = SPEEDS['cruise']
    else:
        if drone_speedup:
            speed = SPEEDS['high']
        else: 
            speed = SPEEDS['low']
    
    return speed

class MeanAngleLoitering(LoiteringSyncBase):
    def __init__(self, name, params):
        super().__init__(name, params, publishing_enabled=True)
        self.mission_start = False
        # initialize the ros node
        self.initialize_ros()

    # Function that gets the ideal speed based on the ideal angle calculations and publishes it
    def control_loop(self):
        # timestamp used in vehicle command publisher. Good to distinguish different commads.
        self.timestamp_ = self.state_sharing.behavior.me.timestamp_

        if (not self.mission_start) :
            self.mission_start = True
            self.publish_mission_start()            

        if (self.loitering or self.BACKYARD):
            self.print_counter += 1
            agents = self.state_sharing.behavior.agents
            yaw_list = []
            # Get all the yaws from the agents including me
            for key in agents:
                # keep yaw in 0-2pi range
                yaw = (agents[key].yaw + 2*pi) % (2*pi)
                yaw_list.append(yaw)
            my_yaw = (self.state_sharing.behavior.me.yaw + 2*pi) % (2*pi)
            # include current drone yaw in list
            yaw_list.append(my_yaw)

            speed = get_speed_ideal(yaw_list, self.SPEEDS, self.ACCEPTANCE_RANGE)

            if self.TRAJECTORY_FLAG:
                if not self.synchronized:
                    # check if it reached the right place and just needs to cruise. 
                    # check if it maintains the synchronized status for 10 cycles
                    # if yes, flag synchronized true and stop synchronization algo
                    if speed == self.SPEEDS['cruise']:
                        self.synchronized_counter += 1
                        if self.synchronized_counter >= 5:
                            self.synchronized = True
                            self.synchronized_counter = 0
                    else:
                        self.synchronized_counter = 0
                else:
                    speed == self.SPEEDS['cruise']

            self.publish_speed(speed)
            if self.print_counter >= 10:
                self.print_counter = 0
                if self.synchronized:
                    self.get_logger().info(f"maintaining cruise speed: {speed}")
                    go_command_node = Go_Commander(100, self.ident) # Send the command using go commander
                    rclpy.spin_once(go_command_node, timeout_sec=1)
                    go_command_node.destroy_node()
                    self.loitering = False # stop loitering
                else:
                    self.get_logger().info(f"MY YAW List: {yaw_list}")
                    # self.get_logger().info(f"loitering delay: {delay}, setting speed: {speed}")
                    # self.get_logger().info(f"loitering distance: {distance}, setting speed: {speed}, direction to move CW?: {drone_speedup}")
                    self.get_logger().info(f"setting speed: {speed}")
