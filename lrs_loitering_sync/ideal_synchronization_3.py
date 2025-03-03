#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Ideal Angle synchronization method 3. This gets the biggest empty arc. Gets its middle point and adds pi to find the ideal meeting point.
This method uses stateSharing to receive the yaw of all the drones, then calculates
the ideal meeting point between them to synchronize.

The states are shared every STATE_PUBLISHING_DT and the control loop gathers all the updated yaws
and passes them to get_speed_ideal() which returns the ideal speed for the current drone to reach
the ideal angle point. 
"""

from .loitering_sync_base_class import LoiteringSyncBase
from math import pi, atan2
import numpy as np


class IdealLoitering3(LoiteringSyncBase):
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

        if self.loitering or self.BACKYARD:
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
            if self.print_counter == 10:
                self.get_logger().info(f"MY YAW List: {yaw_list}")

            speed, phase = get_speed_ideal(yaw_list, self.SPEEDS, self.ACCEPTANCE_RANGE)
            self.publish_speed(speed)
            if self.print_counter >= 10:
                self.get_logger().info(f"desired angle: {phase}, current angle: {my_yaw} setting speed: {speed}")
                # self.get_logger().info(f"loitering distance: {distance}, setting speed: {speed}, direction to move CW?: {drone_speedup}")
                # self.get_logger().info(f"setting speed: {speed}")
                self.print_counter = 0

# returns argmax of list
def argmax(iterable):
    return max(enumerate(iterable), key=lambda x: x[1])[0]

# returns argmin of list
def argmin(iterable):
    return min(enumerate(iterable), key=lambda x: x[1])[0]

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
    phase = get_ideal_angle_3(yaw_list)
    distance, drone_speedup = min_angle_distance(my_yaw, phase)
    # divide acceptance range by 2 to allow for +/- range from the middle angle
    if distance < ACCEPTANCE_RANGE/2.0:
        speed = SPEEDS['cruise']
    else:
        if drone_speedup:
            speed = SPEEDS['high']
        else: 
            speed = SPEEDS['low']
    
    return speed, phase


# Function that finds the mean angle 
def get_ideal_mean_angle(yaws):
    yaws = np.array(yaws)
    x = sum(np.cos(yaws))/len(yaws)
    y = sum(np.sin(yaws))/len(yaws)
    return atan2(y,x)

# Function that gets the shortest difference between two angles
def shortest_angle_difference(angle1, angle2):
    # Compute the difference between the angles
    diff = angle2 - angle1

    # Adjust the difference to be within -π to π range
    # Ensures the result is always the shortest difference
    diff = (diff + pi) % (2 * pi) - pi

    if diff > 0:
        diff2 = diff - 2*pi
    else:
        diff2 = diff + 2*pi

    return diff, diff2

# Function that gets the proposed ideal angle for the drones to meet using max empty arc and reversing it
# 1- For each drone calculate distance to the closest neighbor behind and the closest one in front
# 2- Take maximum of these distances and the two drones that create it
# 3- Find the middle of the found arc between them and take this point + pi as the ideal point
# 4- If the biggest distance is bigger than pi.. they are all on the same side and the middle of the arc is the desired angle
def get_ideal_angle_3(yaws):
    distance_list = []
    yaw_index_list = []
    
    for i in range(len(list(yaws))):
        min_pos_distance = float('inf')
        min_neg_distance = float('inf')
        min_pos_yaw_index = 0
        min_neg_yaw_index = 0
        for j in range(len(list(yaws))):
            if i != j:
                distance, distance2 = shortest_angle_difference(yaws[i], yaws[j])
                if distance >= 0:
                    if distance <= min_pos_distance:
                        min_pos_distance = distance
                        min_pos_yaw_index = j
                else:
                    if abs(distance) < min_neg_distance:
                        min_neg_distance = abs(distance)
                        min_neg_yaw_index = j
                
                if distance2 >= 0:
                    if distance2 <= min_pos_distance:
                        min_pos_distance = distance2
                        min_pos_yaw_index = j
                else:
                    if abs(distance2) < min_neg_distance:
                        min_neg_distance = abs(distance2)
                        min_neg_yaw_index = j

        distance_list.append(min_pos_distance)
        distance_list.append(min_neg_distance)
        yaw_index_list.append([i,min_pos_yaw_index])
        yaw_index_list.append([i,min_neg_yaw_index])

    max_empty_distance = max(distance_list)
    max_empty_distance_index = argmax(distance_list)

    temp_yaw_list_index = yaw_index_list[max_empty_distance_index]
    temp_yaw_list = [yaws[temp_yaw_list_index[0]],yaws[temp_yaw_list_index[1]]]
    desired_angle = get_ideal_mean_angle(temp_yaw_list)
    if max_empty_distance < np.pi:
        desired_angle += np.pi

    return desired_angle