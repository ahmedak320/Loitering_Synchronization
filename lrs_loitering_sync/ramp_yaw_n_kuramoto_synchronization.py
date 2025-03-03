#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Yaw-N synchronization method with ramp and kuramoto speed change.
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
from math import pi, sin

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

# function that finds closest pulse location to angle:
def get_nearest_pulse_location(yaw, pulse_locations):
    min_distance = float('inf')
    min_index = 0
    for i, pulse_location in enumerate(pulse_locations):
        distance = abs(shortest_angle_difference(yaw, pulse_location)[0])
        if distance < min_distance:
            min_distance = distance
            min_index = i
    
    return min_index

class YAW_N_Loitering_RAMP_KURAMOTO(LoiteringSyncBase):
    def __init__(self, name, params):#, publishing_enabled=False):
        super().__init__(name, params, publishing_enabled=False)
        self.yaw = 0.0
        self.speed = self.SPEEDS['cruise']
        self.speed_counter = 0
        self.pulse_sent = False
        self.mission_start = False
        # max speed counter is duration to go half a circle at cruise speed divided by the control time step
        self.SPEED_COUNTER_MAX = int((pi*self.LOITER_RADIUS/self.SPEEDS['cruise'])/self.CONTROL_DELAY)
        self.SPEED_RESET_DURATION_MAX = (2*pi*self.LOITER_RADIUS/self.SPEEDS['cruise'])/self.N_YAW # either 1,2,or 4 pulse locations
        self.SPEED_RESET_DURATION = self.SPEED_RESET_DURATION_MAX
        # get N_YAW pusling angles
        self.pulse_locations = [i*2*pi/self.N_YAW for i in range(self.N_YAW)]
        # get list of neighbors
        self.neighbors = [0 for i in range(self.NUM_DRONES)]
        self.neighbors[self.ident-1]=1
        self.pulse_magnitudes = [0 for i in range(self.NUM_DRONES)]
        self.pulse_timestamps = [-1 for i in range(self.NUM_DRONES)]
        self.time = 0

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
        self.YAW0_RANGE = 0.3*self.CONTROL_DELAY#params['YAW0_RANGE']
        self.SPEED_COUNTER_LIMIT = params['SPEED_COUNTER_LIMIT']
        self.N_YAW = params['N_YAW']
        self.LOITER_RADIUS = params['LOITER_RADIUS']
        self.SPEED_COUNTER_BASE = params['SPEED_COUNTER_BASE']
        self.NUM_DRONES = params['NUM_DRONES']
    
    # function to notify others that we reached 0 yaw
    def publish_n_yaw(self):
        # if yaw is at one of the pulsing angles, send a pulse to others
        closest_pulse_index = get_nearest_pulse_location(self.yaw, self.pulse_locations)
        distance = shortest_angle_difference(self.yaw, self.pulse_locations[closest_pulse_index])[0]

        if abs(distance) < self.YAW0_RANGE:
            if not self.pulse_sent:
                pulse_msg = StateSharingMsg()
                pulse_msg.frame_id = self.ident_key
                pulse_msg.yaw = float(closest_pulse_index)
                self.state_sharing.state_pub_.publish(pulse_msg)
                self.pulse_sent = True
        else:
            self.pulse_sent = False

    # callback function for the state subscriber
    # It reads when a pulse is received and updates the speed
    def state_callback(self, state):
        if self.loitering or self.BACKYARD:
            # If received self state, skip
            if state.frame_id == self.ident_key:
                return

            # get updated yaw before starting:
            self.yaw = self.state_sharing.behavior.me.yaw

            my_distance = shortest_angle_difference(self.yaw, self.pulse_locations[int(state.yaw)])[0]
            # update neighbor matrix
            if abs(my_distance) < pi/4:
                self.neighbors[self.DRONE_IDS[state.frame_id]-1] = 1
            else:
                self.neighbors[self.DRONE_IDS[state.frame_id]-1] = 0
            self.get_logger().info(f"Neighbors: {self.neighbors} total neighbors: {sum(self.neighbors)} pulse_duration: {self.SPEED_RESET_DURATION}")
            # get pulse magnitude and time
            if  abs(my_distance) >= self.ACCEPTANCE_RANGE:
                self.pulse_timestamps[self.DRONE_IDS[state.frame_id]-1] = self.time
                self.pulse_magnitudes[self.DRONE_IDS[state.frame_id]-1] = 2.0 * sin(self.pulse_locations[int(state.yaw)] - self.yaw)
                self.get_logger().info(f"Drone: {self.DRONE_IDS[state.frame_id]} is at pulse: {state.yaw} with angle: {self.pulse_locations[int(state.yaw)]} . my_yaw: {self.yaw} Pulse is: {self.pulse_magnitudes[self.DRONE_IDS[state.frame_id]-1]}")

            # self.get_logger().info(f"Drone: {self.DRONE_IDS[state.frame_id]} is at pulse: {state.yaw}. my_yaw: {self.yaw} Pulse is: {self.pulse_magnitudes[self.DRONE_IDS[state.frame_id]-1]}")

    # Function that applies the synchronization principles and calculates the commands for the agent
    def control_loop(self):
        # timestamp used in vehicle command publisher. Good to distinguish different commads.
        self.time += self.CONTROL_DELAY
        self.print_counter += 1
        self.timestamp_ = self.state_sharing.behavior.me.timestamp_

        # starts mission if SIMULATION parameter is true
        if (not self.mission_start) :
            self.mission_start = True
            self.publish_mission_start()

        if self.loitering or self.BACKYARD:
            # check if yaw reached one of the pulsing angles and publish it
            self.yaw = self.state_sharing.behavior.me.yaw
            self.publish_n_yaw()

            # if the drones are closer, switch to lower duration pulses
            total_neighbors = sum(self.neighbors)
            if total_neighbors > 0.8*self.NUM_DRONES:
                self.SPEED_RESET_DURATION = 0.5*pi*self.LOITER_RADIUS/self.SPEEDS['cruise']
            else:
                self.SPEED_RESET_DURATION = self.SPEED_RESET_DURATION_MAX

            # reset the pulses
            for i in range(self.NUM_DRONES):
                if ((self.time - self.pulse_timestamps[i]) >= self.SPEED_RESET_DURATION) and (self.pulse_timestamps[i]>0):
                    self.pulse_magnitudes[i] = 0.0
                    self.pulse_timestamps[i] = -1

            self.speed = self.SPEEDS['cruise'] + sum(self.pulse_magnitudes)
            if self.speed > self.SPEEDS['high']:
                self.speed  = self.SPEEDS['high']
            elif self.speed  < self.SPEEDS['low']:
                self.speed  = self.SPEEDS['low']

            self.publish_speed(self.speed)
            # if self.print_counter >= 100:
            self.get_logger().info(f"pulse timestamps: {self.pulse_timestamps}")
            self.get_logger().info(f"pulse magnitudes: {self.pulse_magnitudes}")
            self.get_logger().info(f"Total pulses is: {sum(self.pulse_magnitudes)} setting speed: {self.speed}")
            self.print_counter = 0




# # Function that implements change in speed based on pulses with drones reaching yaw 0
# # 1- Go through all the angles and see if any are close to 0
# # 2- If a group is at 0, change speed as if it's only one drone
# # 3- depending on location, speed up or slow down by a step
# def get_ramp_yaw_n_kuramoto_pulses(t):
#     global yaw0_timestamps
#     global pulse_magnitudes
#     global pulse_timestamps
#     global set_speeds
#     # global neighbors_matrix
#     # global SPEED_RESET_DURATION
#     # YAW0_RANGE = 0.3*dt
#     # SPEED_STEP = 0.5
#     # if YAW_N_PULSE_NUM == 1:
#     #     SPEED_RESET_DURATION_MAX = 2*np.pi*r/SPEEDS['cruise'] # 100% of the cycle duration
#     # elif YAW_N_PULSE_NUM == 2:
#     #     SPEED_RESET_DURATION_MAX = np.pi*r/SPEEDS['cruise'] # 50% of the cycle duration
#     # else:
#     #     SPEED_RESET_DURATION_MAX = 0.5*np.pi*r/SPEEDS['cruise'] # 25% of the cycle duration
#     # pulse_locations = np.arange(YAW_N_PULSE_NUM)*2*np.pi/YAW_N_PULSE_NUM

#     for i in range(N):
#         # use neighbors number to adjust pulse duration
#         # total_neighbors = sum(neighbors_matrix[i])
#         # if total_neighbors <= 8:
#         #     SPEED_RESET_DURATION = SPEED_RESET_DURATION_MAX
#         # else:
#         #     SPEED_RESET_DURATION = 0.5*np.pi*r/SPEEDS['cruise'] # 25% of the cycle duration
#         # SPEED_RESET_DURATION = 0.25*SPEED_RESET_DURATION_MAX

#         # angles = known_angles[i,:]
#         # change all angles to -pi to pi range
#         # angles = (angles + pi) % (2 * pi) - pi
#         # my_yaw = angles[i]
#         for j, angle in enumerate(angles):
#             # if i == j:
#             #     continue
#             # closest_pulse_index = get_nearest_pulse_location(angle, pulse_locations)
#             # distance = abs(shortest_angle_difference(angle, pulse_locations[closest_pulse_index])[0])
#             # if distance < YAW0_RANGE:
#                 # change set speed based on my_yaw
#                 # Adjust my yaw to be within -π to π range
#                 # my_distance = abs(shortest_angle_difference(my_yaw, pulse_locations[closest_pulse_index])[0])
#                 # # update neighbor matrix
#                 # if my_distance < np.pi/4:
#                 #     neighbors_matrix[i,j] = 1
#                 # else:
#                 #     neighbors_matrix[i,j] = 0

#                 # get pulse magnitude and time
#                 # if  my_distance >= ACCEPTANCE_RANGE/2.0:
#                 #     pulse_timestamps[i,j] = t
#                 #     pulse_magnitudes[i,j] = 2.0 * sin(pulse_locations[closest_pulse_index] - my_yaw)

#             # if ((t - pulse_timestamps[i,j]) >= SPEED_RESET_DURATION) and (pulse_timestamps[i,j]>0):
#             #     pulse_magnitudes[i,j] = 0.0
#             #     pulse_timestamps[i,j] = -1

#         set_speeds[i] = SPEEDS['cruise'] + sum(pulse_magnitudes[i,:])
#         if set_speeds[i] > SPEEDS['high']:
#             set_speeds[i] = SPEEDS['high']
#         elif set_speeds[i] < SPEEDS['low']:
#             set_speeds[i] = SPEEDS['low']





