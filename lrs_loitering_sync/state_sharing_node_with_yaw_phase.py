#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
This is a modified state sharing node that contains the same things as the default stateSharing. Except, it modifies the 
self odometry update of the yaw to include a phase offset. 
"""

from .state_sharing_node import StateSharing
from lrs_msgs.msg import PhaseOffset

from math import atan2, asin, pi

# # A function that converts quaternion angles to euler
# # Gotten from chatgpt, verified
def quaternion_to_euler(qw, qx, qy, qz):
    roll = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    pitch = asin(2*(qw*qy - qz*qx))
    yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return roll, pitch, yaw


# Updated stateSharing class with yaw offset included in the odometry update
class StateSharing_withOffset(StateSharing):
    def __init__(self, publishing_enabled=True, publishing_timestep=1.0, control_frequency=5, DDS_FLAG=True, ESTIMATION_FLAG=False, ESTIMATION_METHOD='foh', yaw_offset=0):
        super().__init__(publishing_enabled, publishing_timestep, control_frequency, DDS_FLAG, ESTIMATION_FLAG, ESTIMATION_METHOD)
        self.yaw_offset = yaw_offset*pi/180.0
        print("starting with yaw offset: ", yaw_offset)
        self.real_yaw = 0

        # a subscriber for the agent states message
        self.phase_offset_sub_ = self.create_subscription(
            PhaseOffset,
            "/phase_offset",
            self.phase_offset_callback,
            10
        )

        # a publisher for the agent phase offset message
        self.phase_offset_pub_ = self.create_publisher(
            PhaseOffset,
            "/phase_offset",
            10,
        )

    # Callback function for the vehicle odometry subscriber
    # Updates my state quaternion angles
    def vehicle_odometry_callback(self, msg):
        self.behavior.me.timestamp_ = msg.timestamp
        self.behavior.me.update_quaternion(msg.q)
        # Get roll, pitch, yaw from quaternions and update angles
        roll, pitch, yaw = quaternion_to_euler(msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        self.behavior.me.real_yaw = yaw
        self.behavior.me.update_angles(roll, pitch, yaw - self.yaw_offset)
    
    def update_yaw_offset(self, new_yaw_offset):
        # update yaw offset
        new_yaw_offset = new_yaw_offset*pi/180
        if self.yaw_offset == new_yaw_offset:
            return True

        self.yaw_offset = new_yaw_offset

        # update yaw value to reflect new offset
        offset_diff = self.yaw_offset - new_yaw_offset
        new_yaw = self.behavior.me.yaw + offset_diff
        self.behavior.me.update_angles(self.behavior.me.roll, self.behavior.me.pitch, new_yaw)
        return False

    # callback function for the phase_offset subscriber
    # It reads received PhaseOffset messages and updates the drone's phase offset
    def phase_offset_callback(self, msg):
        if msg.timestamp == 0:
            # If received self phase offset, update it
            if msg.frame_id == self.ident:
                received_before = self.update_yaw_offset(msg.phase_offset)
                if received_before:
                    print("New phase already used!")
                else:
                    print("New phase offset is: ", self.yaw_offset)
                self.publish_phases(msg.phase_offset)

    # A function that publishes the new phases from a list of phases
    # timestamp is made self.ident to indicate that this is coming from the drone
    # so the timestamp field is the source of the msg
    def publish_phases(self, new_phase):
        # create msg:
        msg = PhaseOffset()
        msg.timestamp = int(self.ident)
        msg.frame_id = self.ident
        msg.phase_offset = float(new_phase)
        self.phase_offset_pub_.publish(msg)