#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
This is a file that has a simple function that sends a go command to the drones. 
It basically switches the mission waypoint to waypoint 3 (just after loitering synchronization)
This code could be adjusted for other commands to be given to the drones

"""
import sys
import rclpy
from rclpy.node import Node
import time
import yaml
from lrs_msgs.msg import PhaseOffset
parameter_filepath = 'lrs_workspace/src/lrs_modules/lrs_loitering_sync/lrs_loitering_sync/'
# Load the YAML file
with open(parameter_filepath + 'loitering_sync_params.yml') as fh:
    # Convert the YAML data into a dictionary
    PARAMS = yaml.safe_load(fh)

# A class that publishes a phase offset for the drones
class Phase_Offset_Publisher(Node):
    def __init__(self, phases):
        super().__init__("Phase_Offset_Publisher")
        self.timer = self.create_timer(1, self.publish_phases)
        self.new_phases = phases
   
        # qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                        #   history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                        #   depth=1)

        # a publisher for the agent phase offset message
        self.phase_offset_pub_ = self.create_publisher(
            PhaseOffset,
            "/phase_offset",
            10,
        )
    
    # A function that publishes the go command for the drones
    def publish_phases(self):
        drone_ids = list(PARAMS['DRONE_IDS'].keys())
        updated_phases = [False] * len(drone_ids)
        if len(self.new_phases) == len(drone_ids):
            for i in range(len(self.new_phases)):
                if not updated_phases[i]:
                    # create msg:
                    msg = PhaseOffset()
                    msg.timestamp = 0
                    msg.frame_id = drone_ids[i]
                    msg.phase_offset = float(self.new_phases[i])
                    self.phase_offset_pub_.publish(msg)
                    time.sleep(0.05)
        else:
            print(f"Please provide phases for {len(drone_ids)} drones. You provided {len(self.new_phases)} phases.")

        self.timer.cancel()
        time.sleep(1)
        self.destroy_node()  # Destroy the node explicitly

def main():
    if len(sys.argv) < 2:
        print("Usage: phase_offset_publisher phase1 phase2 phase3 ...")
        return

    phases = [float(phase) for phase in sys.argv[1:]]  # Convert all additional arguments to floats

    try:
        rclpy.init()  # initialize ros
        phase_offset_publisher_node = Phase_Offset_Publisher(phases)
        # rclpy.spin(phase_offset_publisher_node)
        rclpy.spin_once(phase_offset_publisher_node, timeout_sec=1)
    finally:
        print("Closing")
        phase_offset_publisher_node.destroy_node()

if __name__ == "__main__":
    main()