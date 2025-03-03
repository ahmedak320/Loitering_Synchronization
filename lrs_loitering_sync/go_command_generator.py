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
from lrs_msgs.msg import GoCommand


# A class that publishes a go command for the drones
class Go_Commander(Node):
    def __init__(self, command, source=0):
        super().__init__("Go_Commander")
        self.timer = self.create_timer(1, self.publish_go_command)
        self.command = command
        self.source = source
   
        # qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                        #   history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                        #   depth=1)

        # a publisher for the agent phase offset message
        self.go_command_pub_ = self.create_publisher(
            GoCommand,
            "/go_command",
            10,
        )
    
    # A function that publishes the go command for the drones
    def publish_go_command(self):
        for i in range(3):
            msg = GoCommand()
            msg.timestamp = 0
            msg.id = self.source
            msg.command = self.command
            self.go_command_pub_.publish(msg)
            self.get_logger().info('Publishing go command!')
            time.sleep(0.2)

        self.timer.cancel()
        time.sleep(1)
        self.destroy_node()  # Destroy the node explicitly

def main():
    if len(sys.argv) < 2:
        print("Usage: go_command <command>. 1: start trajectory. 2: reset loitering sync")
        return

    command = int(sys.argv[1])  # Convert the first argument passed to an integer

    try:
        rclpy.init()  # initialize ros
        go_command_node = Go_Commander(command)
        rclpy.spin_once(go_command_node, timeout_sec=1)
    finally:
        print("Closing")
        go_command_node.destroy_node()

if __name__ == "__main__":
    main()