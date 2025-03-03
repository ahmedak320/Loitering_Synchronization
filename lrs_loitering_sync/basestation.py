#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
This script continuously listens for user input to control drone operations
with specific commands that influence the behavior of the loitering synchronization.
"""

import sys
import rclpy
from .go_command_generator import Go_Commander
from .phase_offset_publisher import Phase_Offset_Publisher
from .loitering_sync_metrics import *
import numpy as np
import threading
from lrs_msgs.msg import GoCommand

"""
missing:
skip to trajectory needs an exit while metrics is running. Currently the exit happens after each method trial with a continue? question.
Needs metrics to run on a separate thread or run on a separate thread. Then send command = 6 to exit during metrics
testing
"""
class BaseStation(Node):
    def __init__(self):
        super().__init__("base_station")
        self.current_state = 0
        self.ids = []
        self.commands = {
            "abort": 0,          # Command code for abort
            "next_method": 1,    # Command code for next method
            "help": 2,           # Command code for help
            "trajectory": 3,     # Command code for starting trajectory
            "start_trials": 4,   # Command code for starting trials in Metrics
            "start_trials_2": 5,   # Command code for starting trials part 2 in Metrics
            "send_phases": 6,    # Command code for sending phases using phase offset
            "go": 7,             # Command code to start trajectory (send when in position)
            "trajectory_off": 8, # Command code to turn off trajectory flag
        }

        if PARAMS['SIMULATION'] == True:
            self.cruise_speed = PARAMS['SIMULATION_SPEEDS']['cruise']
        else:
            self.cruise_speed = PARAMS['REAL_SPEEDS']['cruise']

        # Initialize subscriber to listen to GoCommand messages
        self.subscription = self.create_subscription(
            GoCommand,
            "/go_command",
            self.go_command_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def go_command_callback(self, msg):
        # Process received GoCommand messages
        if msg.id != 0:  # Ignore messages with id of 0
            if msg.id not in self.ids:
                self.ids.append(msg.id)
                self.get_logger().info(f'Received GoCommand with id: {msg.id}')

    def run(self):
        while True:
            user_input = input("Enter command ('exit', 'abort', 'help', 'trajectory', 'go', 'start_trials', 'start_trials_2', 'next_method', 'send_phases'): ").strip().lower()
            if user_input == "exit":
                print("Exiting...")
                break

            if not (user_input in self.commands):
                print("Unknown command. Type 'help' for list of commands.")
            elif user_input == "help":
                self.print_help()
            elif user_input == "abort":
                self.send_command(5) # Send the abort command using go commander
            elif user_input == "next_method":
                if self.current_state < 2:
                    self.send_command(0) # Send the next mission command using go commander
                    self.current_state += 1 # up state to the next one
                else:
                    print("Already at last method (mean). run ansible to stop and restart docker and restart this code to have more options.")
                # else:
                #     user_input = input("This command will shut down loitering sync on all drones. are you sure? (yes, no): ").strip().lower()
                #     if user_input == "yes":
                #         self.send_command(0) # Send the next mission command using go commander
                #         print("Since loitering sync is off, this code will exit. Run ansible command to start code on drones then run this again.")
                #         raise SystemExit
            elif user_input == "start_trials":
                # if trajectory flag is on, turn it off
                self.send_command(7)

                methods = ['ideal3', 'ramp_yaw_n_kuramoto', 'mean']
                for i, method in enumerate(methods):
                    if self.current_state > i:
                        print(f"method: {method} is skipped from the trial. If you want it, please stop loitering sync on the drones using ansible and run this again.")
                        continue
                    self.start_trials(method)
                    if self.current_state < 2:
                        user_input = ''
                        while user_input not in ['yes', 'no']:
                            user_input = input("Continue trials with next method? (yes, no): ").strip().lower()
                        if user_input == "no":
                            print("starting trajectory now!")
                            break
                        self.send_command(0) # send the next mission command using go commander
                        self.current_state += 1
                    else:
                        print("Continuing with trajectory following positioning.")
                self.start_trajectory()
            elif user_input == "start_trials_2":
                # if trajectory flag is on, turn it off
                self.send_command(7)

                methods = ['ideal3', 'ramp_yaw_n_kuramoto', 'mean']
                for i, method in enumerate(methods):
                    if self.current_state > i:
                        print(f"method: {method} is skipped from the trial. If you want it, please stop loitering sync on the drones using ansible and run this again.")
                        continue

                    PARAMS['OUTPUT_PATH'] = + PARAMS['OUTPUT_PATH'] + 'trial_2'
                    self.start_trials(method, random_seed=2) # change random seed for second trials
                    if self.current_state < 2:
                        user_input = input("Continue trials with next method? (yes, no): ").strip().lower()
                        if user_input == "no":
                            print("starting trajectory now!")
                            break                    
                        self.send_command(0) # send the next mission command using go commander
                        self.current_state += 1
                self.start_trajectory()
            elif user_input == "go":
                self.send_command(1) # Send go command for trajectory start
            elif user_input == "trajectory":
                # while user_input not in ['yes', 'cancel']: # keep waiting until the drones are loitering
                #     user_input = input(f"Are all drones loitering right now? (yes, no, cancel): ").strip().lower()
                # if user_input == "yes":
                self.start_trajectory()
            elif user_input == "trajectory_off":
                self.send_command(7) # turns off trajectory flag
            elif user_input == "send_phases":
                num_drones = PARAMS["NUM_DRONES"]
                input_list = input(f"Please enter {num_drones} phases: ").strip().split()
                # Check if the number of inputs matches the expected number
                if len(input_list) != PARAMS["NUM_DRONES"]:
                    print(f"Error: You must enter exactly {PARAMS['NUM_DRONES']} phases.")
                else:
                    phases = np.array(input_list).astype(float)
                    self.send_phases(phases)

            else:
                print("Unknown command. Type 'help' for list of commands.")

    def print_help(self):
        help_text = """
        Available Commands:
            - help: Display this help message.
            - trajectory: Start trajectory following. Gets in position only.
            - abort: Stop all operations and shut down.
            - start_trials: Start metrics node trial 1
            - start_trials_2: Start metrics node trial 2
            - exit: Exit the command center.
            - go: use after trajectory or trial is done. Sends the command to start the trajectory part of the mission.
            - next_method: Advance to the next synchronization method.
            - trajectory_off: Turns off the trajectory flag.
            - send_phases: sends the desired phases to the drones
        """
        print(help_text)
    
    def start_trajectory(self):
        # skip to the mean method (last method)
        while self.current_state < 2:
            self.send_command(0) # Send the next mission command using go commander
            self.current_state += 1 # up state to the next one

        # update the phases for trajectory
        phase_offset = -(self.cruise_speed * PARAMS["TRAJECTORY_DT"] / PARAMS["LOITER_RADIUS"]) * (180.0 / np.pi)
        phases = np.arange(PARAMS["NUM_DRONES"]) * phase_offset
        print(f"Publishing the following phases: {phases}")
        self.send_phases(phases)

        time.sleep(150)
        self.send_command(3) # send the trajectory enable command command using go commander
        # wait for all drones to send maintaining cruise speed
        # I need a subuscriber here for that
        while (len(self.ids) != PARAMS["NUM_DRONES"]):
            rclpy.spin_once(self, timeout_sec=1)
        # received all the confirmations now we wait for go command from user

    def send_phases(self, phases):
        phase_offset_publisher_node = Phase_Offset_Publisher(phases)
        # rclpy.spin(phase_offset_publisher_node)
        rclpy.spin_once(phase_offset_publisher_node, timeout_sec=1)
        phase_offset_publisher_node.destroy_node()

    def send_command(self, command):
        go_command_node = Go_Commander(command) # Send the command using go commander
        # print(f"Sending command: {go_command_node.command} from {go_command_node.source}")
        rclpy.spin_once(go_command_node, timeout_sec=1)
        go_command_node.destroy_node()

    def start_trials(self, method, random_seed=1):
        drone_names = get_drone_names(PARAMS["NUM_DRONES"])
        print("Got all drone names:")
        print(drone_names)

        # get random phases with zeros every 3rd trial on the list
        np.random.seed(random_seed)
        yaw_list = np.random.uniform(low=0.0, high=1.0, size=(NUM_TRIALS,PARAMS["NUM_DRONES"])) * 360.0
        for j in range(NUM_TRIALS):
            if j%2 == 1:
                yaw_list[j] = np.zeros(shape=(1,PARAMS["NUM_DRONES"]))

        print(f"Starting test with method: {method}")
        PARAMS['chosen_method'] = method
        self.send_command(2) # send them to waypoint 2 to start metrics properly
        try:
            metrics_node = LS_Metrics(PARAMS, drone_names, yaw_list)
            # rclpy.spin(metrics_node)
        finally:
            print("Closing")
            metrics_node.destroy_node()
            # rclpy.shutdown()


def main():
    rclpy.init()
    base_station = BaseStation()
    try:
        base_station.run()
    finally:
        if PARAMS['SIMULATION'] == False:
            rclpy.shutdown()
        print("ROS shutdown completed.")

if __name__ == "__main__":
    main()
