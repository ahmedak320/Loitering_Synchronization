#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
This file is the main file calling the synchronized loitering functions
Here, you define the chosen synchronization method, and it
calls the necessary class and starts the ros node.
It reads its parameters from 'lrs_loitering_sync_params.yml'

To add a new synchronization method:
    import the class
    Add necessary parameters into loitering_sync_params.yml
    Add it to the METHODS dictionary and choose it using chosen_method in the yaml file
"""
import rclpy
import yaml
from time import sleep

# Import different loitering sync methods
from .ideal_synchronization import IdealLoitering
from .yaw0_synchronization import YAW0Loitering
from .yaw_n_synchronization import YAW_N_Loitering
from .ideal_synchronization_2 import IdealLoitering2
from .ideal_synchronization_3 import IdealLoitering3
from .mean_angle_synchronization import MeanAngleLoitering
from .ramp_yaw0_synchronization import RAMPYAW0Loitering
from .ramp_yaw_n_kuramoto_synchronization import YAW_N_Loitering_RAMP_KURAMOTO
from .virtual_yaw import LoiteringSync


# defined methods of synchronization and their corresponding classes
METHODS = {
    'virtual': LoiteringSync,
    'ideal': IdealLoitering,
    'yaw0': YAW0Loitering,
    'yaw_n': YAW_N_Loitering,
    'ideal2': IdealLoitering2,
    'ideal3': IdealLoitering3,
    'mean': MeanAngleLoitering,
    'ramp_yaw0': RAMPYAW0Loitering,
    'ramp_yaw_n_kuramoto': YAW_N_Loitering_RAMP_KURAMOTO,
}

# mission_plans_filepath = 'lrs_workspace/src/utilities/qgroundcontrol-plan-transformer/qgroundcontrol_plan_transformer'
parameter_filepath = 'lrs_workspace/src/lrs_modules/lrs_loitering_sync/lrs_loitering_sync/'

# Load the YAML file
with open(parameter_filepath + 'loitering_sync_params.yml') as fh:
    # Convert the YAML data into a dictionary
    PARAMS = yaml.safe_load(fh)
    chosen_method = PARAMS['chosen_method']
    if PARAMS['SIMULATION']:
        PARAMS['SPEEDS'] = PARAMS['SIMULATION_SPEEDS']
    else:
        PARAMS['SPEEDS'] = PARAMS['REAL_SPEEDS']

mission_plans_filepath = PARAMS['MISSION_PATH']
if PARAMS['LOAD_PLAN']:
    from qgroundcontrol_plan_transformer.qgc_plan_parser import *
    from qgroundcontrol_plan_transformer.mission_upload import *
# Function that gets the current drone id
def get_drone_id():
    try:
        rclpy.init()
        temp_node = rclpy.node.Node(node_name="temp_node")
        ident = temp_node.get_namespace()
        if ident[-2].isdigit():
            ident = ident[-2:] # Set the id to be the drone number
        else:
            ident = ident[-1]
    finally:
        temp_node.destroy_node()
        rclpy.shutdown()

    return ident


def main():

    try:
        rclpy.init()  # initialize ros
        # define loitering sync node based on the chosen method. It starts the ros node inside.
        PARAMS['chosen_method'] = 'ideal3'
        loitering_sync_node_1 = METHODS[PARAMS['chosen_method']]("loitering_sync_1", PARAMS)
    finally:
        # Ensure rclpy shutdown is called to clean up resources.
        if rclpy.ok():
            loitering_sync_node_1.destroy_node()
            # rclpy.shutdown()
    sleep(1)

    try:
        # rclpy.init()  # initialize ros
        # define loitering sync node based on the chosen method. It starts the ros node inside.
        PARAMS['chosen_method'] = 'ramp_yaw_n_kuramoto'
        loitering_sync_node_2 = METHODS[PARAMS['chosen_method']]("loitering_sync_2", PARAMS)
    finally:
        # Ensure rclpy shutdown is called to clean up resources.
        if rclpy.ok():
            loitering_sync_node_2.destroy_node()
            # rclpy.shutdown()
    sleep(1)

    try:
        # rclpy.init()  # initialize ros
        # define loitering sync node based on the chosen method. It starts the ros node inside.
        PARAMS['chosen_method'] = 'mean'
        loitering_sync_node_3 = METHODS[PARAMS['chosen_method']]("loitering_sync_3", PARAMS)
    finally:
        # Ensure rclpy shutdown is called to clean up resources.
        if rclpy.ok():
            loitering_sync_node_3.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
