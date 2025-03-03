"""
This file handles the plotting of the data gathered by loitering sync
Data format:
timestamp, trial_number, airspeed x NUM_DRONES, set_speed x NUM_DRONES, phase angles x NUM_DRONES, yaws x NUM_DRONES, desired angle, order_parameter

"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

NUM_DRONES = 10

# function that reads csv file data as floats and returns a pandas dataframe containing them
def read_csv_file(file_path):
    try:
        data = pd.read_csv(file_path).astype(float)
        return data # pd.DataFrame(data).astype(float)
    except Exception as e:
        print(f"An error occurred while reading the file: {e}")
        return None

def create_order_parameter_plots(data):
    if data is not None:

        data_timestamp_order_param = data.iloc[:,[0,1,-1]]
        data_timestamp_order_param.columns = ['timestamp', 'trial_num','order_parameters']

        first_trial = int(data_timestamp_order_param.iloc[0,1])
        last_trial = int(data_timestamp_order_param.iloc[-1,1]) + 1

        for i in range(first_trial,last_trial):
            data_temp = data_timestamp_order_param[data_timestamp_order_param['trial_num'] == i].iloc[:,[0,2]]
            data_temp['timestamp'] = data_temp['timestamp'] - data_temp['timestamp'].iloc[0]
            plt.plot(data_temp['timestamp'], data_temp['order_parameters'])
            plt.show()

def create_desired_angle_plots(data, drone_id=0):
    if data is not None:

        data_timestamp_order_param = data.iloc[:,[0,1,12+drone_id,32+drone_id,42]]
        data_timestamp_order_param.columns = ['timestamp', 'trial_num','set_speed', 'yaw', 'desired_angles']

        first_trial = int(data_timestamp_order_param.iloc[0,1])
        last_trial = int(data_timestamp_order_param.iloc[-1,1]) + 1

        for i in range(first_trial,last_trial):
            data_temp = data_timestamp_order_param[data_timestamp_order_param['trial_num'] == i].iloc[:,[0,2,3,4]]
            data_temp['timestamp'] = data_temp['timestamp'] - data_temp['timestamp'].iloc[0]
            data_temp['set_speed'] = data_temp['set_speed'] - 17.0
            plt.plot(data_temp['timestamp'], data_temp['desired_angles'], 'r')
            plt.plot(data_temp['timestamp'], data_temp['set_speed'], 'g')
            plt.plot(data_temp['timestamp'], data_temp['yaw'], 'b')
            plt.show()

def analyze_desired_angle_plots(data, NUM_DRONES):
    if data is not None:
        if NUM_DRONES == 10:
            data_timestamp_order_param = data.iloc[:,[0,1,32,33,34,35,36,37,38,39,40,41,42]]
            data_timestamp_order_param.columns = ['timestamp', 'trial_num', 'yaw_2', 'yaw_3', 'yaw_4', 'yaw_5', 'yaw_6', 'yaw_7','yaw_8','yaw_9', 'yaw_10', 'yaw_11','desired_angles']

            first_trial = int(data_timestamp_order_param.iloc[0,1])
            last_trial = int(data_timestamp_order_param.iloc[-1,1]) + 1

            for i in range(first_trial,last_trial):
                data_temp = data_timestamp_order_param[data_timestamp_order_param['trial_num'] == i].iloc[:,[0,2,3,4,5,6,7,8,9,10,11,12]]
                data_temp['timestamp'] = data_temp['timestamp'] - data_temp['timestamp'].iloc[0]
                plt.plot(data_temp['timestamp'], data_temp['desired_angles'], 'r')
                plt.plot(data_temp['timestamp'], data_temp['yaw_2'], 'b')
                plt.plot(data_temp['timestamp'], data_temp['yaw_3'], 'b')
                plt.plot(data_temp['timestamp'], data_temp['yaw_4'], 'b')
                plt.plot(data_temp['timestamp'], data_temp['yaw_5'], 'b')
                plt.plot(data_temp['timestamp'], data_temp['yaw_6'], 'b')
                plt.plot(data_temp['timestamp'], data_temp['yaw_7'], 'b')
                plt.plot(data_temp['timestamp'], data_temp['yaw_8'], 'b')
                plt.plot(data_temp['timestamp'], data_temp['yaw_9'], 'b')
                plt.plot(data_temp['timestamp'], data_temp['yaw_10'], 'b')
                plt.plot(data_temp['timestamp'], data_temp['yaw_11'], 'b')
                plt.show()

        elif NUM_DRONES == 5:
            data_timestamp_order_param = data.iloc[:,[0,1,17,18,19,20,21,22]]
            data_timestamp_order_param.columns = ['timestamp', 'trial_num', 'yaw_2', 'yaw_3', 'yaw_4', 'yaw_5', 'yaw_6','desired_angles']

            first_trial = int(data_timestamp_order_param.iloc[0,1])
            last_trial = int(data_timestamp_order_param.iloc[-1,1]) + 1

            for i in range(first_trial,last_trial):
                data_temp = data_timestamp_order_param[data_timestamp_order_param['trial_num'] == i].iloc[:,[0,2,3,4,5,6,7]]
                data_temp['timestamp'] = data_temp['timestamp'] - data_temp['timestamp'].iloc[0]
                plt.plot(data_temp['timestamp'], data_temp['desired_angles'], 'r')
                plt.plot(data_temp['timestamp'], data_temp['yaw_2'], 'b')
                plt.plot(data_temp['timestamp'], data_temp['yaw_3'], 'b')
                plt.plot(data_temp['timestamp'], data_temp['yaw_4'], 'b')
                plt.plot(data_temp['timestamp'], data_temp['yaw_5'], 'b')
                plt.plot(data_temp['timestamp'], data_temp['yaw_6'], 'b')

                plt.show()


if __name__ == "__main__":
    # Example usage
    input_csv_file_path = "ramp_yaw_n_kuramoto_data.csv" # "ideal_data.csv"
    data = read_csv_file(input_csv_file_path)

    create_order_parameter_plots(data)
    # create_desired_angle_plots(data, 9)
    analyze_desired_angle_plots(data, NUM_DRONES)
