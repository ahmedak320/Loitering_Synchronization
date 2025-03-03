"""
This file handles the plotting of the data gathered by loitering sync
Data format:
timestamp, airspeed x NUM_DRONES, set_speed x NUM_DRONES, phase angles x NUM_DRONES, yaws x NUM_DRONES, order_parameter

"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objects as go
from scipy.interpolate import interp1d
from scipy import stats
from trial_replay import *

# function that reads csv file data as floats and returns a pandas dataframe containing them
def read_csv_file(file_path):
    try:
        data = pd.read_csv(file_path).astype(float)
        return data
    except Exception as e:
        print(f"An error occurred while reading the file: {e}")
        return None

# function that opens name_sync_durations.csv file and gets the mean of the values there
def get_mean_duration(name):
    path = name + "_sync_durations.csv"
    data = read_csv_file(path)
    print("Average duration for " + name + " method is: " + str(data.values.mean()))

    return data.values.mean()

# Function that finds the mean angle
def get_mean_angle(yaws_list):
    yaws = np.array(yaws_list)
    x = np.sum(np.cos(yaws), axis=1)/len(yaws[0])
    y = np.sum(np.sin(yaws), axis=1)/len(yaws[0])
    return np.arctan2(y,x)

# function that goes into the csv files in names and plots their order parameters vs time plots
# it includes median lines and quartile lines
def plot_order_parameters(names, save_plot=False, tf = 179, plot_title='', labels=[], save_path=""):
    for j, name in enumerate(names):
        # get_mean_duration(name)

        # read csv file
        path = name + "_data.csv"
        data = read_csv_file(path)

        # get the relevant columns
        data_timestamp_order_param = data.iloc[:,[0,1,-1]]
        data_timestamp_order_param.columns = ['timestamp', 'trial_num','order_parameters']

        # trial numbers based on the read data
        first_trial = int(data_timestamp_order_param.iloc[0,1])
        last_trial = int(data_timestamp_order_param.iloc[-1,1]) + 1

        # time = list(data_timestamp_order_param.iloc[:,[0]].values)
        # trial_num = list(data_timestamp_order_param.iloc[:,[1]].values)
        # order_params = list(data_timestamp_order_param.iloc[:,[2]].values)

        trial_time = []
        trial_order_params = []
        trial_new_time = []
        interpolated_order_params = []
        # Loop that divides the data into lists where the index is the trial number.
        for i in range(first_trial,last_trial):
            # get the current trial data
            data_temp = data_timestamp_order_param[data_timestamp_order_param['trial_num'] == i].iloc[:,[0,2]]
            # subtract the initial timestamp value
            data_temp['timestamp'] = data_temp['timestamp'] - data_temp['timestamp'].iloc[0]
            # create the lists of current trial time and order parameters
            time_temp = list(data_temp.iloc[:,0].values)
            order_params_temp = list(data_temp.iloc[:,1].values)
            # create a new time series and perform linear interpolation of the data to match it
            new_time_temp = np.arange(0, tf, 0.01).tolist()
            order_param_interpolation_func = interp1d(
                np.array(time_temp), np.array(order_params_temp), kind='linear')
            interpolated_order_params_temp = order_param_interpolation_func(new_time_temp)

            # save the final outcome into lists
            trial_time.append(time_temp)
            trial_order_params.append(order_params_temp)
            trial_new_time.append(new_time_temp)
            interpolated_order_params.append(interpolated_order_params_temp)

        # Get quartiles of the data
        data_array = np.array(interpolated_order_params)
        # quartiles = stats.mstats.mquantiles(data_array, prob=[0.0, 0.5, 1.0], axis=0)
        quartiles = stats.mstats.mquantiles(data_array, prob=[0.25, 0.5, 0.75], axis=0)
        # plot the 3 quartile lines
        # plt.plot(trial_new_time[0], quartiles[0], label=name+"_25Q")
        # plt.plot(trial_new_time[0], quartiles[1], label=name+"_50Q")
        # plt.plot(trial_new_time[0], quartiles[2], label=name+"_75Q")
        if len(labels) != 0:
            label = labels[j]
        else:
            label = name
        plt.plot(trial_new_time[0], quartiles[1], label=label)
        plt.fill_between(trial_new_time[0], quartiles[0], quartiles[2], alpha=0.2)
        plt.legend(loc = 'lower right')
        plt.grid(True)

    # plt.title('Comparison between Synchronization methods')
    if plot_title != '':
        plt.title(plot_title)
    plt.xlabel('time (s)')
    plt.ylabel('order parameter')

    if save_plot:
        if save_path == "":
            save_path = names[0] + '_vs_' + names[1] + '.png'
        plt.savefig(save_path)
    plt.show()

# function that gets the trial number of results that look too good to be true
# This is based on how many values or order parameters are over 0.8 before t = 25
def get_too_good_trial_num(data, time_limit=25, order_parameter_limit=0.8):
    # get all trials with order parameters more than order_parameter_limit
    temp_data = data[data['timestamp'] < time_limit] # get all the data with t<time_limit
    too_good_trials = temp_data[temp_data['order_parameters'] >= order_parameter_limit]
    # get a count of how many times a trial number is repeated in this list
    too_good_trials_count = too_good_trials.pivot_table(index = ['trial_num'], aggfunc ='size')
    # The trial number repeated the most is the worst trial
    max_index = too_good_trials_count.argmax()
    too_good_trial_number = too_good_trials_count.index[max_index]
    too_good_trials_list = too_good_trials_count.index
    return too_good_trial_number, too_good_trials_list

# function that gets the worst trial number
# This is based on how many values or order parameters are under 0.01
def get_bad_trial_num(data):
    # get all trials with order parameters less than 0.01
    bad_trials = data[data['order_parameters'] <= 0.01]
    # get a count of how many times a trial number is repeated in this list
    bad_trials_count = bad_trials.pivot_table(index = ['trial_num'], aggfunc ='size')
    # The trial number repeated the most is the worst trial
    max_index = bad_trials_count.argmax()
    bad_trial_number = bad_trials_count.index[max_index]
    bad_trials_list = bad_trials_count.index
    return bad_trial_number, bad_trials_list

# function that plots order parameter of a specific trial number
def plot_trial_order_parameter(data, trial_num):
    temp_data = data[['timestamp', 'trial_num','order_parameters']]
    trial_data = temp_data[temp_data['trial_num'] == trial_num]
    # subtract first timestamp value
    trial_data.iloc[:,0] = trial_data.iloc[:,0] - trial_data.iloc[0,0]
    plt.plot(trial_data['timestamp'], trial_data['order_parameters'])
    plt.title("trial number: " + str(trial_num) + " order parameter vs time")
    plt.show()

# function that gets the number of cycles the drones take until they reach an order parameter value
def count_cycles(data, order_parameter=0.95):
    # get the yaws into a list to replay the trial in matplotlib animation
    yaw_data = data[['trial_num',
                    'yaw0',
                    'yaw1',
                    'yaw2',
                    'yaw3',
                    'yaw4',
                    'yaw5',
                    'yaw6',
                    'yaw7',
                    'yaw8',
                    'yaw9',
                    'order_parameters']]

    first_trial = int(yaw_data.iloc[0,0])
    last_trial = int(yaw_data.iloc[-1,0]) + 1

    cycle_count = []
    for i in range(first_trial,last_trial):
        data_temp = yaw_data[yaw_data['trial_num'] == i].iloc[:,[1,2,3,4,5,6,7,8,9,10,11]]
        data_temp = data_temp[data_temp['order_parameters'] <= order_parameter].values.tolist()
        yaw_array = np.array(data_temp)
        mean_data = get_mean_angle(yaw_array)
        sin_mean = np.sin(mean_data)
        sign_mean = np.sign(sin_mean)
        signchange = ((np.roll(sign_mean, 1) - sign_mean) != 0).astype(int)
        cycle_count.append(sum(signchange))
    return np.mean(cycle_count)


if __name__ == "__main__":
    GENERAL_PLOT = True
    # names of data files in the format name_data.csv
    if GENERAL_PLOT:
        # names = ["simple_longer_yaw0_ramp_kuramoto", "simple_longer_yaw_n_ramp"]# "good_data/mean"] #"good_data/virtual"] # "good_data/middle_drone"
        # names = ["simple_yaw0_ramp", "simple_yaw0_ramp_jolt"]
        # names = ["simple_longer_yaw0_ramp_kuramoto", "simple_mean"]#"simple_longer_yaw0_ramp_calculated_jolt"]
        # names = ["simple_yaw_n_ramp_kuramoto", "simple_mean"]#"simple_longer_yaw0_ramp_calculated_jolt"]
        # names = ["simple_longer_yaw_n_ramp_kuramoto_25", "simple_longer_yaw_n_ramp_kuramoto_50", "simple_longer_yaw_n_ramp_kuramoto_75", "simple_longer_yaw_n_ramp_kuramoto_100", "simple_longer_yaw_n_ramp_kuramoto_hybrid"]
        # names = ['simple_ideal3_0_05', 'simple_ideal3_0_1', 'simple_ideal3_0_5', 'simple_ideal3_1', 'simple_ideal3_10']
        # names = ["simple_longer_yaw_n_ramp_kuramoto_100", "simple_longer_yaw_n_ramp_kuramoto_hybrid"]
        # names = ['simple_longer_ideal3_perfect_0_05', 'simple_longer_ideal3_perfect_0_1']#, 'simple_longer_ideal3_perfect_0_5', 'simple_longer_ideal3_perfect_1', 'simple_longer_ideal3_perfect_10']
        # names = ['simple_longer_ideal3_0_05', 'simple_longer_ideal3_0_1', 'simple_longer_ideal3_0_5']#, 'simple_longer_ideal3_1', 'simple_longer_ideal3_10']
        # names = ['simple_longer_ideal3_0.05_0.05_False', 'simple_longer_ideal3_0.05_0.05_True']#, 'simple_longer_ideal3_0_5']#, 'simple_longer_ideal3_1', 'simple_longer_ideal3_10']
        names = ['simple_longer_ideal3_0.1_0.1_False', 'simple_longer_ideal3_0.1_0.1_True']
        names = ['simple_longer_ideal3_0.5_0.5_False', 'simple_longer_ideal3_0.5_0.5_True']
        # names = ['simple_longer_ideal3_1.0_1.0_False', 'simple_longer_ideal3_1.0_1.0_True']
        names = ['simple_longer_ideal3_0.05_0.05_False', 'simple_longer_ideal3_0.05_0.05_True', 'simple_longer_ideal3_0.1_0.1_False', 'simple_longer_ideal3_0.1_0.1_True']#, 'simple_longer_ideal3_1.0_1.0_False', 'simple_longer_ideal3_1.0_1.0_True']
        # names = ["simple_mean", "simple_virtual_mean", "simple_virtual_pulse"]
        names = ["good_data/ideal3", "ramp_yaw_n_kuramoto"]
        names = ["simple_good_data/simple_mean", "simple_good_data/simple_ideal3", "simple_good_data/simple_yaw_n_ramp_kuramoto"]
        names = ["freq_sweep_data/simple_longer_ideal3_0.05_0.05_False", "freq_sweep_data/simple_longer_ideal3_0.1_0.1_False", "freq_sweep_data/simple_longer_ideal3_0.5_0.5_False", "freq_sweep_data/simple_longer_ideal3_1.0_1.0_False"]
        names = ["ideal3"]

        names_plot1 = ["paper_data/simple/plot_1_synchronization_methods/simple_mean", "paper_data/simple/plot_1_synchronization_methods/simple_ideal3","paper_data/simple/plot_1_synchronization_methods/simple_yaw_1_ramp_kuramoto","paper_data/simple/plot_1_synchronization_methods/simple_yaw_4_ramp_kuramoto"]
        names_plot2 = ["paper_data/simple/plot_2_worst_case_scenario_for_mean/simple_mean_worst_case_range_20_degrees", "paper_data/simple/plot_2_worst_case_scenario_for_mean/simple_ideal3_worst_case_range_20_degrees","paper_data/simple/plot_2_worst_case_scenario_for_mean/simple_yaw_1_ramp_kuramoto_worst_case_range_20_degrees","paper_data/simple/plot_2_worst_case_scenario_for_mean/simple_yaw_4_ramp_kuramoto_worst_case_range_20_degrees"]
        names_plot3 = ["paper_data/simple/plot_3_worst_case_scenario_of_ideal_method/simple_mean_Distributed_case_range_10_degrees", "paper_data/simple/plot_3_worst_case_scenario_of_ideal_method/simple_ideal3_Distributed_case_range_10_degrees","paper_data/simple/plot_3_worst_case_scenario_of_ideal_method/simple_yaw_1_ramp_kuramoto_Distributed_case_range_10_degrees","paper_data/simple/plot_3_worst_case_scenario_of_ideal_method/simple_yaw_4_ramp_kuramoto_Distributed_case_range_10_degrees"]
        names_plot4 = ["paper_data/gazebo/mean", "paper_data/gazebo/ideal3", "paper_data/gazebo/ramp_yaw_1_kuramoto", "paper_data/gazebo/ramp_yaw_4_kuramoto"]
        names_plot5 = ['paper_data/simple_longer_ideal3_0.05_0.05_False', 'paper_data/simple_longer_ideal3_0.1_0.1_False', 'paper_data/simple_longer_ideal3_0.5_0.5_False', 'paper_data/simple_longer_ideal3_1.0_1.0_False']
        names_plot6 = ['paper_data/simple_longer_ideal3_0.05_0.05_True', 'paper_data/simple_longer_ideal3_0.1_0.1_True', 'paper_data/simple_longer_ideal3_0.5_0.5_True', 'paper_data/simple_longer_ideal3_1.0_1.0_True']
        names_plot7 = ['paper_data/simple_longer_yaw_n_ramp_kuramoto_25', 'paper_data/simple_longer_yaw_n_ramp_kuramoto_50', 'paper_data/simple_longer_yaw_n_ramp_kuramoto_75', 'paper_data/simple_longer_yaw_n_ramp_kuramoto_100', 'paper_data/simple_longer_yaw_n_ramp_kuramoto_hybrid']

        # names = ['ramp_yaw_n_kuramoto']
        # (names, save_plot=False, tf = 179, plot_title='Comparison between Synchronization methods', labels=[], save_path="")
        # plot_order_parameters(names_plot1, save_plot=True, tf=179, plot_title='Comparison between Synchronization methods', labels=["baseline", "MOSA", "FPS (1)", "FPS (4)"], save_path="paper_data/simple/simple_synchronization_methods.png")
        plot_order_parameters(names_plot2, save_plot=True, tf=178, plot_title='Method Comparison in baseline Worst Case', labels=["baseline", "MOSA", "FPS (1)", "FPS (4)"], save_path="paper_data/simple/simple_synchronization_methods Worst case for mean.png")
        plot_order_parameters(names_plot3, save_plot=True, tf=178, plot_title='Method Comparison in MOSA Worst Case', labels=["baseline", "MOSA", "FPS (1)", "FPS (4)"], save_path="paper_data/simple/simple_synchronization_methods Distributed Ideal worst case.png")
        # plot_order_parameters(names_plot4, save_plot=True, tf=179, plot_title='Comparison between Synchronization methods in Gazebo', labels=["baseline", "MOSA", "FPS (1)", "FPS (4)"], save_path="paper_data/gazebo/gazebo_synchronization_methods.png")
        # plot_order_parameters(names_plot5, save_plot=True, tf=300, plot_title='Frequency Sweep without Offset', labels=["F = 0.05", "F = 0.1", "F = 0.5", "F = 1.0"], save_path="paper_data/frequency_sweep_no_offset.png")
        # plot_order_parameters(names_plot6, save_plot=True, tf=300, plot_title='Frequency Sweep with Offset', labels=["F = 0.05", "F = 0.1", "F = 0.5", "F = 1.0"], save_path="paper_data/frequency_sweep_with_offset.png")
        # plot_order_parameters(names_plot7, save_plot=True, tf=478, plot_title='Pulse Duration Effect on Single Pulse Performance', labels=["PD = 25%", "PD = 50%", "PD = 75%", "PD = 100%", "PD = 100%/25% hybrid"], save_path="paper_data/frequency_sweep_with_offset.png")
        # plot_order_parameters(names, save_plot=False, tf=178)#, plot_title='Pulse Duration Effect on Single Pulse Performance', labels=["PD = 25%", "PD = 50%", "PD = 75%", "PD = 100%", "PD = 100%/25% hybrid"], save_path="paper_data/frequency_sweep_with_offset.png")


    else:
        names = ["simple_mean", "simple_ideal3", "simple_ideal2", "simple_virtual", "simple_virtual_mean", "simple_yaw0",
                 "simple_yaw0_ramp", "simple_yaw0_ramp_saturated", "simple_kuramoto", "simple_mean_kuramoto",
                 "simple_yaw0_ramp_kuramoto", "simple_longer_yaw0", "simple_longer_yaw0_ramp", "simple_longer_yaw0_ramp_saturated",
                 "simple_longer_yaw0_ramp_kuramoto", "simple_longer_kuramoto"]
        # names = ['simple_good_data/simple_longer_yaw0_ramp_kuramoto']
        # names = ['simple_yaw0_ramp_jolt']
        names = ["simple_longer_ideal3_0.1_0.1_True"]
        for name in names:
        # name = "simple_mean"
        # read csv file
            filename = name + "_data.csv"
            data = read_csv_file(filename)
            data.columns = ['timestamp',
                        'trial_num',
                        'speed0',
                        'speed1',
                        'speed2',
                        'speed3',
                        'speed4',
                        'speed5',
                        'speed6',
                        'speed7',
                        'speed8',
                        'speed9',
                        'set_speed0',
                        'set_speed1',
                        'set_speed2',
                        'set_speed3',
                        'set_speed4',
                        'set_speed5',
                        'set_speed6',
                        'set_speed7',
                        'set_speed8',
                        'set_speed9',
                        'phase0',
                        'phase1',
                        'phase2',
                        'phase3',
                        'phase4',
                        'phase5',
                        'phase6',
                        'phase7',
                        'phase8',
                        'phase9',
                        'yaw0',
                        'yaw1',
                        'yaw2',
                        'yaw3',
                        'yaw4',
                        'yaw5',
                        'yaw6',
                        'yaw7',
                        'yaw8',
                        'yaw9',
                        'desired_angle',
                        'order_parameters']


            # bad_trial_num, bad_trial_list = get_bad_trial_num(data)
            # for bad_trial in bad_trial_list:
            #     plot_trial_order_parameter(data, int(bad_trial))
            replay_data(filename, DESIRED_TRIAL=1, dt=0.1, delay=0)


            # too_good_trial_num, too_good_trial_list = get_too_good_trial_num(data)
            # for too_good_trial in too_good_trial_list:
            #     plot_trial_order_parameter(data, int(too_good_trial))
            # replay_data(filename, DESIRED_TRIAL=0)
            # avg_cycles = count_cycles(data)
            # print("Method: " + name + " avg cycles: " + str(avg_cycles))


# At high speed of 19 m/s and radius of 100m, dt=0.1s. In dt, travelled 1.9m. = r*theta => theta = 1.9/100 = 0.019 rad
# YAW0Range is 0.01 (which is translated to a pulse of a drone).
# ACCEPTANCE_RANGE is 0.2 rads. So it takes 1.1s for a drone to leave the acceptance region. (If drone going at cruise speed then 1.2s)