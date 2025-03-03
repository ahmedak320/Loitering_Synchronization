import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.animation as animation
from matplotlib.patches import Circle
import math

def unwrap_angles(yaw_array):
    return np.unwrap(yaw_array)

def yaw_to_position(yaw_rad, radius=1.0):
    x = radius * math.sin(yaw_rad)
    y = radius * math.cos(yaw_rad)
    return x, y

def get_order_parameter(yaws):
    yaws = np.unwrap(yaws)
    N = len(yaws)
    x = np.sum(np.cos(yaws)) / N
    y = np.sum(np.sin(yaws)) / N
    return np.sqrt(x**2 + y**2)

def plot_order_parameters(names, save_plot=False, tf=179, plot_title='', labels=[], save_path="", animate=True):
    colors = ['orange', 'red']
    
    for j, name in enumerate(names):
        path = name + "_data.csv"
        data = pd.read_csv(path).astype(float)
        
        data_timestamp_order_param = data.iloc[:, [0, 1, 2, 3, 4, 5, 6, 7, -5, -4, -3, -2, -1]]
        data_timestamp_order_param.columns = ['timestamp', 'trial_num', 'speed_1', 'speed_2', 'speed_3', 
                                              'set_speed_1', 'set_speed_2', 'set_speed_3', 
                                              'yaw_1', 'yaw_2', 'yaw_3', 'desired_yaw', 'order_parameters']
        
        first_trial = int(data_timestamp_order_param.iloc[0, 1])
        last_trial = int(data_timestamp_order_param.iloc[-1, 1]) + 1
        
        trial_new_time = []
        trial_yaw_1 = []
        trial_yaw_2 = []
        trial_yaw_3 = []
        trial_desired_yaw = []
        computed_order_params = []
        
        for i in range(first_trial, last_trial):
            data_temp = data_timestamp_order_param[data_timestamp_order_param['trial_num'] == i].copy()
            data_temp['timestamp'] -= data_temp['timestamp'].iloc[0]
            
            time_temp = data_temp['timestamp'].values
            yaw_1_temp = unwrap_angles(data_temp['yaw_1'].values)
            yaw_2_temp = unwrap_angles(data_temp['yaw_2'].values)
            yaw_3_temp = unwrap_angles(data_temp['yaw_3'].values)
            desired_yaw_temp = unwrap_angles(data_temp['desired_yaw'].values)
            
            new_time_temp = np.arange(0, tf, 0.01)
            yaw_1_interp = interp1d(time_temp, yaw_1_temp, kind='linear')(new_time_temp)
            yaw_2_interp = interp1d(time_temp, yaw_2_temp, kind='linear')(new_time_temp)
            yaw_3_interp = interp1d(time_temp, yaw_3_temp, kind='linear')(new_time_temp)
            desired_yaw_interp = interp1d(time_temp, desired_yaw_temp, kind='linear')(new_time_temp)
            
            computed_order_param_temp = [get_order_parameter([yaw_1_interp[k], yaw_2_interp[k], yaw_3_interp[k]]) for k in range(len(new_time_temp))]
            
            trial_new_time.append(new_time_temp)
            trial_yaw_1.append(yaw_1_interp)
            trial_yaw_2.append(yaw_2_interp)
            trial_yaw_3.append(yaw_3_interp)
            trial_desired_yaw.append(desired_yaw_interp)
            computed_order_params.append(computed_order_param_temp)
            
            break  # Only process the first trial
        
        plt.figure(figsize=(10, 6))
        plt.plot(trial_new_time[0], computed_order_params[0], label='Computed Order Parameter', color='blue')
        plt.plot(trial_new_time[0], data_timestamp_order_param['order_parameters'].values[:len(trial_new_time[0])], label='Data Order Parameter', color='red', linestyle='dashed')
        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Order Parameter")
        plt.title("Comparison of Computed vs. Data Order Parameter")
        plt.grid()
        plt.show()
        
        if animate:
            fig_anim, ax_anim = plt.subplots(figsize=(6, 6))
            ax_anim.set_xlim(-1.5, 1.5)
            ax_anim.set_ylim(-1.5, 1.5)
            ax_anim.set_aspect('equal')
            ax_anim.grid(True)
            ax_anim.set_title(f'Drone Motion - {labels[j] if labels else name}')
            
            formation_circle = plt.Circle((0, 0), 1.0, fill=False, color='gray', linestyle='--')
            ax_anim.add_artist(formation_circle)
            
            drone_colors = ['blue', 'green', 'red']
            drone_markers = [plt.Circle((0, 0), 0.1, fill=True, color=drone_colors[i]) for i in range(3)]
            for drone in drone_markers:
                ax_anim.add_patch(drone)
            
            desired_marker = plt.Circle((0, 0), 0.1, fill=True, color='black', alpha=0.5)
            ax_anim.add_patch(desired_marker)
            
            time_text = ax_anim.text(0.05, 0.95, '', transform=ax_anim.transAxes)
            
            def update_animation(frame):
                pos1 = yaw_to_position(trial_yaw_1[0][frame])
                pos2 = yaw_to_position(trial_yaw_2[0][frame])
                pos3 = yaw_to_position(trial_yaw_3[0][frame])
                pos_desired = yaw_to_position(trial_desired_yaw[0][frame])
                
                drone_markers[0].center = pos1
                drone_markers[1].center = pos2
                drone_markers[2].center = pos3
                desired_marker.center = pos_desired
                
                current_time = trial_new_time[0][frame]
                time_text.set_text(f'Time: {current_time:.2f}s')
                
                return drone_markers + [desired_marker, time_text]
            
            frames = len(trial_new_time[0])
            ani = animation.FuncAnimation(fig_anim, update_animation, frames=frames, interval=20, blit=True)
            plt.show()