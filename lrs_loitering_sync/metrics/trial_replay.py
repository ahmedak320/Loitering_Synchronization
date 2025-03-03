"""
This file handles the plotting of the data gathered by loitering sync
Data format:
timestamp, set_speed x NUM_DRONES, airspeed x NUM_DRONES, , phase angles x NUM_DRONES, yaws x NUM_DRONES, desired_angle, order_parameter

It takes in the file name and trial number. It takes out the yaws and speeds, and set speeds of that trial and plays them in animation
It also creates plots for yaws and speeds and order parameter

"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objects as go
from scipy.interpolate import interp1d
from scipy import stats
from math import pi, sin, cos, atan2
import matplotlib.pyplot as plt
import numpy as np
import time
import matplotlib.animation as animation

NUM_DRONES = 10
r = 100

global points
global ann_list
global ax
global arrows
global zero_phase
zero_phase = 0.0
points = []
ann_list = []
arrows = []

# function that reads csv file data as floats and returns a pandas dataframe containing them
def read_csv_file(file_path):
    try:
        data = pd.read_csv(file_path).astype(float)
        return data
    except Exception as e:
        print(f"An error occurred while reading the file: {e}")
        return None


def update(yaws, speeds, dt):
        global points
        global ann_list
        global ax
        global arrows
        global zero_phase


        # time.sleep(0.1)

        for i in range(len(ann_list)):
            ann_list[i].remove()
            if i < len(arrows):
                arrows[i].remove()
        ann_list = []
        arrows = []
        CRUISE_SPEED = 17.0
        zero_phase += CRUISE_SPEED*dt/r
        for i in range(len(yaws)-1):
            x,y = circle(yaws[i] - zero_phase)
            ann = ax.annotate(str(i), xy=(x,y))
            ann_list.append(ann)
            points[i].set_data([x],[y])
            dx, dy = get_arrow(yaws[i], speeds[i])
            arrows.append(ax.arrow(x,y,dx,dy, head_width=8, length_includes_head=True))

        # goal points:
        x,y = circle(yaws[-1] - zero_phase)#
        ann = ax.annotate('X', xy=(x,y))
        ann_list.append(ann)
        points[-1].set_data([x],[y])

        return points


def circle(phi):
    return np.array([r*np.cos(phi), r*np.sin(phi)])

# function that gets the arrow dx and dy for plotting
# should be the tangent to the circle at the yaw angle
# should be scaling with the speed received
def get_arrow(yaw, speed):
    arrow_angle = yaw - zero_phase + np.pi/2
    DEFAULT_MAGNITUDE = 20
    CRUISE_SPEED = 17.0
    arrow_magnitude = DEFAULT_MAGNITUDE * (speed - CRUISE_SPEED)/2.0
    dx = arrow_magnitude * cos(arrow_angle)
    dy = arrow_magnitude * sin(arrow_angle)
    return dx, dy

# function to visualize a trial from a csv data file with 10 drones
def replay_data(filename, DESIRED_TRIAL=1, dt=0.1, delay=0):
    global points
    global ann_list
    global ax
    global arrows

    # read csv file
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

    # get the desired trial data
    trial_data = data[data['trial_num'] == DESIRED_TRIAL]
    # subtract first timestamp value
    trial_data.iloc[:,0] = trial_data.iloc[:,0] - trial_data.iloc[0,0]

    # get the yaws into a list to replay the trial in matplotlib animation
    yaw_data = trial_data[['yaw0',
                          'yaw1',
                          'yaw2',
                          'yaw3',
                          'yaw4',
                          'yaw5',
                          'yaw6',
                          'yaw7',
                          'yaw8',
                          'yaw9',
                          'desired_angle']].values.tolist()
    
    speed_data = trial_data[['speed0',
                            'speed1',
                            'speed2',
                            'speed3',
                            'speed4',
                            'speed5',
                            'speed6',
                            'speed7',
                            'speed8',
                            'speed9',]].values.tolist()

    time_data = trial_data['timestamp'].values.tolist()
    order_parameter_data = trial_data['order_parameters'].values.tolist()


    # start the animation replay of the trial
    plt.rcParams["figure.figsize"] = 8,6
    plt.ion()
    fig, ax = plt.subplots()

    ax.axis([-1.5*r,1.5*r,-1.5*r,1.5*r])
    ax.set_aspect("equal")
    circ = plt.Circle((0, 0), radius=r, edgecolor='b', facecolor='None')
    ax.set_aspect(1)
    ax.add_artist(circ)

    # create a point in the axes
    points = []
    ann_list = []
    arrows = []
    for j in range(len(yaw_data[0])-1):
        x,y = circle(yaw_data[0][j])
        ann = ax.annotate(str(j), xy=(x,y))
        ann_list.append(ann)
        points.append(ax.plot(x, y, marker="o", color='b')[0])
        dx, dy = get_arrow(yaw_data[0][j], speed_data[0][j])
        arrows.append(ax.arrow(x,y,dx,dy, head_width=8, length_includes_head=True))

    x,y = circle(yaw_data[0][-1])
    ann = ax.annotate("X", xy=(x,y))
    ann_list.append(ann)
    points.append(ax.plot(x, y, marker="o", color='r')[0])

    # data skip factor for if the trial had a lot of points
    if len(yaw_data) > 3000:
        DATA_SKIP = 10
    else:
        DATA_SKIP = 1

    for i, yaws in enumerate(yaw_data):
        if i%DATA_SKIP == 0:
            if i%10 == 0:
                print("time is: ", time_data[i], " order parameter: ", order_parameter_data[i])
            speeds = speed_data[i]
            points = update(yaws, speeds, dt)
            fig.canvas.draw()
            fig.canvas.flush_events()
            if delay != 0:
                time.sleep(delay)
    plt.close()


if __name__ == "__main__":
    # names of data files in the format name_data.csv
    filename = "simple_mean_data.csv"
    trial_num = 1
    replay_data(filename, trial_num)
