"""
A file that contains a simple visualization setup for synchronization algorithms.
It contains multiple synchronization methods.
Currently, it has no delays in state sharing or delays in speed changing.

Next steps:
1- Implement randomness to speed values to simulate real speeds of physical system
2- Implement slow change in speed as opposed to set speed with controllable acceleration decceleration rates.
3- Implement delays in state sharing. Not all drones would have the last updated yaw of everyone. Need to think how to do this.
"""

from math import pi, sin, cos, atan2
import matplotlib.pyplot as plt
import numpy as np
import time
import matplotlib.animation as animation


# returns argmax of list
def argmax(iterable):
    return max(enumerate(iterable), key=lambda x: x[1])[0]

# returns argmin of list
def argmin(iterable):
    return min(enumerate(iterable), key=lambda x: x[1])[0]

# gets the minimum arc distance between the two angles
# also returns a check of whether drone 1 needs to speed up or slow down
# Steps: we convert the angles to 0-2pi. we check angle1-angle2. 
#        If negative and abs is more than pi: drone1 speed up false. distance is 2pi - abs(diff). 
#        If negative and abs is less than pi: drone1 speed up true. distance is abs(diff)
#        If positive and less than pi. drone1 speed up false. distance is diff
#        If positive and more than pi. drone1 speed up true. distance is 2pi - diff
#       optimize the conditions by grouping more than pi vs less than pi condition first.
#        If abs is more than pi and negative: drone1 speed up false. distance is 2pi - abs(diff). 
#        If abs more than pi and positive. drone1 speed up true. distance is 2pi - diff
#        If abs is less than pi and negative: drone1 speed up true. distance is abs(diff)
#        If abs less than pi and positive. drone1 speed up false. distance is diff
def min_angle_distance(angle1, angle2):
    drone1_speedup = True
    # change angle range to 0-2pi
    angle1 %= 2*pi
    angle2 %= 2*pi

    diff = angle1 - angle2
    if abs(diff) < pi:
        distance = abs(diff)
        if diff > 0:
            drone1_speedup = False
    else:
        distance = 2*pi - abs(diff)
        if diff < 0:
            drone1_speedup = False
       
    return distance, drone1_speedup

# plots a unit circle along with the points from an angle list
# highlights the point at index with green color on the circle
# highlights the point at index2 with red color on the circle
def plot_angles(angle_list, index, index2 = None):
    x = []
    y = []
    for angle in angle_list:
        x.append(cos(angle))
        y.append(sin(angle))
    figure, axes = plt.subplots()
    circ = plt.Circle((0, 0), radius=1., edgecolor='b', facecolor='None')
    axes.set_aspect(1)
    axes.add_artist(circ)
    axes.scatter(x,y)
    axes.scatter(x[index], y[index], color="green")
    if index2:
        axes.scatter(x[index2], y[index2], color="red")
    plt.xlim( -1.2 , 1.2 )
    plt.ylim( -1.2 , 1.2 )
    plt.show()


# Function that finds the mean angle 
def get_ideal_mean_angle(yaws):
    yaws = np.array(yaws)
    x = sum(np.cos(yaws))/len(yaws)
    y = sum(np.sin(yaws))/len(yaws)
    return atan2(y,x)

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

# Function that implements change in speed based on pulses with drones reaching yaw 0
# 1- Go through all the angles and see if any are close to 0
# 2- If a group is at 0, change speed as if it's only one drone
# 3- depending on location, speed up or slow down by a step
def get_yaw0_pulses(t):
    global yaw0_timestamps
    global set_speeds
    YAW0_RANGE = 0.01
    SPEED_RESET_DURATION = 5
    for i in range(N):
        angles = known_angles[i,:]
        # change all angles to -pi to pi range
        angles = (angles + pi) % (2 * pi) - pi
        my_yaw = angles[i]
        for angle in angles:
            if abs(angle) < YAW0_RANGE:
                # change set speed based on my_yaw
                # Adjust my yaw to be within -π to π range
                yaw0_timestamps[i] = t
                if my_yaw > 0:
                    set_speeds[i] = SPEEDS['low']
                else:
                    set_speeds[i] = SPEEDS['high']
                break
        print("Checking yaw0 timestamp: ", t, yaw0_timestamps[i])
        if ((t - yaw0_timestamps[i]) >= SPEED_RESET_DURATION) and (yaw0_timestamps[i]>0):
            set_speeds[i] = SPEEDS['cruise']
            yaw0_timestamps[i] = -1


# Function that implements change in speed based on pulses with drones reaching yaw 0
# 1- Go through all the angles and see if any are close to 0
# 2- If a group is at 0, change speed as if it's only one drone
# 3- depending on location, speed up or slow down by a step
def get_ramp_yaw0_pulses(t):
    global yaw0_timestamps
    global zero_phase
    global set_speeds
    global pulse_magnitudes
    global pulse_timestamps
    YAW0_RANGE = 0.1
    SPEED_STEP = 2.0 #2.0
    SPEED_RESET_DURATION = 10
    for i in range(N):
        break_flag = False
        angles = known_angles[i,:]
        # change all angles to -pi to pi range
        angles = (angles + pi) % (2 * pi) - pi
        my_yaw = angles[i]
        for j, angle in enumerate(angles):
            if i == j:
                continue
            if abs(angle) < YAW0_RANGE:
                # change set speed based on my_yaw
                # Adjust my yaw to be within -π to π range
                if  abs(my_yaw) >= ACCEPTANCE_RANGE/2.0:
                    pulse_timestamps[i,j] = t
                    if my_yaw > 0:
                        pulse_magnitudes[i,j] = -SPEED_STEP     
                    else:
                        pulse_magnitudes[i,j] = SPEED_STEP
                    break_flag = True

            if ((t - pulse_timestamps[i,j]) >= SPEED_RESET_DURATION) and (pulse_timestamps[i,j]>0):
                pulse_magnitudes[i,j] = 0.0
                pulse_timestamps[i,j] = -1

            # if break_flag:
            #     break

        set_speeds[i] = SPEEDS['cruise'] + sum(pulse_magnitudes[i,:])
        if set_speeds[i] > SPEEDS['high']:
            set_speeds[i] = SPEEDS['high']
        elif set_speeds[i] < SPEEDS['low']:
            set_speeds[i] = SPEEDS['low']

NUM_DRONES = 10
yaw_list = np.zeros((1, NUM_DRONES))
# yaws1 = np.array([[0,45,90]])
# yaws2 = np.array([[-45,45,180]])
yaws1 = np.array([[357.31562838338346,309.58075036630436,43.52038553009031,119.77026672964648,259.7343867299766,256.0290370903007,337.11861124780546,151.95851998610948,298.8128495787577,241.31000390906556]])
yaws2 = np.array([[109.21266393585034,211.5290182116814,317.6924402994688,304.6310706341926,181.90217540865615,212.0408128737186,12.42929885448297,87.38639047550436,287.065529119549,149.15303974827873]])
yaw_list = np.concatenate((yaw_list,yaws1,yaws2), axis=0 )

r = 100
# SPEEDS = {'cruise':0.0, 'low':-2.0, 'high':2.0}
SPEEDS = {'cruise':17.0, 'low':15.0, 'high':19.0}
dt = 1.0
t0 = 0
tf = 1000 #180
RANDOM_MEAN = 0
RANDOM_STD = 0.1
global ann_list
global prev_t
global known_angles
known_angles = np.zeros((NUM_DRONES, NUM_DRONES))
global desired_angles
desired_angles = np.zeros(NUM_DRONES)
ACCELERATION_RATE = 0.5
DECCELERATION_RATE = -0.5
ENABLE_ACCELERATION = False
ENABLE_SPEED_RANDOMNESS = False
PROBABILITY_KNOWN = 1.0 # value to control percentage of known/updated angles for each drone
global speeds
global set_speeds
global LEADER_DRONE
global leader_drone_votes
global VIRTUAL_PHASE
LEADER_DRONE = -1
VIRTUAL_PHASE = 0.0

global pulse_magnitudes
global pulse_timestamps
pulse_magnitudes = np.zeros((NUM_DRONES, NUM_DRONES)) # used for yaw0 and pulsing methods
pulse_timestamps = -np.ones((NUM_DRONES, NUM_DRONES))
global yaw0_timestamps
yaw0_timestamps = -np.ones(NUM_DRONES)
global zero_phase
zero_phase = 0.0
ACCEPTANCE_RANGE = 0.2


DEBUG = True

# Function that changes the speed of each drone to match the set speed of the drone.
# The change will be done with an acceleration/deceleration rate that we set as a parameter
# There will be added noise to match the effect of the physical environment and imperfect controller
def update_speeds():
    global set_speeds
    global speeds
    for i in range(N):
        ds = set_speeds[i] - speeds[i]

        if ENABLE_ACCELERATION:
            if ds > ACCELERATION_RATE:
                ds = ACCELERATION_RATE
            elif ds < DECCELERATION_RATE:
                ds = DECCELERATION_RATE
        
        speeds[i] += ds
    if ENABLE_SPEED_RANDOMNESS:
        speeds += np.random.normal(loc=RANDOM_MEAN, scale=RANDOM_STD, size=set_speeds.shape)


# this function updates the known angles by each drone based on some decided policy
# we will start with all drones have updated angles and move forward by restricting how many get updated per cycle
# To change the percentage of updated angles, control PROBABILITY_KNOWN. This value should be between 0-1.0
def update_known_angles():
    global known_angles
    # get random choices of what is shared and what isnt
    random_options = np.array([True, False])
    random_probabilities = np.array([PROBABILITY_KNOWN, 1.0-PROBABILITY_KNOWN]) # probability that a random choice is made
    known_matrix = np.random.choice(random_options, size=known_angles.shape, replace=True, p=random_probabilities)
    np.fill_diagonal(known_matrix, True) # A drone always knows its own angle
    expanded_angles = np.tile(angles, (known_angles.shape[0],1))
    known_angles[known_matrix] = expanded_angles[known_matrix]


def update(t, speed):
        global ann_list
        global zero_phase
        # set point coordinates
        time.sleep(0.2)
        update_known_angles()
        get_ramp_yaw0_pulses(t)
        # get_yaw0_pulses(t)
        update_speeds()
        print(t,speeds)
        
        for i in range(len(ann_list)):
            ann_list[i].remove()
        ann_list = []
        zero_phase += SPEEDS['cruise']*dt/r
        for i in range(N):
            d_angle = speed[i]*dt/r
            angles[i] = angles[i] + d_angle
            if angles[i] > (2*np.pi):
                angles[i] -= 2*np.pi
            x,y = circle(angles[i] - zero_phase)
            ann = ax.annotate(str(i), xy=(x,y))
            ann_list.append(ann)
            points[i].set_data([x],[y])
        
        # goal points:
        for j in range(N):
            x,y = circle(zero_phase)
            ann = ax.annotate(str(j), xy=(x,y))
            ann_list.append(ann)
            points[j+N].set_data([x],[y])

        return points


def circle(phi):
    return np.array([r*np.cos(phi), r*np.sin(phi)])


if __name__ == "__main__":
    time_list = np.linspace(t0,tf,int(tf/dt), endpoint=False)
    for i in range(2,yaw_list.shape[0]):
        plt.rcParams["figure.figsize"] = 8,6
        plt.ion()
        fig, ax = plt.subplots()

        ax.axis([-1.5*r,1.5*r,-1.5*r,1.5*r])
        ax.set_aspect("equal")
        circ = plt.Circle((0, 0), radius=r, edgecolor='b', facecolor='None')
        ax.set_aspect(1)
        ax.add_artist(circ)

        angles = yaw_list[i]*np.pi/180.0
        # create initial conditions
        set_speeds = np.ones(angles.shape)*SPEEDS['cruise']
        speeds = set_speeds + np.random.normal(loc=RANDOM_MEAN, scale=RANDOM_STD, size=set_speeds.shape)
        leader_drone_votes = -np.ones(angles.shape, dtype=int)
        
        # amount of points
        N = len(angles)  

        # create a point in the axes
        points = []
        ann_list = []
        for j in range(N):
            x,y = circle(angles[j])
            ann = ax.annotate(str(j), xy=(x,y))
            ann_list.append(ann)
            points.append(ax.plot(x, y, marker="o", color='b')[0])
        
        for j in range(N):
            x,y = circle(desired_angles[j])
            ann = ax.annotate(str(j), xy=(x,y))
            ann_list.append(ann)
            points.append(ax.plot(x, y, marker="o", color='r')[0])

        if DEBUG:
            t = t0
            while t < tf:
                t += dt
                points = update(t, speeds)
                fig.canvas.draw()
                fig.canvas.flush_events()
            plt.close()
        else:
            ani = animation.FuncAnimation(fig,update,
                fargs=(speeds,), 
                interval = 2, 
                frames=time_list,
                blit=True,
                repeat=False)
            # plt.show(block=True)
            writergif = animation.PillowWriter(fps=30)
            ani.save('animations/' + method + '_animation_' + str(i) + '.gif', writer=writergif)
            plt.close()
