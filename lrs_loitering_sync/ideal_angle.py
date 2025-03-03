"""
A file that contains the functions necessary to get the ideal middle angle in a list of angles.
The function get_speed_ideal() takes in the list of angles and returns the desired speed for convenience. 
The function get_desired_angle_middlepoint() takes in the list of angles, finds the shortest arc to contain all the angles, 
    and returns the middle point of that arc as the desired angle.
There is also the option to get the desired behavior of the drones. Simply use: get_drone_behavior(angle_list)
This returns the distances to the desired angle between them and the direction at which each drone needs to move in the speedup_list element.

plot_angles() is available to plot the angles with special colors for the desired angle and indicated drone. Can be used for debugging/visualization purposes.
Needs matplotlib import to be uncommented. Needs matplotlib to be installed in the docker. 
"""

from math import pi, sin, cos
# import matplotlib.pyplot as plt


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


# Function that takes in a list of angles and returns a list of the distances between
# the angle at the index and each other angle
def angle_distance_list(angle_list, index):
    my_angle = angle_list[index]
    distances = []
    drone_speedup_list = []
    for i, angle in enumerate(angle_list):
        dist, drone_speedup = min_angle_distance(angle, my_angle)
        distances.append(dist)
        drone_speedup_list.append(drone_speedup)
    return distances, drone_speedup_list
    # plot_angles(angle_list, index)


# Function that takes in a list of angles, gets the distances between them,
# Finds the middle angle and decides on the desired angle to be at the middle between the angles around the middle angle
# It returns the middle point of the arc. or what we would call the desired angle
def get_desired_angle_middlepoint(angle_list):
    desired_angle = 0
    distances_list = []
    directions_list = []
    avg_distances = []
    for i in range(len(angle_list)):
        distances, directions = angle_distance_list(angle_list, i)
        distances_list.append(distances)
        directions_list.append(directions)
        if len(distances) == 1:
            avg_distances = distances
        else:
            avg_distances.append(sum(distances)/(len(distances)-1))

    middle_angle_index = argmin(avg_distances)
    middle_angle_distances = distances_list[middle_angle_index]
    middle_angle_directions = directions_list[middle_angle_index]

    max_distance_angle_index = argmax(middle_angle_distances)

    target_distance = avg_distances[middle_angle_index]
    max_direction = middle_angle_directions[max_distance_angle_index]
    
    angle1 = angle_list[max_distance_angle_index]
    if max_direction:
        desired_angle = angle1 + target_distance
    else:
        desired_angle = angle1 - target_distance

    desired_angle %= 2*pi
    return desired_angle

# function that gets the desired angle from an angle list and returns a speed up or slow down list for each drone based
# on their distance from that desired angle
def get_drone_behavior(angle_list):
    desired_angle = get_desired_angle_middlepoint(angle_list)
    angle_list.append(desired_angle)
    # get distances and behavior decision from last angle (desired angle)
    distances, speedup_list = angle_distance_list(angle_list, -1)
    print("Distances from desired angle: ", distances[:-1])
    print("Decisions to speed up: ", speedup_list[:-1])
    return distances[:-1], speedup_list[:-1]
    # plot_angles(angle_list, -1)


# Function that returns the speed to be used for the drone
def get_speed_ideal(yaw_list, SPEEDS, ACCEPTANCE_RANGE):
    my_yaw = yaw_list[-1]
    phase = get_desired_angle_middlepoint(yaw_list)
    distance, drone_speedup = min_angle_distance(my_yaw, phase)
    # divide acceptance range by 2 to allow for +/- range from the middle angle
    if distance < ACCEPTANCE_RANGE/2.0:
        speed = SPEEDS['cruise']
    else:
        if drone_speedup:
            speed = SPEEDS['high']
        else: 
            speed = SPEEDS['low']
    
    return speed



# plots a unit circle along with the points from an angle list
# highlights the point at index with green color on the circle
# highlights the point at index2 with red color on the circle
# def plot_angles(angle_list, index, index2 = None):
#     x = []
#     y = []
#     for angle in angle_list:
#         x.append(cos(angle))
#         y.append(sin(angle))
#     figure, axes = plt.subplots()
#     circ = plt.Circle((0, 0), radius=1., edgecolor='b', facecolor='None')
#     axes.set_aspect(1)
#     axes.add_artist(circ)
#     axes.scatter(x,y)
#     axes.scatter(x[index], y[index], color="green")
#     if index2:
#         axes.scatter(x[index2], y[index2], color="red")
#     plt.xlim( -1.2 , 1.2 )
#     plt.ylim( -1.2 , 1.2 )
#     plt.show()
