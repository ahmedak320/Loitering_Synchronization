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
    plot_angles(angle_list, -1)
    return distances[:-1], speedup_list[:-1]


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
    
    return speed, phase



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

# Function that gets the proposed ideal angle for the drones to meet:
# 1- Get mean angle
# 2- Find drones furthest from it in both positive and negative sides (the biggest arc)
# 3- Get mean of those 2 angles as the desired meeting point. 
# 4- If result is too far from mean angle, we took the middle point of the wrong arc and should add pi to it
def get_ideal_angle_2(yaws):
    mean_angle = get_ideal_mean_angle(yaws)
    max_distance = -float('inf')
    max_yaw_index = 0
    min_distance = float('inf')
    min_yaw_index = 0
    for i, yaw in enumerate(yaws):
        distance = shortest_angle_difference(yaw, mean_angle)[0]
        if distance >= max_distance:
            max_distance = distance
            max_yaw_index = i
        elif distance < min_distance:
            min_distance = distance
            min_yaw_index = i

    extreme_yaws = [yaws[min_yaw_index], yaws[max_yaw_index]]
    desired_angle = get_ideal_mean_angle(extreme_yaws)
    far_from_mean_distance = min([abs(max_distance), abs(min_distance)])
    if abs(shortest_angle_difference(mean_angle, desired_angle)[0]) > far_from_mean_distance:
        desired_angle += np.pi
    return desired_angle


# Function that gets the proposed ideal angle for the drones to meet using max empty arc and reversing it
# 1- For each drone calculate distance to the closest neighbor behind and the closest one in front
# 2- Take maximum of these distances and the two drones that create it
# 3- Find the middle of the found arc between them and take this point + pi as the ideal point
# 4- If the biggest distance is bigger than pi.. they are all on the same side and the middle of the arc is the desired angle
def get_ideal_angle_3(yaws):
    distance_list = []
    yaw_index_list = []
    
    for i in range(len(list(yaws))):
        min_pos_distance = float('inf')
        min_neg_distance = float('inf')
        min_pos_yaw_index = 0
        min_neg_yaw_index = 0
        for j in range(len(list(yaws))):
            if i != j:
                distance, distance2 = shortest_angle_difference(yaws[i], yaws[j])
                if distance >= 0:
                    if distance <= min_pos_distance:
                        min_pos_distance = distance
                        min_pos_yaw_index = j
                else:
                    if abs(distance) < min_neg_distance:
                        min_neg_distance = abs(distance)
                        min_neg_yaw_index = j
                
                if distance2 >= 0:
                    if distance2 <= min_pos_distance:
                        min_pos_distance = distance2
                        min_pos_yaw_index = j
                else:
                    if abs(distance2) < min_neg_distance:
                        min_neg_distance = abs(distance2)
                        min_neg_yaw_index = j

        # print("min_pos_distance is: ", min_pos_distance)
        # print("min_neg_distance is: ", min_neg_distance)

        distance_list.append(min_pos_distance)
        distance_list.append(min_neg_distance)
        yaw_index_list.append([i,min_pos_yaw_index])
        yaw_index_list.append([i,min_neg_yaw_index])

    max_empty_distance = max(distance_list)
    max_empty_distance_index = argmax(distance_list)
    # print("Max empty distance: ", max_empty_distance)
    # print("between drones: ", yaw_index_list[max_empty_distance_index])

    temp_yaw_list_index = yaw_index_list[max_empty_distance_index]
    temp_yaw_list = [yaws[temp_yaw_list_index[0]],yaws[temp_yaw_list_index[1]]]
    desired_angle = get_ideal_mean_angle(temp_yaw_list)
    if max_empty_distance < np.pi:
        desired_angle += np.pi

    return desired_angle


# Function that gets the proposed ideal angle by distributed consensus
# 1- Get mean angle
# 2- Find drones closest to the mean angle and vote that as the leader
# 3- Follow the leader. Leader stays at cruise speed.
def get_distributed_consensus_angle(yaws, my_id):
    global LEADER_DRONE
    mean_angle = get_ideal_mean_angle(yaws)
    min_distance = float('inf')
    min_yaw_index = 0
    leader_yaw = 0.0
    for i, yaw in enumerate(yaws):
        distance = abs(shortest_angle_difference(yaw, mean_angle)[0])
        if distance < min_distance:
            min_distance = distance
            min_yaw_index = i
            leader_yaw = yaw

    leader_drone_votes[my_id] = min_yaw_index
    desired_angle = leader_yaw
    return desired_angle

# Virtual yaw method:
def get_virtual_angle(t, yaws=[], my_id=0):
    loiter_time = 2*pi*r/SPEEDS['cruise']
    # calculate the yaw of the virtual drone
    desired_angle = (t % loiter_time) / loiter_time * 2 * pi
    return desired_angle

# Virtual yaw method with mean angle used as starting point
def get_virtual_angle_with_mean(t, yaws, my_id=0):
    global VIRTUAL_PHASE
    # if virtual phase is 0, we set it
    if VIRTUAL_PHASE == 0:
        VIRTUAL_PHASE = get_ideal_mean_angle(yaws)

    loiter_time = 2*pi*r/SPEEDS['cruise']
    # calculate the yaw of the virtual drone
    desired_angle = (t % loiter_time) / loiter_time * 2 * pi
    phased_desired_angle = desired_angle + VIRTUAL_PHASE
    return phased_desired_angle

# Function that implements change in speed based on pulses with drones reaching yaw 0
# 1- Go through all the angles and see if any are close to 0
# 2- If a group is at 0, change speed as if it's only one drone
# 3- depending on location, speed up or slow down by a step
def get_yaw0_pulses(t):
    global yaw0_timestamps
    global set_speeds
    YAW0_RANGE = 0.3*dt
    SPEED_RESET_DURATION = 5
    for i in range(N):
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
                    yaw0_timestamps[i] = t
                    if my_yaw > 0:
                        set_speeds[i] = SPEEDS['low']
                    else:
                        set_speeds[i] = SPEEDS['high']
                    break
        # print("Checking yaw0 timestamp: ", t, yaw0_timestamps[i])
        if ((t - yaw0_timestamps[i]) >= SPEED_RESET_DURATION) and (yaw0_timestamps[i]>0):
            set_speeds[i] = SPEEDS['cruise']
            yaw0_timestamps[i] = -1


# Function that implements change in speed based on pulses with drones reaching yaw 0
# 1- Go through all the angles and see if any are close to 0
# 2- If a group is at 0, change speed as if it's only one drone
# 3- depending on location, speed up or slow down by a step
def get_ramp_yaw0_pulses(t):
    global yaw0_timestamps
    global pulse_magnitudes
    global pulse_timestamps
    global set_speeds
    YAW0_RANGE = 0.3*dt
    SPEED_STEP = 0.5
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


# Function that implements change in speed based on pulses with drones reaching yaw 0 with jolt
# 1- Go through all the angles and see if any are close to 0
# 2- If a group is at 0, change speed as if it's only one drone
# 3- depending on location, speed up or slow down by a step
# 4- Apply random jolts to get out of balanced oscillations
def get_ramp_yaw0_random_jolt_pulses(t):
    global yaw0_timestamps
    global pulse_magnitudes
    global pulse_timestamps
    global set_speeds
    global jolt_timer
    YAW0_RANGE = 0.3*dt
    SPEED_STEP = 0.5
    SPEED_RESET_DURATION = 10
    random_options = np.array([True, False])
    JOLT_PROBABILITY = 0.005
    JOLT_RESET_DURATION = 20
    random_probabilities = np.array([JOLT_PROBABILITY, 1.0-JOLT_PROBABILITY]) # probability that a random choice is made
    random_jolt = np.random.choice(random_options, p=random_probabilities)
    if random_jolt and (t - jolt_timer >= JOLT_RESET_DURATION):
        jolt_timer = t
        print("JOLT INITIATED!")
    if t - jolt_timer >= JOLT_RESET_DURATION:
        for i in range(N):
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

                if ((t - pulse_timestamps[i,j]) >= SPEED_RESET_DURATION) and (pulse_timestamps[i,j]>0):
                    pulse_magnitudes[i,j] = 0.0
                    pulse_timestamps[i,j] = -1

            set_speeds[i] = SPEEDS['cruise'] + sum(pulse_magnitudes[i,:])
            if set_speeds[i] > SPEEDS['high']:
                set_speeds[i] = SPEEDS['high']
            elif set_speeds[i] < SPEEDS['low']:
                set_speeds[i] = SPEEDS['low']


# Function that implements change in speed based on pulses with drones reaching yaw 0 with jolt
# 1- Go through all the angles and see if any are close to 0
# 2- If a group is at 0, change speed as if it's only one drone
# 3- depending on location, speed up or slow down by a step
# 4- Finds the drones' groups based on proximity when pulses are happening
# 5- 
def get_ramp_yaw0_calculated_jolt_pulses(t):
    global yaw0_timestamps
    global pulse_magnitudes
    global pulse_timestamps
    global set_speeds
    global neighbor_count
    global same_state_count
    global jolt_timer
    global JOLT_ACTIVE
    YAW0_RANGE = 0.3*dt
    SPEED_STEP = 0.5
    SPEED_RESET_DURATION = 20
    old_neighbor_count = neighbor_count
    JOLT_RESET_DURATION = 40
    if t - jolt_timer < JOLT_RESET_DURATION:
        if not JOLT_ACTIVE:
            set_speeds = np.random.choice([SPEEDS['low'], SPEEDS['high']], size=set_speeds.shape)
            print("JOLTING!!!")
            print(set_speeds)
            JOLT_ACTIVE = True
            return
        else:
            return

    for i in range(N):
        temp_neighbor_count = 0
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
                else:
                    temp_neighbor_count += 1

            if ((t - pulse_timestamps[i,j]) >= SPEED_RESET_DURATION) and (pulse_timestamps[i,j]>0):
                pulse_magnitudes[i,j] = 0.0
                pulse_timestamps[i,j] = -1

        if temp_neighbor_count > neighbor_count[i]:
            neighbor_count[i] = temp_neighbor_count
        print("neighbor count for drone: ", i, " is: ", neighbor_count[i])
        set_speeds[i] = SPEEDS['cruise'] + sum(pulse_magnitudes[i,:])
        if set_speeds[i] > SPEEDS['high']:
            set_speeds[i] = SPEEDS['high']
        elif set_speeds[i] < SPEEDS['low']:
            set_speeds[i] = SPEEDS['low']
    
    # if we stay at the same state for 1000*dt = 100 seconds.. jolt the system
    if np.array_equal(old_neighbor_count, neighbor_count) and sum(neighbor_count) < 7:
        same_state_count += 1
        if same_state_count == 500:
            print("same state for too long.. starting jolt")
            same_state_count = 0
            jolt_timer = t
    else:
        same_state_count = 0




# function that finds closest pulse location to angle:
def get_nearest_pulse_location(yaw, pulse_locations):
    min_distance = float('inf')
    min_index = 0
    for i, pulse_location in enumerate(pulse_locations):
        distance = abs(shortest_angle_difference(yaw, pulse_location)[0])
        if distance < min_distance:
            min_distance = distance
            min_index = i
    
    return min_index


# Function that implements change in speed based on pulses with drones reaching yaw 0
# 1- Go through all the angles and see if any are close to 0
# 2- If a group is at 0, change speed as if it's only one drone
# 3- depending on location, speed up or slow down by a step
def get_ramp_yaw_n_pulses(t):
    global yaw0_timestamps
    global pulse_magnitudes
    global pulse_timestamps
    global set_speeds
    YAW0_RANGE = 0.3*dt
    SPEED_STEP = 0.5
    SPEED_RESET_DURATION = 10
    pulse_locations = np.arange(YAW_N_PULSE_NUM)*2*np.pi/YAW_N_PULSE_NUM

    for i in range(N):
        angles = known_angles[i,:]
        # change all angles to -pi to pi range
        angles = (angles + pi) % (2 * pi) - pi
        my_yaw = angles[i]
        for j, angle in enumerate(angles):
            if i == j:
                continue
            closest_pulse_index = get_nearest_pulse_location(angle, pulse_locations)
            distance = abs(shortest_angle_difference(angle, pulse_locations[closest_pulse_index])[0])
            if distance < YAW0_RANGE:
                # change set speed based on my_yaw
                # Adjust my yaw to be within -π to π range
                my_distance = shortest_angle_difference(my_yaw, pulse_locations[closest_pulse_index])[0]
                if  abs(my_distance) >= ACCEPTANCE_RANGE/2.0:
                    pulse_timestamps[i,j] = t
                    if my_distance < 0:
                        pulse_magnitudes[i,j] = -SPEED_STEP
                    else:
                        pulse_magnitudes[i,j] = SPEED_STEP

            if ((t - pulse_timestamps[i,j]) >= SPEED_RESET_DURATION) and (pulse_timestamps[i,j]>0):
                pulse_magnitudes[i,j] = 0.0
                pulse_timestamps[i,j] = -1

        set_speeds[i] = SPEEDS['cruise'] + sum(pulse_magnitudes[i,:])
        if set_speeds[i] > SPEEDS['high']:
            set_speeds[i] = SPEEDS['high']
        elif set_speeds[i] < SPEEDS['low']:
            set_speeds[i] = SPEEDS['low']

# Function that implements change in speed based on pulses with drones reaching yaw 0
# 1- Go through all the angles and see if any are close to 0
# 2- If a group is at 0, change speed as if it's only one drone
# 3- depending on location, speed up or slow down by a step
def get_ramp_yaw0_saturated_pulses(t):
    global yaw0_timestamps
    global pulse_magnitudes
    global pulse_timestamps
    global set_speeds
    YAW0_RANGE = 0.3*dt
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

# Function that implements change in speed based on pulses with drones reaching yaw 0
# 1- Go through all the angles and see if any are close to 0
# 2- If a drone is at 0, save the time with each pulse I get from other drones.
# 3- Based on when I reach the 0 yaw and create a pulse, I get an estimate of where those other drones were
# 4- calculate the mean of the estimate angles and make it a desired angle
def get_timed_pulses(t, yaws, my_id):
    global yaw0_timestamps
    global pulse_magnitudes
    global pulse_timestamps
    global set_speeds
    global angle_estimates
    YAW0_RANGE = 0.3*dt
    SPEED_STEP = 2.0 #2.0
    SPEED_RESET_DURATION = 10

    # change all angles to -pi to pi range
    yaws = (yaws + pi) % (2 * pi) - pi
    my_yaw = yaws[my_id]
    for j, yaw in enumerate(yaws):
        if abs(yaw) < YAW0_RANGE:
            pulse_timestamps[my_id,j] = t

    # check if im at 0 yaw to update the yaw estimates I have of the others
    if abs(my_yaw < YAW0_RANGE):
        # check if all the pulse timestamps got updated after the first run
        # if not, stay at cruise speed with desired_angle=my_yaw
        if (pulse_timestamps[i]<0).any():
            desired_angle = my_yaw
        else:
            # I reached the 0 point.
            # we reached 0 point so we estimate the location of all those drones based on their timesstamps
            # current timestamp - pulse_timestamps should give me an estimate of the angle difference. 
            # assuming cruise speed.. speed*time_diff=distance=r*theta => theta_diff = speed*time_diff/r
            time_diff = t - pulse_timestamps[i]
            angle_estimates[i] = my_yaw + SPEEDS['cruise']*time_diff/r
            desired_angle = get_ideal_mean_angle(angle_estimates[i])

        print("For drone: ", my_id, " angles estimates: ")
        print(angle_estimates[i])
        return desired_angle


# Function that implements change in speed based on pulses with drones reaching yaw 0 + kuramoto speed steps
# 1- Go through all the angles and see if any are close to 0
# 2- If a group is at 0, change speed as if it's only one drone
# 3- depending on location, speed up or slow down by a kuramoto speed difference
def get_ramp_yaw0_kuramoto_pulses(t):
    global yaw0_timestamps
    global pulse_magnitudes
    global pulse_timestamps
    global set_speeds
    YAW0_RANGE = 0.3*dt
    SPEED_RESET_DURATION = 40
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
                    pulse_magnitudes[i,j] = 2.0 * sin(0.0 - my_yaw)

            if ((t - pulse_timestamps[i,j]) >= SPEED_RESET_DURATION) and (pulse_timestamps[i,j]>0):
                pulse_magnitudes[i,j] = 0.0
                pulse_timestamps[i,j] = -1

        set_speeds[i] = SPEEDS['cruise'] + sum(pulse_magnitudes[i,:])
        if set_speeds[i] > SPEEDS['high']:
            set_speeds[i] = SPEEDS['high']
        elif set_speeds[i] < SPEEDS['low']:
            set_speeds[i] = SPEEDS['low']


# Function that implements change in speed based on pulses with drones reaching yaw 0
# 1- Go through all the angles and see if any are close to 0
# 2- If a group is at 0, change speed as if it's only one drone
# 3- depending on location, speed up or slow down by a step
def get_ramp_yaw_n_kuramoto_pulses(t):
    global yaw0_timestamps
    global pulse_magnitudes
    global pulse_timestamps
    global set_speeds
    global neighbors_matrix
    YAW0_RANGE = 0.3*dt
    SPEED_STEP = 0.5
    if YAW_N_PULSE_NUM == 1:
        SPEED_RESET_DURATION_MAX = 2*np.pi*r/SPEEDS['cruise'] # 100% of the cycle duration
    elif YAW_N_PULSE_NUM == 2:
        SPEED_RESET_DURATION_MAX = np.pi*r/SPEEDS['cruise'] # 50% of the cycle duration
    else:
        SPEED_RESET_DURATION_MAX = 0.5*np.pi*r/SPEEDS['cruise'] # 25% of the cycle duration
    pulse_locations = np.arange(YAW_N_PULSE_NUM)*2*np.pi/YAW_N_PULSE_NUM

    for i in range(N):
        # use neighbors number to adjust pulse duration
        total_neighbors = sum(neighbors_matrix[i])
        if total_neighbors <= 8:
            SPEED_RESET_DURATION = SPEED_RESET_DURATION_MAX
        else:
            SPEED_RESET_DURATION = 0.5*np.pi*r/SPEEDS['cruise'] # 25% of the cycle duration
        # SPEED_RESET_DURATION = np.pi*r/SPEEDS['cruise'] # 100% of the cycle duration

        angles = known_angles[i,:]
        # change all angles to -pi to pi range
        angles = (angles + pi) % (2 * pi) - pi
        my_yaw = angles[i]
        for j, angle in enumerate(angles):
            if i == j:
                continue
            closest_pulse_index = get_nearest_pulse_location(angle, pulse_locations)
            distance = abs(shortest_angle_difference(angle, pulse_locations[closest_pulse_index])[0])
            if distance < YAW0_RANGE:
                # change set speed based on my_yaw
                # Adjust my yaw to be within -π to π range
                my_distance = abs(shortest_angle_difference(my_yaw, pulse_locations[closest_pulse_index])[0])
                # update neighbor matrix
                if my_distance < np.pi/4:
                    neighbors_matrix[i,j] = 1
                else:
                    neighbors_matrix[i,j] = 0

                # get pulse magnitude and time
                if  my_distance >= ACCEPTANCE_RANGE/2.0:
                    pulse_timestamps[i,j] = t
                    pulse_magnitudes[i,j] = 2.0 * sin(pulse_locations[closest_pulse_index] - my_yaw)

            if ((t - pulse_timestamps[i,j]) >= SPEED_RESET_DURATION) and (pulse_timestamps[i,j]>0):
                pulse_magnitudes[i,j] = 0.0
                pulse_timestamps[i,j] = -1

        set_speeds[i] = SPEEDS['cruise'] + sum(pulse_magnitudes[i,:])
        if set_speeds[i] > SPEEDS['high']:
            set_speeds[i] = SPEEDS['high']
        elif set_speeds[i] < SPEEDS['low']:
            set_speeds[i] = SPEEDS['low']

# Function that implements the mean variation of kuramoto synchronization method:
# 1- get the mean angle
# 2- get sin() of difference of each angle and the mean
def get_mean_kuramoto_speeds(t):
    global set_speeds
    for i in range(N):
        yaws = known_angles[i,:]
        mean_angle = get_ideal_mean_angle(yaws)
        my_yaw = yaws[i]
        speed_diff = 2.0 * sin(mean_angle - my_yaw)
        set_speeds[i] = SPEEDS['cruise'] + speed_diff

# Function that implements the kuramoto synchronization method:
# 1- get difference from all other angles
# 2- get sin() of difference of each angle and the mean
def get_kuramoto_speeds(t):
    global set_speeds
    for i in range(N):
        yaws = known_angles[i,:]
        my_yaw = yaws[i]
        yaws = np.array(yaws)
        yaw_sum = sum(np.sin(yaws - my_yaw))
        speed_diff = 2.0*yaw_sum/N
        set_speeds[i] = SPEEDS['cruise'] + speed_diff

# Function that implements change in speed based on pulses with drones reaching yaw 0 + kuramoto speed steps
# 1- Go through all the angles and see if any are close to 0
# 2- If a group is at 0, change speed as if it's only one drone
# 3- depending on location, speed up or slow down by a kuramoto speed difference
def get_ramp_yaw0_kuramoto_improved_pulses(t):
    global yaw0_timestamps
    global pulse_magnitudes
    global pulse_timestamps
    global set_speeds
    global neighbors_matrix
    YAW0_RANGE = 0.3*dt
    SPEED_RESET_DURATION = 40
    for i in range(N):
        total_neighbors = sum(neighbors_matrix[i])
        if total_neighbors <= 8:
            SPEED_RESET_DURATION = 40
        else:
            SPEED_RESET_DURATION = 10
        angles = known_angles[i,:]
        # change all angles to -pi to pi range
        angles = (angles + pi) % (2 * pi) - pi
        my_yaw = angles[i]
        for j, angle in enumerate(angles):
            if i == j:
                continue
            if abs(angle) < YAW0_RANGE:
                # if my yaw is close to the 0 yaw consider this a neighbor
                distance = abs(shortest_angle_difference(my_yaw, 0.0)[0])
                if distance < np.pi/4:
                    neighbors_matrix[i,j] = 1
                else:
                    neighbors_matrix[i,j] = 0
                # change set speed based on my_yaw
                # Adjust my yaw to be within -π to π range
                if  abs(my_yaw) >= ACCEPTANCE_RANGE/2.0:
                    pulse_timestamps[i,j] = t
                    pulse_magnitudes[i,j] = 2.0 * sin(0.0 - my_yaw)

            if ((t - pulse_timestamps[i,j]) >= SPEED_RESET_DURATION) and (pulse_timestamps[i,j]>0):
                pulse_magnitudes[i,j] = 0.0
                pulse_timestamps[i,j] = -1

        set_speeds[i] = SPEEDS['cruise'] + sum(pulse_magnitudes[i,:])
        if set_speeds[i] > SPEEDS['high']:
            set_speeds[i] = SPEEDS['high']
        elif set_speeds[i] < SPEEDS['low']:
            set_speeds[i] = SPEEDS['low']


# Function that implements change in speed based on pulses with drones reaching yaw 0
# 1- Go through all the angles and see if any are close to 0
# 2- If a group is at 0, change speed as if it's only one drone
# 3- depending on location, only allowed to speed up
def get_yaw0_speedup_pulses(t):
    global yaw0_timestamps
    global set_speeds
    YAW0_RANGE = 0.3*dt
    SPEED_RESET_DURATION = 10
    for i in range(N):
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
                    if my_yaw < 0:
                        yaw0_timestamps[i] = t
                        set_speeds[i] = SPEEDS['high']
                        break
        # print("Checking yaw0 timestamp: ", t, yaw0_timestamps[i])
        if ((t - yaw0_timestamps[i]) >= SPEED_RESET_DURATION) and (yaw0_timestamps[i]>0):
            set_speeds[i] = SPEEDS['cruise']
            yaw0_timestamps[i] = -1

NUM_DRONES = 10
# yaw_list = np.zeros((1, NUM_DRONES))
# yaws1 = np.array([[0,45,90]])
# yaws2 = np.array([[-45,45,180]])
# yaws1 = np.array([[357.31562838338346,309.58075036630436,43.52038553009031,119.77026672964648,259.7343867299766,256.0290370903007,337.11861124780546,151.95851998610948,298.8128495787577,241.31000390906556]])
# yaws2 = np.array([[109.21266393585034,211.5290182116814,317.6924402994688,304.6310706341926,181.90217540865615,212.0408128737186,12.42929885448297,87.38639047550436,287.065529119549,149.15303974827873]])
# yaw_list = np.concatenate((yaw_list,yaws1,yaws2), axis=0 )
np.random.seed(0)
yaw_list = np.random.uniform(low=0.0, high=1.0, size=(100,NUM_DRONES)) * 360.0


r = 100
ACCEPTANCE_RANGE = 0.2
# SPEEDS = {'cruise':0.0, 'low':-10.0, 'high':10.0}
SPEEDS = {'cruise':17.0, 'low':15.0, 'high':19.0}
dt = 1.0
t0 = 0
tf = 900 # 15 mins #180#780 # 12 minutes # 180 # 3 minutes
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
global points
global metrics_data_list
global arrows
arrows = []
metrics_data_list = []
global speed_method_list
global state_sharing_offsets
# offsets for when state sharing happens
global shared_angles
global STATE_SHARING_OFFSET_ENABLED
STATE_SHARING_OFFSET_ENABLED = True
global STATE_SHARING_FREQUENCY
STATE_SHARING_FREQUENCY = 1
global CONTROL_FREQUENCY
CONTROL_FREQUENCY = 1

# variables for yaw methods:
global pulse_magnitudes
global pulse_timestamps
pulse_magnitudes = np.zeros((NUM_DRONES, NUM_DRONES)) # used for yaw0 and pulsing methods
pulse_timestamps = -np.ones((NUM_DRONES, NUM_DRONES))
global yaw0_timestamps
yaw0_timestamps = -np.ones(NUM_DRONES)
global angle_estimates
angle_estimates = np.zeros((NUM_DRONES, NUM_DRONES))
YAW_N_PULSE_NUM = 1
global jolt_timer
global neighbor_count
neighbor_count = np.zeros(NUM_DRONES)
global neighbors_matrix
neighbors_matrix = np.identity((NUM_DRONES))
global same_state_count
same_state_count = 0
global JOLT_ACTIVE
JOLT_ACTIVE = False

global zero_phase
zero_phase = 0.0

method = 'ideal3'
# methods that control speed directly
speed_method_list = ['yaw0', 'yaw0_speedup', 'yaw0_ramp', 'yaw0_ramp_saturated', 'kuramoto', 'mean_kuramoto', 'yaw0_ramp_kuramoto', 'yaw_n_ramp', 'yaw0_ramp_random_jolt', 'yaw0_ramp_calculated_jolt', 'yaw0_ramp_kuramoto_improved', 'yaw_n_ramp_kuramoto']
METHODS = {
    'ideal3': get_ideal_angle_3,
    'ideal2': get_ideal_angle_2,
    'mean': get_ideal_mean_angle,
    'ideal1': get_desired_angle_middlepoint,
    'dc': get_distributed_consensus_angle,
    'virtual': get_virtual_angle,
    'virtual_mean': get_virtual_angle_with_mean,
    'yaw0': get_yaw0_pulses,
    'yaw0_speedup': get_yaw0_speedup_pulses,
    'yaw0_ramp': get_ramp_yaw0_pulses,
    'yaw0_ramp_saturated': get_ramp_yaw0_saturated_pulses,
    'kuramoto': get_kuramoto_speeds,
    'mean_kuramoto': get_mean_kuramoto_speeds,
    'yaw0_ramp_kuramoto': get_ramp_yaw0_kuramoto_pulses,
    'yaw0_ramp_kuramoto_improved': get_ramp_yaw0_kuramoto_improved_pulses,
    'timed_pulses': get_timed_pulses,
    'yaw_n_ramp': get_ramp_yaw_n_pulses,
    'yaw0_ramp_random_jolt': get_ramp_yaw0_random_jolt_pulses,
    'yaw0_ramp_calculated_jolt': get_ramp_yaw0_calculated_jolt_pulses,
    'yaw_n_ramp_kuramoto': get_ramp_yaw_n_kuramoto_pulses,
}

DEBUG = True
VISUALIZATION = True

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
    if STATE_SHARING_OFFSET_ENABLED:
        used_angles = shared_angles
    else:
        used_angles = angles
    # get random choices of what is shared and what isnt
    random_options = np.array([True, False])
    random_probabilities = np.array([PROBABILITY_KNOWN, 1.0-PROBABILITY_KNOWN]) # probability that a random choice is made
    known_matrix = np.random.choice(random_options, size=known_angles.shape, replace=True, p=random_probabilities)
    np.fill_diagonal(known_matrix, True) # A drone always knows its own angle
    expanded_angles = np.tile(used_angles, (known_angles.shape[0],1))
    known_angles[known_matrix] = expanded_angles[known_matrix]

# This function updates the desired angles for each drone based on the known angles by each drone
def update_desired_angles(t):
    global desired_angles
    for i in range(desired_angles.shape[0]):
        if method == 'dc':
            # if not agreed on a leader we repeat the voting process
            if (not np.all(leader_drone_votes == leader_drone_votes[0])) or leader_drone_votes[0] < 0:
                desired_angles[i] = METHODS[method](known_angles[i,:], i)
            else:
                LEADER_DRONE = leader_drone_votes[0]
                desired_angles[i] = known_angles[i,LEADER_DRONE]
        elif method == 'virtual' or method == 'virtual_mean' or method == 'timed_pulses':
            desired_angles[i] = METHODS[method](t, known_angles[i,:], i)
        else:
            desired_angles[i] = METHODS[method](known_angles[i,:])

# Function that updates set speeds for each drone based on the desired angles found before
def update_set_speeds():
    for i in range(N):
        distance, drone_speedup = min_angle_distance(angles[i], desired_angles[i])
        # divide acceptance range by 2 to allow for +/- range from the middle angle
        if distance < ACCEPTANCE_RANGE/2.0:
            set_speeds[i] = SPEEDS['cruise']
        else:
            if drone_speedup:
                set_speeds[i] = SPEEDS['high']
            else:
                set_speeds[i] = SPEEDS['low']


def circle(phi):
    return np.array([r*np.cos(phi), r*np.sin(phi)])

# function that gets the arrow dx and dy for plotting
# should be the tangent to the circle at the yaw angle
# should be scaling with the speed received
def get_arrow(yaw, speed):
    arrow_angle = yaw + np.pi/2
    DEFAULT_MAGNITUDE = 20
    CRUISE_SPEED = 17.0
    arrow_magnitude = DEFAULT_MAGNITUDE * (speed - CRUISE_SPEED)/2.0
    dx = arrow_magnitude * cos(arrow_angle)
    dy = arrow_magnitude * sin(arrow_angle)
    return dx, dy

# Function that goes over list of yaws and determines if drones are in sync based on an acceptance range
def check_in_sync(yaw_list, acceptance_range):
    if len(yaw_list) < 2:
        print("Warning! Less than 2 angles are provided!")
    max_diff = 0.0
    # get the biggest difference between all angles
    for i in range(len(yaw_list)):
        for j in range(i+1, len(yaw_list)):
            diff = abs(shortest_angle_difference(yaw_list[i], yaw_list[j]))
            if diff > max_diff:
                max_diff = diff
                # stop searching if we already know we are not in sync
                if max_diff > acceptance_range:
                    return max_diff, False
    return max_diff, True

# Function that outputs a list into a csv file
def write_list_to_file(input_list, file_path):
    try:
        with open(file_path, 'a') as file:
            for item in input_list:
                if isinstance(item, list):
                    for i, element in enumerate(item):
                        file.write(str(element))
                        if i < len(item)-1:
                            file.write(',')
                    file.write('\n')
                else:
                    file.write(str(item) + '\n')

        print(f"List has been written to {file_path} successfully.")
    except Exception as e:
        print(f"An error occurred: {e}")

# Function that gets the data necessary and populates them in a list item
def get_data_list(timestamp, current_trial_num, airspeed, setspeed, phase_offsets, yaws, desired_angle, order_parameter):
    data = [timestamp, current_trial_num]
    for speed in airspeed:
        data.append(speed)
    for speed in setspeed:
        data.append(speed)
    for phase in phase_offsets:
        data.append(phase)
    for yaw in yaws:
        data.append(yaw)
    data.append(desired_angle)
    data.append(order_parameter)
    return data

# Function that calculates the order parameter from a list of yaws
def get_order_parameter(yaws):
    proj_x_axis = []
    yaws = np.array(yaws)
    N = yaws.shape[0]

    x = sum(np.cos(yaws))/N
    y = sum(np.sin(yaws))/N
    order_parameter = np.sqrt(x**2 + y**2)

    return order_parameter

# function that saves all the data into a csv file with similar format to the gazebo simulation
def save_data(t, trial_num):
    global metrics_data_list
    order_parameter = get_order_parameter(angles)
    data_item = get_data_list(t, trial_num, speeds, set_speeds, yaw_list[trial_num], angles, desired_angles[0], order_parameter)
    metrics_data_list.append(data_item)


# if __name__ == "__main__":
#     time_list = np.linspace(t0,tf,int(tf/dt), endpoint=False)
#     freq_list = [0.05, 0.1, 0.5, 1.0]
#     control_freq_list = [0.05, 0.1, 0.5,1.0]
#     offset_list = [True, False]
#     for F in freq_list:
#         for C in control_freq_list:
#             for offset_enabled in offset_list:
#                 for i in range(0, yaw_list.shape[0]):
#                     angles = yaw_list[i]*np.pi/180.0
#                     shared_angles = angles.copy()
#                     # create initial conditions
#                     set_speeds = np.ones(angles.shape)*SPEEDS['cruise']
#                     speeds = set_speeds #+ np.random.normal(loc=RANDOM_MEAN, scale=RANDOM_STD, size=set_speeds.shape)                    
#                     # amount of points
#                     N = len(angles)  

#                     # frequencies to try are: 0.05 0.1 0.5 1 10 20 100
#                     STATE_SHARING_FREQUENCY = F
#                     CONTROL_FREQUENCY = C

#                     dt = 0.1
#                     t = t0
#                     state_sharing_dt = int(10/F)
#                     control_dt = int(10/C)
#                     if offset_enabled:
#                         t_int = int(t0) + np.random.randint(low=0,high=10) # allows an offset in the state sharing updates
#                         if i == 0:
#                             state_sharing_offsets = np.random.randint(low=0, high=state_sharing_dt, size=(NUM_DRONES))
#                     else:
#                         t_int = int(t0)
#                         state_sharing_offsets = (state_sharing_dt*np.ones(shape=NUM_DRONES)).astype(int)
#                     while(t<=tf):
#                         t += dt
#                         t_int += 1

#                         zero_phase += SPEEDS['cruise']*dt/r
#                         # physical update of system
#                         d_angle = speeds*dt/r
#                         angles += d_angle
#                         angles[angles>(2*np.pi)] -= 2*np.pi

#                         # check if state sharing should be published for any drone
#                         # state sharing updates
#                         # if offset enabled.. check the offset for state sharing
#                         # if offset disabled.. update all shared angles together at dt_state_sharing
#                         shared_angles[t_int%state_sharing_offsets == 0] = angles[t_int%state_sharing_offsets == 0]
#                         # update_known_angles()
#                         known_angles = np.tile(shared_angles, (known_angles.shape[0],1))
#                         np.fill_diagonal(known_angles, angles) # each drone knows its own location
#                         update_desired_angles(t)
#                         update_set_speeds()

#                         # control updates:
#                         if t_int%control_dt:
#                             update_speeds()

#                         if t_int%(1000) == 0:
#                             print(t,speeds)

#                         save_data(t, i)

#                     if tf <= 300:
#                         OUTPUT_PATH = OUTPUT_PATH = 'simple_'
#                     else:
#                         OUTPUT_PATH = OUTPUT_PATH = 'simple_longer_'
#                     write_list_to_file(metrics_data_list, OUTPUT_PATH+method+"_" + str(F)+"_" + str(C)+"_" + str(offset_enabled) + "_data.csv")
#                     metrics_data_list = []


if __name__ == "__main__":
    time_list = np.linspace(t0,tf,int(tf/dt), endpoint=False)
    freq_list = [0.1]#[0.05, 0.1, 0.5, 1.0]
    control_freq_list = [0.1]#[0.05, 0.1, 0.5,1.0]
    offset_list = [True, False]
    for F in freq_list:
        for C in control_freq_list:
            for offset_enabled in offset_list:
                for i in range(0, yaw_list.shape[0]):
                    VISUALIZATION = True
                    ROTATING_FRAME = False
                    if VISUALIZATION:
                        plt.rcParams["figure.figsize"] = 8,6
                        plt.ion()
                        fig, ax = plt.subplots()

                        ax.axis([-1.5*r,1.5*r,-1.5*r,1.5*r])
                        ax.set_aspect("equal")
                        circ = plt.Circle((0, 0), radius=r, edgecolor='b', facecolor='None')
                        ax.set_aspect(1)
                        ax.add_artist(circ)

                    angles = yaw_list[i]*np.pi/180.0
                    shared_angles = angles.copy()
                    # create initial conditions
                    set_speeds = np.ones(angles.shape)*SPEEDS['cruise']
                    speeds = set_speeds.copy() #+ np.random.normal(loc=RANDOM_MEAN, scale=RANDOM_STD, size=set_speeds.shape)                    
                    # amount of points
                    N = len(angles)

                    if VISUALIZATION:
                        # create a point in the axes
                        points = []
                        ann_list = []
                        arrows = []
                        for j in range(N):
                            x,y = circle(angles[j])
                            ann = ax.annotate(str(j), xy=(x,y))
                            ann_list.append(ann)
                            points.append(ax.plot(x, y, marker="o", color='b')[0])
                            dx, dy = get_arrow(angles[j], speeds[j])
                            arrows.append(ax.arrow(x,y,dx,dy, head_width=8, length_includes_head=True))

                        for j in range(N):
                            x,y = circle(desired_angles[j])
                            ann = ax.annotate(str(j), xy=(x,y))
                            ann_list.append(ann)
                            points.append(ax.plot(x, y, marker="o", color='r')[0])
                        
                        for j in range(N):
                            x,y = circle(angles[j])
                            ann = ax.annotate(str(j), xy=(x,y))
                            ann_list.append(ann)
                            points.append(ax.plot(x, y, marker="o", color='m')[0])
                            dx, dy = get_arrow(angles[j], speeds[j])
                            arrows.append(ax.arrow(x,y,dx,dy, head_width=8, length_includes_head=True))
                        
                        # rotating frame of reference:
                        x,y = circle(zero_phase)
                        points.append(ax.plot(x, y, marker="o", color='g')[0])



                    # frequencies to try are: 0.05 0.1 0.5 1
                    STATE_SHARING_FREQUENCY = F
                    CONTROL_FREQUENCY = C

                    main_frequency = max([F,C])*NUM_DRONES

                    dt = (1/main_frequency)

                    integer_scale = main_frequency

                    t = t0
                    t_int = int(t0)*integer_scale

                    state_sharing_dt = 1/F
                    state_sharing_dt_int = integer_scale*state_sharing_dt
                    # state_sharing_offset_steps = state_sharing_dt/NUM_DRONES
                    control_dt = 1/C
                    control_dt_int = integer_scale*control_dt
                    if offset_enabled:
                        t_int += np.random.randint(low=0,high=state_sharing_dt_int) # allows an offset in the state sharing updates
                        if i == 0:
                            state_sharing_offsets = np.random.randint(low=0, high=state_sharing_dt_int, size=(NUM_DRONES))
                    else:
                        state_sharing_offsets = np.zeros(shape=NUM_DRONES).astype(int)
                    while(t<=tf):
                        t += dt
                        t_int += 1

                        if VISUALIZATION:
                            for k in range(len(ann_list)):
                                ann_list[k].remove()
                                if k < len(arrows):
                                    arrows[k].remove()
                            ann_list = []
                            arrows = []

                        if ROTATING_FRAME:
                            zero_phase += SPEEDS['cruise']*dt/r

                        # physical update of system
                        d_angle = speeds*dt/r
                        angles += d_angle
                        angles[angles>(2*np.pi)] -= 2*np.pi

                        # check if state sharing should be published for any drone
                        # state sharing updates
                        # if offset enabled.. check the offset for state sharing
                        # if offset disabled.. update all shared angles together at dt_state_sharing
                        shared_angles[t_int%state_sharing_dt_int == state_sharing_offsets] = angles[t_int%state_sharing_dt_int == state_sharing_offsets]
                        # update_known_angles()
                        known_angles = np.tile(shared_angles, (known_angles.shape[0],1))
                        
                        # control updates:
                        if t_int%control_dt_int == 0:
                            np.fill_diagonal(known_angles, angles) # each drone knows its own location
                            update_desired_angles(t)
                            update_set_speeds()
                            update_speeds()

                        if t_int%(control_dt_int) == 0:
                            print(t,speeds)

                        if VISUALIZATION:
                            # goal points:
                            for k in range(N):
                                x,y = circle(shared_angles[k] - zero_phase)
                                ann = ax.annotate(str(k), xy=(x,y))
                                ann_list.append(ann)
                                points[k].set_data([x],[y])
                                dx, dy = get_arrow(shared_angles[k] - zero_phase, speeds[k])
                                arrows.append(ax.arrow(x,y,dx,dy, head_width=8, length_includes_head=True))
                                x,y = circle(desired_angles[k] - zero_phase)
                                # ann = ax.annotate(str(k), xy=(x,y))
                                # ann_list.append(ann)
                                points[k+N].set_data([x],[y])
                                
                                x,y = circle(angles[k] - zero_phase)
                                ann = ax.annotate(str(k), xy=(x,y))
                                ann_list.append(ann)
                                points[k+2*N].set_data([x],[y])
                                dx, dy = get_arrow(angles[k] - zero_phase, speeds[k])
                                arrows.append(ax.arrow(x,y,dx,dy, head_width=8, length_includes_head=True))
                            
                            x,y = circle(zero_phase)
                            points[3*N].set_data([x],[y])

                            fig.canvas.draw()
                            fig.canvas.flush_events()
                        else:
                            save_data(t, i)
            
                    if VISUALIZATION:
                        plt.close()
                    else:

                        if tf <= 300:
                            OUTPUT_PATH = OUTPUT_PATH = 'simple_'
                        else:
                            OUTPUT_PATH = OUTPUT_PATH = 'simple_longer_'
                        write_list_to_file(metrics_data_list, OUTPUT_PATH+method+"_" + str(F)+"_" + str(C)+"_" + str(offset_enabled) + "_data.csv")
                    metrics_data_list = []