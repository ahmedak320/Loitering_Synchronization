# Loitering Synchronization

## Goal:
The goal of this project is to implement synchronized loitering in mission mode with 2 or more drones.
This is part of the bigger [lrs_environment project](https://bitbucket.org/autonomouscv/lrs_environment/src/master/).

## Mission Plan:
Each drone would contain a mission plan that has the following waypoints:
    takes off
    goes to necessary height with a second waypoint
    loiters indefinitely as a third waypoint
    then a landing waypoint

The synchronized loitering methods start controlling the speed of the drones when they reach the third waypoint (indefinite loitering).
The drones speed up, slow down, or just cruise to reach a point where they are all pointed in the same direction as they loitering about in a circle.

## Contents:

### General Files:
The file loitering_sync.py is the main file that calls the synchronization methods.
The file loitering_sync_params.yml contains all the parameters to be set. Including choosing one of the synchronization methods listed below.
The file loitering_sync_base_class.py contains a base class called LoiteringSyncBase, which can be used as a base for any method implemented.

### Synchronization Methods:
Three methods are currently implemented to get the drones synchronized:
#### virtual yaw method: 
- No communication between the drones. The drone compares itself to a virtual drone and tries to synchronize with that.
- The position of the virtual drone is determine using the ratio of current time to total time to complete single rotation
- The total time to complete a single rotation is calculated using: loiter_time = 2*pi*loiter_radius / cruise_speed
#### ideal method:
- The state of each drone is shared every STATE_PUBLISHING_DT timestep. 
- All drones know the positions of each other. 
- Equations are used to determine the shortest arc that contains all the drones.
- The middle point of the shortest arc is the ideal angle that all drones try to reach. 
- As they move, this ideal angle is updated.
#### yaw0 method:
- A pulse is shared by each drone that reaches a yaw angle of 0 radians. 
- Reduced communication compared to ideal angle method.
- When the drones receive a pulse, they compare their position to the 0 radians position.
- They speed up or slow down to get closer to 0 radians. The change in speed is reset after a few seconds.


