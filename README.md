# Distributed Loitering Synchronization with Fixed-Wing UAVs

## Overview
This repository contains the implementation of distributed loitering synchronization algorithms for fixed-wing Unmanned Aerial Vehicles (UAVs) as presented in the paper "Distributed Loitering Synchronization with Fixed-Wing UAVs" by Ahmed AlKatheeri, Agata Barciś, and Eliseo Ferrante.

Distributed loitering synchronization is the process whereby a group of fixed-wing UAVs align with each other while following a circular path in the air. This process is essential to establish proper initial conditions for missions in the real world, especially where coordinated deployment is required.

## The Challenge
Fixed-wing UAVs require a runway for takeoff, making it difficult to deploy a swarm simultaneously. Our approach enables drones to:
1. Take off individually
2. Loiter at different altitudes on coincident circles
3. Synchronize in the air by adjusting their speeds
4. Start their mission together once synchronized

## Synchronization Methods
This repository implements three primary synchronization algorithms:

### 1. Baseline Method (Mean Synchronization)
The baseline method is based on distributed consensus and uses the mean phase as the synchronization target.

- **Implementation**: `mean_angle_synchronization.py`
- **Algorithm**:
  - Each UAV calculates the mean heading angle of all drones
  - The mean heading is determined by converting each angle to a position vector on a unit circle and finding their centroid:
    ```
    p̄ = (1/N) * ∑(j=1 to N) e^(iθj)
    ```
  - The synchronization target is computed as:
    ```
    θ = Arg(p̄)
    ```
  - UAVs adjust their speeds to reach this target

### 2. MOSA Method (Minimum Of Shortest Arc)
The MOSA method improves upon the baseline by finding the shortest arc containing all drones and setting the synchronization target as the middle point of that arc.

- **Implementation**: `ideal_synchronization_3.py`
- **Algorithm**:
  1. For each drone, calculate the distance to the closest neighbor behind and in front
  2. Find the maximum of these distances (the largest empty arc)
  3. Find the middle point of the arc between the two drones that create this maximum distance
  4. Add π to this point to find the ideal meeting point (if the arc is less than π)
  5. This becomes the desired synchronization angle
  - Mathematically, MOSA finds the middle point of the arc and adds π:
    ```
    θmosa = π + mean([θjmax, θmmax])
    ```
  - This ensures all drones contribute to synchronization effectively

### 3. FPS Method (Firefly multi-Pulse Synchronization)
The FPS method is inspired by firefly synchronization and uses pulse-coupled oscillators, requiring significantly less communication while maintaining performance.

- **Implementation**: `ramp_yaw_n_kuramoto_synchronization.py`
- **Algorithm**:
  1. The circle is divided into K points (pulse locations)
  2. Each drone sends a pulse when it reaches one of these K predefined positions
  3. The pulse contains only the index of the position (requiring only log₂(K) bits)
  4. Upon receiving a pulse, drones adjust their speeds proportionally to their distance from the pulse location:
     ```
     Δv = Δvmax * sin(θpulse - θ)
     ```
  5. Speed adjustments reset after a pulse reset duration
  6. This approach reduces communication by a factor of 10 compared to the baseline

## Key Parameters

The system uses the following key parameters:
- `N`: Number of UAVs
- `R`: Radius of the loitering circle
- `vcruise`: Cruise speed of the UAVs
- `vlow`: Minimum speed (vcruise - Δvmax)
- `vhigh`: Maximum speed (vcruise + Δvmax)
- `YAW0_RANGE`: Angle threshold for pulse emission
- `ACCEPTANCE_RANGE`: Angle threshold for synchronization
- `SPEED_RESET_DURATION`: Time after which speed adjustments reset
- `LOITER_RADIUS`: Radius of the loitering circle

## Performance Comparison

- **MOSA**: Outperforms the baseline in continuous motion scenarios
- **FPS**: Requires 10x less communication while maintaining comparable performance to the baseline
- The algorithms have been validated through:
  - Simple simulation
  - Realistic simulation using Gazebo with fixed-wing dynamics
  - Real-world flights using 3 fixed-wing drones

## Usage

### Parameters Configuration
Configure parameters in `loitering_sync_params.yml` including:

# Key parameters
chosen_method: 'mean'  # Options: 'ideal3' (MOSA), 'ramp_yaw_n_kuramoto' (FPS), 'mean' (Baseline)
N_YAW: 4  # Number of pulse locations for FPS method
ACCEPTANCE_RANGE: 0.2  # Synchronization threshold in radians
LOITER_RADIUS: 150  # Radius of loitering circle in meters
CONTROL_DELAY: 1.0  # Control loop time step

### Running the Synchronization
bash
ros2 run lrs_loitering_sync loitering_sync
Running Metrics and Visualization
bash
ros2 run lrs_loitering_sync loitering_sync_metrics

## References
AlKatheeri, A., Barciś, A., & Ferrante, E. (2023). Distributed Loitering Synchronization with Fixed-Wing UAVs.
## License
[License information]
This README provides a comprehensive explanation of the three synchronization methods (baseline, MOSA, and FPS) from the paper, with proper mathematical notation that should render correctly in most markdown viewers. You can copy and paste this directly into your repository.
