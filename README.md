# Baseline Algorithms and Benchmarking for F1Tenth
This repository contains a benchmarking pipeline for testing algorithms in the F1Tenth Simulator as well as on an actual F1Tenth vehicle accompanying this [paper](Benchmarking_Algorithms_for_Autonomous_Racing.pdf).


>**Drive the car for at least one, preferably two laps, to ensure the benchmarking can provide accurate results.**



## Requirements and Installation
### Vehicle
For setting up the software stack on the vehicle follow: https://github.com/TUM-AVS/F1TENTH_Auxiliaries/tree/main/F1TENTH%20-%20Lab%20Setup \
then clone this repo and run:

    git clone <github_url>
    cd <repo_ws>
    colcon build --symlink-install
    source install/setup.bash
    export PYTHONPATH=${PYTHONPATH}:/usr/lib/python3.8/site-packages/range_libc-0.1-py3.8-linux-aarch64.egg
    colcon build --symlink-install
this enables running the baseline algorithms and data collection on the vehicle

To be able to store ros2bags in mcap storage format install:

    sudo apt-get install ros-foxy-rosbag2-storage-mcap

### Simulation
Install the F1tenth Simulation as usual and use the f1tenth gym ros provided in this folder instead, this enables managing maps and racelines via one python file for all algorithms and the simulation

### Benchmarking
For running the benchmarking script create a venv with: \
python 3.8.10 \
pandas 2.0.3 \
matplotlib 3.7.5 \
numpy 1.22.0 \
PyYAML 6.0.1 \
opencv-python 4.10.0.84


## Usage
Parameters of the algorithms can be adjusted in their repsective python file, usually at the very top of the file. \
Re-building is not necessary when the "--symlink-install" flag was used.
Start the algorithm before starting the particle filter, sometimes map creation fails, in this case stop and restart the particle fitler if it fails twice. Stop the algorithm and the particle filter and restart both.

### Baseline Algorthims in the simulation
In the [ws_params.py](src/ws_params.py) adjust the paths so they match the absolute paths to the desired map and trajectory.
Launch simulation:

    ros2 launch f1tenth_gym_ros gym_bridge_launch.py
For algorithms requiring the  position of the vehicle (all baseline algorithms except follow the gap), launch the particle filter. If the initial pose is incorrect, use "2D Pose Estimate" to fix it.

    ros2 launch particle_filter sim_localize_launch.py 

#### Follow the Gap
    ros2 run follow_the_gap follow_the_gap

#### Pure Pursuit
    ros2 run pure_pursuit pure_pursuit

#### Stanley
    ros2 run stanley stanley

#### Stanley Avoidance 
    ros2 launch stanley_avoidance sim_stanley_avoidance_launch.py

### Baseline Algorithms on the vehicle
In the ws_params.py adjust the paths so they match the absolute paths to the desired map and trajectory. Launch the F1Tenth Stack. Launch the algorithms before starting the particle filter. Sometimes map generation fails on the first try. In this case stop and restart the particle filter. If it still doesn't work restart algorithm and particle fitler accordingly.
#### Follow the Gap
    ros2 run follow_the_gap follow_the_gap
#### Particle filter
    ros2 launch particle_filter localize_launch.py
#### Pure Pursuit
    ros2 launch pure_pursuit pure_pursuit_launch.py
#### Stanley
    ros2 launch stanley stanley_launch.py
#### Stanley Avoidance
    ros2 launch stanley_avoidance stanley_avoidance_launch.py

### Baseline Algorithms Parameters
#### Follow the Gap
Parameters are found in [Line 14-21](src/follow_the_gap/follow_the_gap/follow_the_gap.py#L14-21)
- bubble radius: Defines the radius of the ’bubble’ around the vehicle, used in the Follow the Gap algorithm to ignore points within a certain distance. Increasing this radius may prevent the vehicle from steering into walls but can also cause the vehicle to miss narrower gaps.
- smoothing filter size: This parameter affects the size of the filter used to smooth the LiDAR data. A larger filter size will result in smoother data, which can help the vehicle make more stable steering decisions, but may also reduce the sensitivity to smaller obstacles or gaps.
- truncated coverage angle: Specifies the angular range of the LiDAR that is considered for detecting gaps. Reducing this angle can help focus on the most relevant obstacles in front of the vehicle, potentially improving performance in tight turns.
- max accepted distance: Determines the maximum distance at which an object is consid-
ered an obstacle. Objects beyond this range are ignored. Reducing this value might help the
car ignore distant obstacles that are not immediate threats, but setting it too low could lead
to collisions with overlooked obstacles.
- error based velocities.low, .medium, .high: These parameters set different speeds for the vehicle based on the steering angle. For sharp turns (high error), the vehicle uses the ’high’ error velocity, which is the slowest speed setting. For moderate turns, it uses the ’medium’ velocity, and for slight turns or straight paths, it uses the ’low’ velocity (which is set to the highest speed). Adjusting these can fine-tune how aggressively the vehicle slows down for turns.
- steering angle reactivity: This coefficient affects how quickly the steering angle is adjusted based on the distance to the closest obstacle. A higher value will make the vehicle steer away from close obstacles more aggressively, which can be useful in cluttered environments but may result in too high steering angles.

#### Pure Pursuit
Parameters are found in [Line 35-39](src/pure_pursuit/pure_pursuit/pure_pursuit_pf_raceline_data.py#L35-39)
- wheel base: the wheel base of the vehicle necessary for the calculation
- max steering angle: the allowed maximal steering angle in radian
- speed percentage: The speed is simply set to the target speed of the reference point of the vehicle's current position and multiplied with the percentage. If you want to additionally slow down proportional to the steering angle uncomment [line 145](src/pure_pursuit/pure_pursuit/pure_pursuit_pf_raceline_data.py#L145). You can also adjust the formula there if desired.
- min_dist_point_reached and max_dist_point_reached: Define the maximum and minimum values for the lookahead distance. In difference to the original algorithm, the vehicle doesn't select the next target point when the current target point is reached, but as soon as the distance from the vehicle to the current target point is smaller than the lookahead distance the target is moved to the next waypoint of the raceline (iterating through all points of the raceline). The lookahead distance is calculated based on the steering angle the curvature of the raceline at the position closest to the vehicle and the curvature of the raceline in the current target point.
    - min_dist_point_reached: adjusts how close the vehicle needs to get to the target point in curvy parts of the racetrack. Too large values result in corner cutting. Too small values result in the vehicle not reaching the point when driving past/over it resulting in a target point behind the vehicle causing it to turn around on the racetrack or possibly to crash
    - max_dist_point_reached: adjusts how far ahead the vehicle looks on straight. Larger values can reduce oscillation, but can also lead to corner cutting.
    - Note: The min and max values interact due to the calculation of the lookahead distance. Changing the max value also influences the behavior in curvy parts of the track and changing the min value also influences the behavior on straigher parts.
#### Stanley
Parameters are found in [Line 21-24](src/stanley/stanley/stanley.py#L21-24).
- velocity goal: The speed is simply set to the target speed of the reference point of the vehicle's current position and multiplied with the value of the velocity goal.
- k_e: Weight for the Cross Track Error. Penalizes deviation from the optimal raceline with regard to the position. Higher values should lead to vehicle driving closer to the optimal raceline, but might also cause oscillation, resulting in a larger Heading Error.
- k_h: Weight for the Heading Error. Penalizes deviation from the optimal raceline with regard to the heading. Higher values should lead to vehicle steering more similarly to the optimal raceline, but might cause more devation from the optimal raceline with regard to the postion, resulting in a larger Cross track Error.
- lf: Distance from the center of gravity of the vehicle to its front achsle.
- lr: Distance from the center of gravity of the vehicle to its rear achsle.
- max steering angle: the allowed maximal steering angle in radian

#### Stanley Avoidance
This code is almost unchanged from it's [original](https://github.com/fjahncke/f1tenth_ws_waterloo). The parameters are declared as ROS parameters and then the parameter values are read from the config yaml-file. To not require colcon-building each time after changing a parameter, the parameters can be changed directly in the python file. 
- k_e and k_h [Line 86-87](src/stanley_avoidance/stanley_avoidance/stanley_avoidance.py#L86-87) : same impact as in Stanley (see above)
- velocity perecentage [Line 96](src/stanley_avoidance/stanley_avoidance/stanley_avoidance.py#L96) : same thing as velocity goal in stanley just a different name
- PID parameters [Line 88-90](src/stanley_avoidance/stanley_avoidance/stanley_avoidance.py#L88-90) : Typical PID Controller. For detailed explanation on how to tune a PID Controller consult the [Lab 9 Handout](handout_lab_9_PID_Controller.pdf)

#### Only for Experts
Only change these values of you have a profoud understanding of the whole algorithms code.
- The calculation of the lookahead distance for the pure pursuit algorithm can be edited in [Line 155-157](src/pure_pursuit/pure_pursuit/pure_pursuit_pf_raceline_data.py#L155-157) if desired.
- Stanley Avoidance has more tunable parameters. However some are unused and some are targeted for use cases with objects to avoid (e.g. another vehicle).

### Recording data to rosbag
For saving data to a Rosbag run this command choose a name to save it to and list the topics to record. The necessary topics for the benchmarking pipeline are given for the [simulation](bags_sim/BagTopics_sim.txt) and the [vehcile](bags/BagTopics.txt)

    ros2 bag record -s mcap -o choosen_name /topic1 /topic2
On the vehicle first source the F1Tenth Stack overlay and then this workspace's overlay to get all necessary messsage definitions.

### Benchmarking
 If you run it parallel to the simulation (or a rosbag from a simulation) set the [Simulation-Flag](src/benchmarking/benchmarking/data_from_rosbag.py#L38) to 'True' beforehand else to 'False'. To run the data recorder node parallel to an algorithm or the replay of a rosbag use:

    ros2 run benchmarking read_data
when finished stop the node wiht CTR+C then enter the name for the csv file e.g. "Example1" and confirm with enter. The data will then be saved to data/Example1.csv

In the [benchmarking script](src/benchmarking.py) define the following paths:
1. [results_path](src/benchmarking.py#L13) : path to save the resulting metrics per lap to
2. [data_path](src/benchmarking.py#L14) : path to file generated via the data recorder node, which shall be evaluated
3. [raceline_path](src/benchmarking.py#L15) : path to used pre-computed raceline
4. [map_path](src/benchmarking.py#L16) : path to the used map. To enable plotting of the racetrack outline, the map has to be saved as PNG-file. 
Then choose the [laps](src/benchmarking.py#L19) to plot.
If another car than the Traxxas Slash was used, adjust the values for the [wheelbase](src/benchmarking.py#L23)  and [lf_veh](src/benchmarking.py#L22)  accordingly.

To calculate the metrics and show the plots simply run the script.


## Authors and acknowledgment
Huge thanks to [Felix Jahncke](https://github.com/fjahncke) for making this whole project possible and laying much of the groundwork. Parts of this code are based on https://github.com/fjahncke/f1tenth_ws_waterloo which is an adjusted version of https://github.com/CL2-UWaterloo/f1tenth_ws. 

The Main context for this Software package is contained in the Semester's Thesis "Benchmarking Algorithms for Autonomous Racing" by Jonathan Mohr, supervised by Felix Jahncke.

Felix Jahncke ([Website](https://www.mos.ed.tum.de/en/avs/team/felix-jahncke/)) leads the TUM F1Tenth/RoboRacer project at the Professorship of Autonomous Vehicle Systems under the supervision of Professor Johannes Betz ([Website](https://www.mos.ed.tum.de/en/avs/team/prof-dr-ing-johannes-betz/)).

