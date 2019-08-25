# ** multiuav_collision_avoidance ** #

This repository containts an algorithm for collision avoidance between drones based on collision-free velocity assignment. The algorithm checks conflicts in a time windows in a centralized manner, assigning velocities that would avoid collsions in an optimal manner. The ROS package `collision_free_vel` contains the implementation of the algorithm, while the package `experiments` contains files to run simulations or actual experiments for testing and processing results.

## Dependencies ##

 * [grvc-ual](https://github.com/grvcTeam/grvc-ual)
 * [PX4 Firmware](https://github.com/PX4/Firmware) at tag [v1.6.3](https://github.com/PX4/Firmware/tree/v1.6.3) (if you want to run SITL simulations)
 
## **Installation** ##

You can create a catkin workspace, clone the repository there and compile it:

```
    mkdir -p [workspace]/src
    cd [workspace]
    catkin_make
    git clone --recursive https://github.com/jescap/multiuav_collision_avoidance.git ./src
    catkin build
```

## **Usage** ##

A sample simulation without SITL can be run with:

```
roslaunch experiments simulation_light.launch
```

A sample simulation with the PX4 autopilot SITL can be run with:

```
roslaunch experiments simulation.launch
```

Run a central controller executing the collision avoidance algorithm with:
```
roslaunch collision_free_vel controller.launch 
```

**NOTE**: Use configuration files in `conf` folder to specify number of drones, heights and goal points. 

Additionally, you can run a visualization node to publish markers for RVIZ:
```
roslaunch collision_free_vel visualization.launch 
```
