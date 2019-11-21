# TurtleBot3 Demo

## Usage

### Install ROS packages and Build
```sh
(Move to your catkin workspace)
$ cd ~/catkin_ws/src/

(Download packages)
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_demo.git

(Install dependencies)
http://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup
$ sudo apt-get install ros-kinetic-laser-filters
$ sudo apt-get install ros-kinetic-realsense-camera

(Build)
$ cd ~/catkin_ws && catkin_make

(Copy parameter files to TB3 navigation and bringup pkg)
$ rospack profile
$ cp `rospack find turtlebot3_demo`/nav_param/costmap_common_params_waffle_pi.yaml `rospack find turtlebot3_navigation`/param/
$ cp `rospack find turtlebot3_demo`/nav_param/dwa_local_planner_params_waffle_pi.yaml `rospack find turtlebot3_navigation`/param/
$ cp `rospack find turtlebot3_demo`/nav_param/global_costmap_params.yaml `rospack find turtlebot3_navigation`/param/
$ cp `rospack find turtlebot3_demo`/nav_param/local_costmap_params.yaml `rospack find turtlebot3_navigation`/param/
$ cp `rospack find turtlebot3_demo`/nav_param/turtlebot3_lidar.launch `rospack find turtlebot3_bringup`/launch/
```

### Execute ROS packages
```sh
(Remote PC)
$ roscore

(Turtlebot3 PC)
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_demo turtlebot3_demo.launch 

(Remote PC)
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_demo remote_rviz.launch 
```

### Param Settings
- sim_time: 1.0
- global: 
  - width: 3.0
  - height: 3.0
  - resolution: 0.01
  - inflation_radius: 0.4
  - cost_scaling_factor: 5.0
- local: 
  - width: 1.0
  - height: 1.0
  - resolution: 0.005
  - inflation_radius: 0.4
  - cost_scaling_factor: 5.0
  - footprint: x 0.8

### Reference
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [turtlebot3 eManual](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
