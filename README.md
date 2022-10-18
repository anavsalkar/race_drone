Racing Drones Simulation Framework
===============

This repositories contains various packages to simulate drone racing. This is primarily based on [Flightmare](https://github.com/uzh-rpg/flightmare), [RotorS](https://github.com/ethz-asl/rotors_simulator), [RPG Quadrotor Control](https://github.com/uzh-rpg/rpg_quadrotor_control) and [RPG MPC](https://github.com/uzh-rpg/rpg_mpc).


Installation Instructions - Developed on Ubuntu 20.04 ROS Melodic
---------------------------------------------------------
 1. Install system dependencies:

 ```
 $ sudo apt-get install libgoogle-glog-dev protobuf-compiler ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-joy python3-vcstool

 ```
 2. Get catkin tools with the following commands:

 ```
 $ sudo apt-get install python-pip
 $ sudo pip install catkin-tools
 ```

 3. Create a catkin workspace with the following commands:

   ```
   $ cd
   $ mkdir -p catkin_ws/src
   $ cd catkin_ws
   $ catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

   ```

 4. Clone this repository and build:

   ```
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/anavsalkar/race_drone
   $ catkin build
   ```

5. Add sourcing of your catkin workspace and FLIGHTMARE_PATH environment variable to your .bashrc file:
   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ echo "export FLIGHTMARE_PATH=~/catkin_ws/src/flightmare" >> ~/.bashrc
   $ source ~/.bashrc
   ```

6. Follow the instructions on [this](https://flightmare.readthedocs.io/en/latest/getting_started/quick_start.html#download-flightmare-unity-binary) page to download Flighmare Unit Binary.


Basic Usage
-----------
All the nodes can be launched from a single command:
```
$ cd ~/catkin_ws/src
$ catkin build
$ source devel/setup.bash
$ roslaunch rpg_quadrotor_integration_test my_racing.launch
```
Differential flatness-based controller is used by default. To use model predictive control add arguement `use_mpc:=true` at the end of the launch command. All the gate positions and trajectory generation options can be specified in the `rpg_quadrotor_control/test/rpq_quadrotor_integration_test/src/my_racing.cpp`. 

Contact me at anavsalkar@gmail.com additional questions. 

 