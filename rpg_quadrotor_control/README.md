# RPG Quadrotor Control

## License

The RPG Quadrotor Control repository provides packages that are intended to be used with [ROS](http://www.ros.org/).

## Summary

This repository contains a complete framework for flying quadrotors based on control algorithms developed by the [Robotics and Perception Group](http://www.ifi.uzh.ch/en/rpg.html).
We also provide an interface to the [RotorS](https://github.com/ethz-asl/rotors_simulator) Gazebo plugins to use our algorithms in simulation.
Together with the provided simple trajectory generation library, this can be used to test our sofware entirely in simulation.
We also provide some utility to command a quadrotor with a gamepad through our framework as well as some calibration routines to compensate for varying battery voltage.
Finally, we provide an interface to communicate with flight controllers used for First-Person-View racing.
