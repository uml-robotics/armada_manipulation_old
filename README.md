# Table of Contents

- [Important](#important) 
- [Supported Versions](#supported-versions)
- [Pick and Place](#pick-and-place)
  - [Setup](#setup)

# Important
This repository only contains code used to control Moveit! configured robots (real or simulated), and should not contain any models or configurations of its own. Other packages may be required for use  

# Supported Versions
All branches have been tested with ROS Kinetic + Melodic.

# Pick and Place
The `uml_hri_nerve_pick_and_place` package provides some C++ code and architecture for students working at the NERVE Center or in the HRI Lab at UMass lowell to develop a percepton-based pick and place task/exercise using simulated and real industrial robots such as the Kinova Jaco2, Gen3, and the Universal Robots UR5e, among others. Students will contribute to the package by adding in more behaviors and creating demonstrable activities utilizing hardware such as vision systems and grippers as well as existing grasping algorithms.  

## Setup
The code contained within this package uses Moveit! to control the robots both in simulation and on the actual hardware.   
Certain pieces of code may require other packages (gripper-specific packages or grasping/planning packages) in order to function properly.

## Usage
In one terminal:
rosrun uml_hri_nerve_pick_and_place pick_and_ace manipulator workstation

In a separate terminal you can spawn an object into gazebo using a command such as:
rosrun gazebo_ros spawn_model -file ~/.gazebo/models/coke_can/model.sdf -sdf -z 1.2 -model soda

You can locate your .gazebo folder (hidden folder if you use the file manager) and create a models folder if one does not already exist. Model files can be cloned from the following repository:
[osrf/gazebo_models](https://github.com/osrf/gazebo_models)

I like to use the coke_can because it is easy for the robotiq_2f_85 gripper to grab
