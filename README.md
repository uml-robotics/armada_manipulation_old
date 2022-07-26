# This repository is being archived to maintain work but halt development. Please instead refer to newer maipulation packages in the uml-robotics gihub library such as: https://github.com/uml-robotics/armada_behaviors which is in development as of the time of this writing (07/22)



# Table of Contents

- [Important](#important) 
- [Supported Versions](#supported-versions)
- [Pick and Place](#pick-and-place)
  - [Setup](#setup)
  - [Usage](#usage)
- [Additional Resources](#additional-resources)

# Important
This repository only contains code used to control Moveit! configured robots (real or simulated), and should not contain any models or configurations of its own. Other packages may be required for use  

This README will be revised and reuploaded soon once more testing has been completed and a few tweaks to workflow implemented.

# Supported Versions
All branches have been tested with ROS Kinetic + Melodic.

# Pick and Place
The `uml_hri_nerve_pick_and_place` package provides some C++ code and architecture for students working at the NERVE Center or in the HRI Lab at UMass lowell to develop a percepton-based pick and place task/exercise using simulated and real industrial robots such as the Kinova Jaco2, Gen3, and the Universal Robots UR5e, among others. Students will contribute to the package by adding in more behaviors and creating demonstrable activities utilizing hardware such as vision systems and grippers as well as existing grasping algorithms.  

## Setup
The code contained within this package uses Moveit! to control the robots both in simulation and on the actual hardware.   
Certain pieces of code may require other packages (gripper-specific packages or grasping/planning packages) in order to function properly.

## Usage
A full demonstration can be performed using the following commands:  

- In the first terminal bring up the robot simulation (gazebo and rviz): 
```
roslaunch ur_e_gazebo gazebo_universal_robot.launch arm:=ur5 gripper:=robotiq_2f_85 sim_workstation:=true
```

- In the second terminal run the node that will control the robot:  
```
rosrun uml_hri_nerve_pick_and_place pick_and_place manipulator workstation
```

- In the third terminal spawn an object into gazebo using a command such as (if you don't get this in before the first scan it's okay, the code will loop):  
```
rosrun gazebo_ros spawn_model -file ~/.gazebo/gazebo_models/coke_can/model.sdf -sdf -z 1.2 -model soda
```

- You can locate your .gazebo folder (hidden folder if you use the file manager) and create a models folder if one does not already exist  
- Model files can be cloned from the following repository:  
[osrf/gazebo_models](https://github.com/osrf/gazebo_models)  
- The coke_can is easy for the robotiq_2f_85 gripper to grab  

# Additional Resources
Any additional packages (such as the ur_e_gazebo package from the uml-robotics/universal_robot repository used in the example above) can be acquired by following the links provided in my other package. Please read through the readme, this hyperlink will take you to the section detailing integrated/connected packages along with instructions for usage.  
[uml-robotics/uml_hri_nerve_armada_workstation#integrated_packages](https://github.com/uml-robotics/uml_hri_nerve_armada_workstation#integrated-packages)  
