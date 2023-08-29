[![Ubuntu 22.04](https://github.com/ARC-OPT/wbc_ros/actions/workflows/build_and_test_ubuntu22.04.yml/badge.svg)](https://github.com/ARC-OPT/wbc_ros/actions/workflows/build_and_test_ubuntu22.04.yml)

# wbc_ros - A ROS interface for the WBC library

[Code API](https://arc-opt.github.io/wbc_ros/)  | [Full Documentation](https://arc-opt.github.io/Documentation/)

This package provides a ros2 control interface for the [Whole-Body Control library](https://github.com/ARC-OPT/wbc).

WBC was initiated and is currently developed at the [Robotics Innovation Center](http://robotik.dfki-bremen.de/en/startpage.html) of the [German Research Center for Artificial Intelligence (DFKI)](http://www.dfki.de) in Bremen.

<img src="doc/images/DFKI_Logo_e_schrift.jpg" alt="drawing" width="300"/>

## Subscribed Topics
## Published Topics
## Parameters
* [Whole Body Controller](https://github.com/ARC-OPT/wbc_ros/blob/main/doc/parameters/whole_body_controller_parameters.md)

## Requirements / Dependencies

Currently supported OS: Ubuntu20.04, Ubuntu22.04

wbc_ros requires a ROS2 installation (humble is the only tested distribution) as well as the [WBC library](https://github.com/ARC-OPT/wbc). See package.xml for further dependencies.

## Installation

See [here](https://arc-opt.github.io/Documentation/installation/installation_ros.html) for installation instructions.

## Testing

### Unit tests

The unit tests for the wbc library can be found [here](https://github.com/ARC-OPT/wbc/tree/master/test).

### Launch Tests

Run 
```
colcon test --packages-select wbc_ros
``` 
to execute all launch tests which can be found [here](https://github.com/ARC-OPT/wbc_ros/tree/main/test). For more verbosity, you can also execute the tests manually by typing 
```
launch_test install/wbc_ros/share/wbc_ros/test/<test_name>.test.py
```

### Examples

Ensure that robot state publisher and joint state publisher are installed:
```
sudo apt-get install ros-humble-robot-state-publisher ros-humble-joint-state-publisher -y
```
For visualizing the resulting robot motion install rviz:
```
sudo apt-get install ros-humble-rviz2
source install/setup.bash
```
For the Cartesian space example run 
```
ros2 launch wbc_ros cartesian_space_example.launch.py
rviz2 -d install/wbc_ros/share/wbc_ros/config/default.rviz
``` 
You should see the kuka iiwa robot executing a circular end effector motion in the xy-plane. For the joint space example run
```
ros2 launch wbc_ros joint_space_example.launch.py
rviz2 -d install/wbc_ros/share/wbc_ros/config/default.rviz
```
You can see the kuka iiwa robot executing a sinusoidal movement with the elbow joint. 

## Contributing

Please use the [issue tracker](https://github.com/ARC-OPT/wbc_ros/issues) to submit bug reports and feature requests.

Please use merge requests as described [here](https://github.com/ARC-OPT/wbc_ros/blob/main/CONTRIBUTING.md) to add/adapt functionality.

## License

wbc_ros is distributed under the [3-clause BSD license](https://opensource.org/licenses/BSD-3-Clause).

## Acknowledge WBC

If you use WBC within your scientific work, please cite the following publication:

```
@INPROCEEDINGS{mronga2022,
author = "D. Mronga and S.Kumar and F.Kirchner",
title = "Whole-Body Control of Series-Parallel Hybrid Robots",
year = "2022",
note = "{2022 IEEE International Conference on Robotics and Automation (ICRA)}, Accepted for publication",
}
```

## Funding

WBC has been developed in the research projects [TransFit](https://robotik.dfki-bremen.de/en/research/projects/transfit/) (Grant number 50RA1701) and [BesMan](https://robotik.dfki-bremen.de/en/research/projects/besman.html) (Grant number 50RA1216) funded by the German Aerospace Center (DLR) with funds from the German Federal Ministry for Economic Affairs and Climate Action (BMWK). It is further developed in the [M-Rock](https://robotik.dfki-bremen.de/en/research/projects/m-rock/) (Grant number 01IW21002) and [VeryHuman](https://robotik.dfki-bremen.de/en/research/projects/veryhuman/) (Grant number  01IW20004) projects funded by the German Aerospace Center (DLR) with federal funds from the German Federal Ministry of Education and Research (BMBF).

## Maintainer / Authors / Contributers

Dennis Mronga, dennis.mronga@dfki.de

Copyright 2017, DFKI GmbH / Robotics Innovation Center
