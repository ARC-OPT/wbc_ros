# wbc_ros - A ROS interface for the Whole-Body Control library in ARC-OPT

[Code API](TODO)  | [Full Documentation](https://arc-opt.github.io/Documentation/)

This task library provides a ROS interface for the Whole-Body Control library.

WBC was initiated and is currently developed at the [Robotics Innovation Center](http://robotik.dfki-bremen.de/en/startpage.html) of the [German Research Center for Artificial Intelligence (DFKI)](http://www.dfki.de) in Bremen.

<img src="doc/images/DFKI_Logo_e_schrift.jpg" alt="drawing" width="300"/>

## Getting Started

* Please check out the tutorials section in the [documentation](https://arc-opt.github.io/Documentation/) for examples of usage.

## Requirements / Dependencies

Currently supported OS: Ubuntu20.04, Ubuntu22.04

wbc_ros requires a bare-bones ROS installation (noetic is the only tested distribution) as well as the [WBC library](https://github.com/ARC-OPT/wbc). See package.xml for further dependencies.

## Installation

* Install ROS noetic as described [here](http://wiki.ros.org/noetic/Installation/Ubuntu)
* Install the WBC library as described [here](https://arc-opt.github.io/Documentation/installation/installation_no_rock.html)
* Create a catkin workspace as described [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) or use an existing one
* If not yet done, install rosdep as described [here](wiki.ros.org/rosdep)
* In a console type
 ```
 cd ~/my_catkin_workspace/src
 git clone https://git.hb.dfki.de/dfki-control/wbc/wbc_ros
 git clone https://git.hb.dfki.de/dfki-control/wbc/wbc_msgs
 cd ..
 rosdep install wbc_ros
 catkin_make wbc_ros
 ```

## Documentation

Doygen documentation can be generated with [rosdoc_lite](http://wiki.ros.org/rosdoc_lite)

## Testing

For testing the ROS interface you can run ```rostest wbc_ros test_wbc.test```.

The unit tests for the wbc library can be found [here](https://github.com/ARC-OPT/wbc/tree/master/test).

## Contributing

Please use the [issue tracker](TODO) to submit bug reports and feature requests.

Please use merge requests as described [here](TODO) to add/adapt functionality.

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
