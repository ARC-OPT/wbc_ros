[![ROS 2 Jazzy](https://github.com/ARC-OPT/wbc_ros/actions/workflows/build_and_test_ubuntu24.04.yml/badge.svg)](https://github.com/ARC-OPT/wbc_ros/actions/workflows/build_and_test_ubuntu24.04.yml)

# wbc_ros - A ROS 2 interface for the WBC library

[Code API](https://arc-opt.github.io/wbc_ros/)  | [Full Documentation](https://arc-opt.github.io/Documentation/)

This package provides a ros2 control interface for the [Whole-Body Control library](https://github.com/ARC-OPT/wbc).

WBC was initiated and is currently developed at the [Robotics Innovation Center](http://robotik.dfki-bremen.de/en/startpage.html) of the [German Research Center for Artificial Intelligence (DFKI)](http://www.dfki.de) in Bremen.

<img src="doc/images/DFKI_Logo_e_schrift.jpg" alt="drawing" width="300"/>

## Requirements / Dependencies

Currently supported OS: Ubuntu22.04, Ubuntu24.04

wbc_ros requires a ROS2 installation (humble is the only tested distribution) as well as the [WBC library](https://github.com/ARC-OPT/wbc). See package.xml for further dependencies.

## Installation

See [here](https://arc-opt.github.io/Documentation/installation/installation_ros2.html) for installation instructions.

## Testing

### Unit tests

Unit tests can be found in the [wbc library](https://github.com/ARC-OPT/wbc).

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

Check out the tutorials in the [documentation(https://arc-opt.github.io/Documentation/).

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
