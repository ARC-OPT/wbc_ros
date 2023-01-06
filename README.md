# wbc_ros - A ROS interface for the Whole-Body Control library in ARC-OPT

[Code API](TODO)  | [Full Documentation](https://arc-opt.github.io/Documentation/)

This task library provides a ROS interface for the Whole-Body Control library.

WBC was initiated and is currently developed at the [Robotics Innovation Center](http://robotik.dfki-bremen.de/en/startpage.html) of the [German Research Center for Artificial Intelligence (DFKI)](http://www.dfki.de) in Bremen.

<img src="doc/images/DFKI_Logo_e_schrift.jpg" alt="drawing" width="300"/>

## Motivation

WBC is a framework for optimization-based control of redundant robots. It contains various implementations of whole-body feedback control approaches on velocity-, acceleration- and force/torque-level. WBC is meant for controlling robots with redundant degrees of freedom, like humanoids or other legged robots with floating base, but also fixed-base systems like mobile manipulators, dual-arm systems or even simple manipulators. It is also meant for controlling multiple tasks simultaneously while taking into account the physical constraints of the robot. E.g., on a humanoid robot do ... (1) keep balance (2) Grasp an object with one arm (3) maintaining an upright body posture (4) Consider the joint torque limits,  etc... WBC is a purely reactive approach, i.e., it does not involve any motion planning or trajectory optimization. However, it can be used to stabilize trajectories coming from a motion planner or trajectory optimizer and integrate them with other objectives and physical constraints of the robot.


The general idea of optimization-based robot control is to formulate simultaneously running robot tasks as constraints or within the cost function of an instantaneous optimization problem. 
Now, in each control cycle ...

  * The constraints/cost functions are updated with the current robot state/control reference
  * The optimization problem is solved
  * The solution is applied to the actuators of the robot

The online solution of this problem is the robot joint control signal that complies with all tasks, while integrating physical constraints like actuator limits. An advantage of this approach is that complex tasks can be composed from low-dimensional descriptors, which are typically  easier to specify and control than the complete task are once. Also, the redundancy of the robot is nicely exploited utilizing  all the dof of the system (whole body).

## Getting Started

* Please check out the tutorials section in the [documentation](https://arc-opt.github.io/Documentation/) for examples of usage.

## Requirements / Dependencies

Currently supported OS: Ubuntu20.04, Ubuntu22.04

wbc_ros requires a bare-bones ROS installation (noetic is the only well-tested distribution). See package.xml for further dependencies.


## Installation

* Install ROS noetic as described [here](http://wiki.ros.org/noetic/Installation/Ubuntu)
* Install the WBC library as described [here](https://arc-opt.github.io/Documentation/installation/installation_no_rock.html)

## Testing

TODO

## Contributing

Please use the [issue tracker](TODO) to submit bug reports and feature requests.

Please use merge requests as described [here](TODO) to add/adapt functionality. 

## License

orogen-wbc is distributed under the [3-clause BSD license](https://opensource.org/licenses/BSD-3-Clause).

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


