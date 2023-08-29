# Whole Body Controller Parameters

Default Config
```yaml
whole_body_controller:
  ros__parameters:
    control_mode: ""
    command_interfaces: {}
    integrate: false
    task_names: {}
    contact_names: {}
    joint_weights: {}
    robot_model.file: ""
    robot_model.type: "rbdl"
    robot_model.submechanism_file: ""
    robot_model.floating_base: false
    solver.type: "qp_oases"
    solver.file: ""
    scene.type: "velocity_qp"
    scene.file: ""
```

## control_mode

Control mode of the wbc. Can be one of [velocity,acceleration].

* Type: `string`
* Default Value: ""


*Constraints:*
 - parameter is not empty
 - one of the specified values: ['velocity', 'acceleration']

## command_interfaces

Command interfaces to claim from hardware.

* Type: `string_array`
* Default Value: {}


*Constraints:*
 - parameter is not empty

## integrate

Do numerical integration on the solver output, e.g. if output is acceleration type, integrate twice to get velocity and position

* Type: `bool`
* Default Value: false

## task_names

Names of the tasks to be configured in task configuration. Each task will be a summand in the QP solver's cost function.

* Type: `string_array`
* Default Value: {}


*Constraints:*
 - parameter is not empty

## contact_names

Link names that are in contact with the environment. These have to be valid link names in the robot model.

* Type: `string_array`
* Default Value: {}


*Constraints:*
 - contains no duplicates

## joint_weights

The joint weights control the contribution of each individual joint to the solution. Values have to be within [0,1]. A zero means here that the joint is not used at all. Size has to be same as number of robot joints.

* Type: `double_array`
* Default Value: {}


*Constraints:*
 - parameter is not empty
 - each element of array must be greater than or equal to 0.0
 - each element of array must be less than or equal to 1.0

## robot_model.file

Absolute path to plain URDF file of the robot

* Type: `string`
* Default Value: ""


*Constraints:*
 - parameter is not empty

## robot_model.type

Robot model type. Must be the exact name of one of the registered robot model plugins. See src/robot_models for all available plugins.*/

* Type: `string`
* Default Value: "rbdl"


*Constraints:*
 - one of the specified values: ['rbdl', 'pinocchio', 'kdl', 'hyrodyn']

## robot_model.submechanism_file

Only if type:=hyrodyn. Absolute path to submechanism file, which the robot's parallel submechanisms. See Hyrodyn docs for more info.

* Type: `string`
* Default Value: ""

## robot_model.floating_base

Optionally attach a virtual 6 DoF floating base to the model. Naming scheme: [floating_base_trans_x, floating_base_trans_y, floating_base_trans_z, floating_base_rot_x, floating_base_rot_y, floating_base_rot_z]. The floating must not be added to the URDF model.

* Type: `bool`
* Default Value: false

## robot_model.contact_points.<contact_name>.active

Is the link initially in contact?

* Type: `int`
* Default Value: 0

*Constraints:*
 - one of the specified values: [0,1]

## robot_model.contact_points.<contact_name>.mu

Friction coeffcient for the contact point.

* Type: `double`
* Default Value: 0.0

*Constraints:*
 - must be greater than 0.0
 
## robot_model.contact_points.<contact_name>.wx

x-dimension of the contact surface (only for surface contacts).

* Type: `double`
* Default Value: 0.0

*Constraints:*
 - must be greater than 0.0
 
## robot_model.contact_points.<contact_name>.wy

y-dimension of the contact surface (only for surface contacts).

* Type: `double`
* Default Value: 0.0

*Constraints:*
 - must be greater than 0.0

## solver.type

QP Solver type. Must be the exact name of one of the registered QP solver plugins. See src/solvers for all available plugins.

* Type: `string`
* Default Value: "qp_oases"

*Constraints:*
 - one of the specified values: ['qpoases', 'hls', 'proxqp', 'qpswift', 'eiquadprog']

## solver.file

Absolute path to solver configuration file in yaml format. If empty, default solver configuration is used.

* Type: `string`
* Default Value: ""

## scene.type

Scene type. Must be the exact name of one of the registered scene plugins. See src/scenes for all available plugins.

* Type: `string`
* Default Value: "velocity_qp"

*Constraints:*
 - parameter is not empty
 - one of the specified values: ['velocity', 'velocity_qp', 'acceleration', 'acceleration_tsid', 'acceleration_reduced_tsid']

## scene.file

Absolute path to scene configuration file in yaml format. If empty, default scene configuration is used.

* Type: `string`
* Default Value: ""

## tasks.<task_name>.type

Task type, can be one of 'jnt' (joint space task), 'cart' (Cartesian task) or 'com' (Center of Mass task).

* Type: `string`
* Default Value: ""

*Constraints:*
 - one of the specified values: ['jnt', 'cart', 'com']

## tasks.<task_name>.weights

Initial weights for this task. Size has to be same as number of task variables, e.g. number of joint names in joint space tasks, and 6 in case of a Cartesian task. All entries have to be >= 0.  Can be used to balance contributions of the task variables. A value of 0 means that the reference of the corresponding task variable will be ignored while computing the solution.

* Type: `double_array`
* Default Value: {}

*Constraints:*
 - each element of array must be greater than or equal to 0.0
 - Array must not be empty
 
## tasks.<task_name>.activation

Initial activation for this task. Has to be within 0 and 1. Can be used to enable(1)/disable(0) the whole task, or to apply a smooth activation function.
                              
* Type: `double`
* Default Value: 0.0

*Constraints:*
 - must be greater than or equal to 0.0 and smaller than or equal to 1.0

## tasks.<task_name>.priority

Priority of this task. 0 corresponds to the highest priority. Prioritization is only supported by the hls solver.

* Type: `int`
* Default Value: 0

*Constraints:*
- must be greater than or equal to 0

## tasks.<task_name>.root

Only Cartesian tasks:  Root frame of the kinematic chain associated with this task. Has to be the name of a valid link in the robot model. Only robot_model.type!=kdl supports root frames, which are not equal to the root of the URDF tree.
                              
* Type: `string`
* Default Value: ""

## tasks.<task_name>.tip

Only Cartesian tasks:  Tip frame of the kinematic chain associated with this task. Has to be the name of a valid link in the robot model.

* Type: `string`
* Default Value: ""

## tasks.<task_name>.ref_frame

Only Cartesian tasks:  Reference frame of the task input (frame wrt which the input is expressed). This has to be the name of a valid link in robot model. If ref_frame != root the input will be transformed to the root frame.
                              
* Type: `string`
* Default Value: ""

## tasks.<task_name>.joint_names

Only joint space tasks: Names of the joints involved in this task.

* Type: `string_array`
* Default Value: {}

*Constraints:*
 - contains no duplicates


## tasks.<task_name>.timeout

Timeout of this task in seconds. Output for this task will be set to zero if, for more than this amount of time, no new reference is set. A value of 0 will be interpreted as infinite.
                              
* Type: `double`
* Default Value: 0.0

*Constraints:*
- must be greater than or equal to 0.0

