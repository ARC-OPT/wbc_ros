# Joint Position Controller Parameters

Default Config
```yaml
joint_position_controller:
  ros__parameters:
    control_mode: ""
    wbc_name: ""
    task_name: ""
    joint_names: {}
    p_gain: {}
    d_gain: {}
    ff_gain: {}
    max_control_output: {}
    dead_zone: {}
```

## control_mode

Control mode of the wbc. Can be one of [velocity,acceleration].

* Type: `string`
* Default Value: ""


*Constraints:*
 - parameter is not empty
 - one of the specified values: ['velocity', 'acceleration']

## wbc_name

Name of the WBC controller.

* Type: `string`
* Default Value: ""


*Constraints:*
 - parameter is not empty

## task_name

Name of the WBC task this controller is assigned to.

* Type: `string`
* Default Value: ""


*Constraints:*
 - parameter is not empty

## joint_names

Joint names used in the controller.

* Type: `string_array`
* Default Value: {}


*Constraints:*
 - parameter is not empty
 - contains no duplicates

## p_gain

P-Gain of the controller, size has to be same as no joints.

* Type: `double_array`
* Default Value: {}


*Constraints:*
 - parameter is not empty

## d_gain

P-Gain of the controller, size has to be 6.

* Type: `double_array`
* Default Value: {}


*Constraints:*
 - parameter is not empty

## ff_gain

Feed forward gain of the controller, size has to be 6.

* Type: `double_array`
* Default Value: {}


*Constraints:*
 - parameter is not empty

## max_control_output

Controller output saturation per element. Size has to be 6.

* Type: `double_array`
* Default Value: {}


*Constraints:*
 - parameter is not empty

## dead_zone

Dead zone for the position error per element. Size has to be 6.

* Type: `double_array`
* Default Value: {}


*Constraints:*
 - parameter is not empty
