# Example:

## Build the node

```
 mkdir colcon_ws
 mkdir colcon_ws/src
 cd colcon_ws/src
 git clone https://github.com/picknikrobotics/generate_parameter_library.git
 cd ..
 colcon build
```

## Run the node

```
source install/setup.bash
ros2 run generate_parameter_library_example test_node --ros-args --params-file src/generate_parameter_library/example/config/implementation.yaml

```

You should see an output like this:
`[INFO] [1656018676.015816509] [admittance_controller]: Control frame is: 'ee_link'`

## ROS 2 CLI

Run the following:

`ros2 param list`

You should see:

```
/admittance_controller:
  admittance.damping_ratio
  admittance.mass
  admittance.selected_axes
  admittance.stiffness
  chainable_command_interfaces
  command_interfaces
  control.frame.external
  control.frame.id
  enable_parameter_update_without_reactivation
  fixed_array
  fixed_string
  fixed_string_no_default
  fixed_world_frame.frame.external
  fixed_world_frame.frame.id
  ft_sensor.filter_coefficient
  ft_sensor.frame.external
  ft_sensor.frame.id
  ft_sensor.name
  gravity_compensation.CoG.force
  gravity_compensation.CoG.pos
  gravity_compensation.frame.external
  gravity_compensation.frame.id
  interpolation_mode
  joints
  kinematics.alpha
  kinematics.base
  kinematics.group_name
  kinematics.plugin_name
  kinematics.plugin_package
  kinematics.tip
  one_number
  pid.elbow_joint.d
  pid.elbow_joint.i
  pid.elbow_joint.p
  pid.rate
  pid.shoulder_lift_joint.d
  pid.shoulder_lift_joint.i
  pid.shoulder_lift_joint.p
  pid.shoulder_pan_joint.d
  pid.shoulder_pan_joint.i
  pid.shoulder_pan_joint.p
  pid.wrist_1_joint.d
  pid.wrist_1_joint.i
  pid.wrist_1_joint.p
  pid.wrist_2_joint.d
  pid.wrist_2_joint.i
  pid.wrist_2_joint.p
  pid.wrist_3_joint.d
  pid.wrist_3_joint.i
  pid.wrist_3_joint.p
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  scientific_notation_num
  state_interfaces
  three_numbers
  three_numbers_of_five
  use_feedforward_commanded_input
  use_sim_time
  ```

All parameter are automatically declared and callbacks are setup by default. You can set a parameter by typing:

`ros2 param set /admittance_controller control.frame.id new_frame`

You should see:

`[INFO] [1656019001.515820371] [admittance_controller]: Control frame is: 'new_frame'`

Congratulations, you updated the parameter!

If you try to set a parameter that is read only, you will get an error. Running the following

`ros2 param set /admittance_controller command_interfaces ["velocity"]`

will result in the error

`Setting parameter failed: parameter 'command_interfaces' cannot be set because it is read-only`

Running the following

`ros2 param describe /admittance_controller admittance.damping_ratio`

will show a parameter's description

 ```
 Parameter name: admittance.damping_ratio
  Type: double array
  Description: specifies damping ratio values for x, y, z, rx, ry, and rz used in the admittance calculation. The values are calculated as damping can be used instead: zeta = D / (2 * sqrt( M * S ))
  Constraints:
    Min value: 0.1
    Max value: 10.0
```

If you try to set a value out of the specified bounds,

`ros2 param set /admittance_controller admittance.damping_ratio [-10.0,-10.0,-10.0,-10.0,-10.0,-10.0]`

you will get the error

`Setting parameter failed: Invalid value '-10' for parameter 'admittance.damping_ratio'. Required bounds: [0.1, 10]`

If you try to set a vector parameter with the wrong length,

`ros2 param set /admittance_controller admittance.damping_ratio [1.0,1.0,1.0]`

you will get the error

`Setting parameter failed: Invalid length '3' for parameter 'admittance.damping_ratio'. Required equal to: 6`

If you try to load a yaml file with missing required parameters

`ros2 run generate_parameter_library_example test_node --ros-args --params-file src/generate_parameter_library/example/config/missing_required.yaml`

you will get the error

```
terminate called after throwing an instance of 'rclcpp::exceptions::ParameterUninitializedException'
  what():  parameter 'fixed_string_no_default' is not initialized
[ros2run]: Aborted
```
