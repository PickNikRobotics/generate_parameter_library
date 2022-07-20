# generate_parameter_library
This package aims to automate parameter handling within ROS 2. The build script automatically generates a c++ library from a yaml file input. The generated library contains a struct with all parameters as nested fields. Addtional methods are generated, which enable parameter updating.   

# Usage
You must include the generate_parameter_library pakcage in your cmake project.

`find_package(generate_parameter_library REQUIRED)`

You then need call the `generate_parameter_library` function with the following arguments:
```
LIB_NAME # cmake target name for the parameter library
YAML_FILE # path to input yaml file
VALIDATE_HEADER # Optional path to header containing custom validators
```

See [example project](example/CMakeLists.txt) to understand how to use this.

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
ros2 run gen_param_lib_example test_node --ros-args --params-file src/example/config/implementation.yaml

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
  fixed_world_frame.frame.external
  fixed_world_frame.frame.id
  ft_sensor.frame.external
  ft_sensor.frame.id
  ft_sensor.name
  gravity_compensation.CoG.force
  gravity_compensation.CoG.pos
  gravity_compensation.frame.external
  gravity_compensation.frame.id
  joints
  kinematics.base
  kinematics.plugin_name
  kinematics.tip
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  state_interfaces
  use_sim_time
  ```
  
  All parametter are automatically declared and callbacks are setup by default. You can set a parameter by typing:
  
  `ros2 param set /admittance_controller control.frame.id new_frame`
  
  You should see:
  
  `[INFO] [1656019001.515820371] [admittance_controller]: Control frame is: 'new_frame'`
  
  Congratulations, you updated the parameter! 
  
 If you try to set a parameter that is read only, you will get an error. Running the follwing
  
  `ros2 param set /admittance_controller joints ["joint_new"]`
  
  will result in the error
  
  `Setting parameter failed: parameter 'joints' cannot be set because it is read-only`
  
 Running the follwing
 
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

`Setting parameter failed: Invalid value for parameter admittance.damping_ratio. Value not within required bounds.`

If you try to set a vector parameter with the wrong length, 

`ros2 param set /admittance_controller admittance.damping_ratio [1.0,1.0,1.0]`

you will get the error

`Setting parameter failed: Invalid size for vector parameter admittance.damping_ratio. Expected 6 got 3`

If you try to load a yaml file with missing required parameters

`ros2 run generate_parameter_library_example test_node --ros-args --params-file src/generate_parameter_library/generate_parameter_library_example/include/config/missing_required.yaml`


you will get the error
```
terminate called after throwing an instance of 'rclcpp::exceptions::ParameterUninitializedException'
  what():  parameter 'joints' is not initialized
[ros2run]: Aborted
```
