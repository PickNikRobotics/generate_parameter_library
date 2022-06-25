# gen_param_struct
This package aims to automate parameter handling within ROS 2. The build script automatically generates a c++ struct from a yaml file input. The generated struct contains all parameters as nested fields. Addtional methods are generated, which enable parameter updating.   

# Usage
You must include the gen_param_struct pakcage in your cmake project.

`find_package(gen_param_struct REQUIRED)`

You then need call the `generate_param_struct_header` function with the following arguments:
```
OUT_DIR # output directory for generated struct
YAML_FILE # path to yaml file
YAML_TARGET # root name in yaml file to parse, e.g. joint_trajectory_controller  
```

# Example:
## build the node
```
 mkdir colcon_ws
 mkdir colcon_ws/src
 cd colcon_ws/src 
 git clone https://github.com/pac48/gen_param_struct.git
 cd ..
 colcon build
```

## run the node
```
source install/setup.bash
ros2 run gen_param_struct_example test_node
```

You should see an output like this:
`[INFO] [1656018676.015816509] [minimal_publisher]: Joint 0 is: 'shoulder_pan_joint'`

## ROS 2 CLI
Run the following:

`ros2 param list`

You should see:


```
  /minimal_publisher:
  admittance.damping_ratio.rx
  admittance.damping_ratio.ry
  admittance.damping_ratio.rz
  admittance.damping_ratio.x
  admittance.damping_ratio.y
  admittance.damping_ratio.z
  admittance.mass.rx
  admittance.mass.ry
  admittance.mass.rz
  admittance.mass.x
  admittance.mass.y
  admittance.mass.z
  admittance.selected_axes.rx
  admittance.selected_axes.ry
  admittance.selected_axes.rz
  admittance.selected_axes.x
  admittance.selected_axes.y
  admittance.selected_axes.z
  admittance.stiffness.rx
  admittance.stiffness.ry
  admittance.stiffness.rz
  admittance.stiffness.x
  admittance.stiffness.y
  admittance.stiffness.z
  chainable_command_interfaces
  command_interfaces
  control.frame_external
  control.frame_id
  control.open_loop_control
  enable_parameter_update_without_reactivation
  fixed_world_frame.external
  fixed_world_frame.id
  ft_sensor.frame_external
  ft_sensor.frame_id
  ft_sensor.name
  gravity_compensation.CoG.force
  gravity_compensation.CoG.x
  gravity_compensation.CoG.y
  gravity_compensation.CoG.z
  gravity_compensation.external
  gravity_compensation.frame_id
  joint_limiter_type
  joints
  kinematics.base
  kinematics.group_name
  kinematics.plugin_name
  kinematics.tip
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  state_interfaces
  state_publish_rate
  use_sim_time
  ```
  
  All parametter are automatically loaded and callbacks are setup by default. You can set a parameter by typing:
  
  `ros2 param set /minimal_publisher joints ["new value"]`
  
  You should see:
  
  `[INFO] [1656019001.515820371] [minimal_publisher]: Joint 0 is: 'new value'`
  
  Congratulations, you updated the parameter!
  
## Sample output
A sample generated file is located here: https://github.com/pac48/gen_param_struct/blob/main/example/include/parameters/admittance_controller.h
