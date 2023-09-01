# Readme
- This package is a ROS2 driver for schunk egh 80 gripper. It provides a service to send commands to the gripper via modbus TCP communication protocol.
- The service can be used to send control commands to the 'control register' and receive status data from the 'status register'.
- The package also publishes joint states of the gripper on *[schunk/schunk_egh80/joint_states]*, this topic can be customized through the launch file.
- The package generates the used services headers internaly. All services files are in 'srv/' directory.
  
## Dependencies
1. std_msgs
2. sensor_msgs 
3. modbuspp *[header file provided with the package: Modbus TCP c++ library]*

## Services
1. schunk/control/schunk_egh80 (*schunk_control_egh80::srv::ControlEGH80*)
    
    This service provide multiple commands as follows:
    - **connect**:      connects to the gripper modbus TCP server. (ip_address and device_port are obtained from a yaml file)
    - **set_position**: sets the opening position of the gripper. (input: percentage openning)
      - parameters:
        1. position: [float32] opening value (0-100%) of the gripper. 
    - **acknowledge**:  acknowledge error.
    - **get_position**: obtains the current opening position (in percentage).
    - **get_status**:   obtains the current status of the gripper.
    - **stop_gripper**: stop the gripper.
    - **reference**:    define the zero position of the gripper.
    - **disconnect**:   disconnect the gripper modbus TCP server.

## Topics
1. joint_states *[sensor_msgs/msg/JointState]*
   - schunk/schunk_egh80/joint_states
2. status *[std_msgs/msg/UInt8]*
   - schunk/schunk_egh80/status    

## Launch
To launch this node, use the launch.py script
```
ros2 launch schunk_control_egh80 egh80_driver_launch.py
```

## ToDo:
1. provide a common yaml file in a configuration package that manages all the ros parameters and default values instead of declaring them in the launch file or in the executable.