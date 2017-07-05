[![Build Status](https://travis-ci.org/AUV-IITK/auv.svg?branch=master)](https://travis-ci.org/AUV-IITK/auv)

Repository Organization:
```
├── debug_layer
│   ├── calibrate
│   │   ├── cfg
│   │   └── src
│   ├── remote_control
│   │   └── scripts
│   ├── varun_description
│   │   └── meshes
│   └── varun_gazebo
│       ├── launch
│       ├── models
│       │   ├── buoy_green
│       │   │   ├── meshes
│       │   │   └── urdf
│       │   ├── buoy_red
│       │   │   ├── meshes
│       │   │   └── urdf
│       │   ├── buoy_yellow
│       │   │   ├── meshes
│       │   │   └── urdf
│       │   ├── gate
│       │   │   ├── meshes
│       │   │   └── urdf
│       │   ├── light
│       │   │   ├── light_negative_x
│       │   │   ├── light_negative_y
│       │   │   ├── light_negative_z
│       │   │   ├── light_x
│       │   │   ├── light_y
│       │   │   └── light_z
│       │   ├── line
│       │   │   ├── meshes
│       │   │   └── urdf
│       │   ├── marker_target_o
│       │   │   ├── meshes
│       │   │   └── urdf
│       │   ├── marker_target_x
│       │   │   ├── meshes
│       │   │   └── urdf
│       │   ├── mittal
│       │   │   ├── meshes
│       │   │   └── urdf
│       │   ├── octagon
│       │   │   ├── meshes
│       │   │   └── sdf
│       │   ├── octagon_signal
│       │   │   ├── meshes
│       │   │   └── urdf
│       │   ├── perimeter
│       │   │   └── sdf
│       │   └── torpedo_target
│       │       ├── meshes
│       │       │   └── torpedo_target.dae
│       │       └── urdf
│       │           └── torpedo_target.urdf
│       ├── scripts
│       ├── src
│       └── worlds
├── hardware_layer
│   ├── hardware_arduino
│   │   ├── launch
│   │   ├── scripts
│   │   └── src
│   │       └── testing_arduino_node.cpp
│   ├── hardware_camera
│   │   ├── cfg
│   │   └── src
│   └── hardware_imu
│       └── src
├── master_layer
│   └── the_master
│       ├── cfg
│       ├── launch
│       └── src
├── motion_library_layer
│   ├── motion_commons
│   │   ├── action
│   │   └── launch
│   ├── motion_forward
│   │   ├── cfg
│   │   └── src
│   ├── motion_sideward
│   │   ├── cfg
│   │   └── src
│   ├── motion_turn
│   │   ├── cfg
│   │   │   └── turning.cfg
│   │   └── src
│   │       ├── turningXY.cpp
│   │       └── turningXYTest.cpp
│   └── motion_upward
│       ├── cfg
│       └── src
├── task_handler_layer
│   ├── task_buoy
│   │   ├── cfg
│   │   ├── launch
│   │   │   ├── task_buoy_gazebo.launch
│   │   │   └── task_buoy_varun.launch
│   │   └── src
│   ├── task_commons
│   │   ├── action
│   │   └── launch
│   │       ├── task_nodes_gazebo.launch
│   │       └── task_nodes_varun.launch
│   ├── task_gate
│   │   ├── cfg
│   │   ├── launch
│   │   │   ├── task_gate_gazebo.launch
│   │   │   └── task_gate_varun.launch
│   │   └── src
│   ├── task_line
│   │   ├── cfg
│   │   ├── launch
│   │   │   ├── task_line_gazebo.launch
│   │   │   └── task_line_varun.launch
│   │   └── src
│   └── task_octagon
│       ├── cfg
│       ├── launch
│       │   ├── task_octagon_gazebo.launch
│       │   └── task_octagon_varun.launch
│       └── src
└── utils

```
