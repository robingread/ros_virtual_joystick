# ros_virtual_joystick

All the `sensor_msgs/Joy` data is published on the `/joy` topic.

## Launching the standalone node

A standalone node can be launched by running:

```bash
ros2 launch ros_virtual_joystick joystick_node.launch.py
```

## RQt & RViz Plugins

It is also possible to load the tool in both `rqt_gui` and `rviz2`, meaning that you can build it into existing, more complex GUI setups that utilize more of the ROS ecosystem while reducing the number of windows that you need to have open.

To load into `rqt_gui`, simply open it via the `Plugins/Teleoperation` menu.

To load the panel into `rviz2` select it through the `Panels/Add New Panel` dialogue box.
