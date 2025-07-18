In this package:

1. We have designed pikachu arm using solidworks.
2. Then we used SW urdf plugin to emit corresponding meshes and urdf package.
3. We then copied the meshes and urdf files for the arm into this package.
4. joint2 is fully controlled.
5. joint5 is not yet wired.
6. Does not use moveit.
7. Uses different controllers to control the joint2.


Commands:
Close any arduino IDE first to release the serial port.
colcon build  --packages-select urdf_tutorial; source install/setup.bash; ros2 launch urdf_tutorial kp.display.launch.py;
ros2 launch urdf_tutorial test_forward_position_controller.launch.py;
# robot-arm-with-controller
# robot-arm-with-controller
