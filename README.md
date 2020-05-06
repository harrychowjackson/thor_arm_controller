Thor_arm_controller

This is a ROS package to control the Thor robotic arm (https://hackaday.io/project/12989-thor).

Both nodes convert a set of joint states into G-code commands that the node will send via serial port to an arduino running a modified version of GRBL. 

thor_arm_controller will listen to the /move_group/fake_controller_joint_states topic from MoveIt, and mimic the position of the simulated arm. This should be run in conjunction with Moveit. 

thor_arm_controller_joystick will listen to an XBOX 360 controller for realtime control, and also save/load up to four waypoints. This can be easily launched with roslaunch thor_arm_controller thor_arm_controller_joystick.launch
