`ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e`
`ros2 launch ur_simulation_gz ur_sim_control.launch.py   ur_type:=ur5e   description_file:=/manipulator_simulator/src/ur5e_rgbd/urdf/ur5e_rgbd.xacro`
`ros2 launch ur5e_simulator sim_env.launch.py `
### Quick terminal waypoint publisher
```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names:
- shoulder_pan_joint
- shoulder_lift_joint
- elbow_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint
points:
- positions: [0.0, -1.2, 1.4, -1.4, -1.5, 0.0]
  time_from_start: {sec: 2}
"
```

