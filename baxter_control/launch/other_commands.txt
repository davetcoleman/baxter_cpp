rosservice call /robot/controller_manager/load_controller "name: 'position_joint_mode_controller'"
rosservice call /robot/controller_manager/load_controller "name: 'left_position_trajectory_controller'"
rosservice call /robot/controller_manager/load_controller "name: 'right_position_trajectory_controller'"

% switch to position mode
rosservice call /robot/controller_manager/switch_controller "{start_controllers: ['position_joint_mode_controller','left_position_trajectory_controller','right_position_trajectory_controller'], stop_controllers: ['velocity_joint_mode_controller','left_velocity_trajectory_controller','right_velocity_trajectory_controller'], strictness: 2}"

% switch to velocity mode
rosservice call /robot/controller_manager/switch_controller "{start_controllers: ['velocity_joint_mode_controller','left_velocity_trajectory_controller','right_velocity_trajectory_controller'], stop_controllers: ['position_joint_mode_controller','left_position_trajectory_controller','right_position_trajectory_controller'], strictness: 2}"
