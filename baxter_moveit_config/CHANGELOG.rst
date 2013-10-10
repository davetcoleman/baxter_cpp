^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package baxter_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2013-10-09)
------------------
* Added ability for cuff squeeze to update trajectory controller
* Increased controller wait time
* New debug options, reduced error msgs, tweaked environment
* Updated random planner
* Tweaks and updates
* Removed anon because failes if computer has no name
* Created new simulation method with ros_control
* Updated trajectory tolerances

0.2.0 (2013-09-30)
------------------
* Tweaked trajectory controllers
* Fixed hardware performance
* Tweaked collision geometries
* Cleaned up Gazebo simulation stuff
* Created scripts package
* Updated Travis
* Finished left and right arm position-based trajectory controllers using ros_control
* Expanded the hardware interface into seperate arm classes
* Joint trajectory controller initially working on hardware
* Fixes for latest moveit version
* Updated collision bodies
* Successfully tuned PIDs for ros_control joint_trajectory_controller. Working on collision issue
* Setup to work with ros_controllers trajectory controller
* Small tweaks

0.1.0 (2013-09-19)
------------------
* Normalized version numbers to 0.0.0
* Final tweaks
* Preparing to merge development branch
* Refactored visualization_tools, added better support for baxter left arm
* Repressed warning message
* Verticle test is initially working
* Added open/close buttons to cuff of baxter
* Created new environment, disabled left arm, tweaked gripper server
* Added rqt trajectory gui
* Made baxter_utilities into library, cleaned up enable sonar script
* Refactored launch files
* Refactored packages
* Renamed simple_pick_place
* Merged
* Tweaks to gripper server
* Refactored baxter_pick_place
* Working on pick and place
* Updated SRDF
* Before Setup Assistant run
* Created new node for getting joint values of any planning group with moveit
* Hard coded lab environment into pick place
* Added debug, small simulation plugin fix
* Migrated to latest MoveIt changes
* Finished controller switching
* Added lots of error checking
* Small fixes
* Tweaks to pick and place
* Renamed utility class, tweaks, end of day
* Added second end effector and modeled a wall
* Automated turning on and off baxter
* Gripper action server and pick place improvements
* Pick and place progress
* Broken ikfast plugins, fixes for real hardware
* Fixes for simulated controllers
* Baxter + MoveIt in Gazebo initialy working
* Baxter trajectory control work
* Tuned PID values, added joint state publisher and robot state publisher
* Fixed gitignore issue
* Added joint_state_controller, robot_state_publisher, Rviz support
* Controller updates
* Basic random demo done
* Possible demo version
* Tweaks to pick place
* Tweaks to pick and place
* Created new baxter gripper controller
* Updated SRDF
* Creaing new mimic node
* Reduced right end effector
* Removed SDK packages
