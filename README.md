baxter_cpp
======

A C++ version of the Baxter SDK that works along side the Rethink SDK. Currently contains a ros_control implementation of the Baxter controllers and pick and place MoveIt! code for Baxter, as well as other tools. 

On going development continues in the development branch and contributors are strongly encouraged to send pull requests and use this code. The master branch of this repository is kept as stable as posible and is continuously integrated using [Travis](https://travis-ci.org/).

<img align="right" src="https://raw.github.com/davetcoleman/baxter_cpp/indigo-devel/baxter_pick_place/resource/BaxterPickPlace.png" />

### Features

 * Baxter pick and place with MoveIt!
   * Generate grasps for simple blocks on a table
   * Execute a pick and place routine
   * Works on hardware and in an Rviz visualization
   * Other tools for testing trajectories
 * Baxter ros_control position, velocity, or torque trajectory controllers
   * Uses the ros_control [joint_trajectory_controller](https://github.com/ros-controls/ros_controllers/tree/hydro-devel/joint_trajectory_controller) instead of the python trajectory controller that comes with the SDK
 * Integrated Asus Xtion Pro depth sensor (Kinect sensor)
   * Displays in MoveIt!
 * Other stuff
   * Baxter's face follows obstacles using the sonars

### Build Status

[![Build Status](https://travis-ci.org/davetcoleman/baxter_cpp.png?branch=indigo-devel)](https://travis-ci.org/davetcoleman/baxter_cpp)

# Branches

 * [indigo-devel](https://github.com/davetcoleman/baxter_cpp/tree/indigo-devel) - for current Baxter 1.0.0 SDK software on ROS Indigo. 
 * [hydro-devel](https://github.com/davetcoleman/baxter_cpp/tree/hydro-devel) - for Baxter 0.7.0 SDK software on ROS Hydro.
 * [development](https://github.com/davetcoleman/baxter_cpp/tree/development) - latest indigo-devel work is commited here. Unstable.
 * [groovy-devel-sdk0.6.2](https://github.com/davetcoleman/baxter_cpp/tree/groovy-devel-sdk0.6.2) - for Baxter 0.6.2 SDK software on ROS Groovy. *NOT SUPPORTED*
 * [hydro-devel-sdk0.6.2](https://github.com/davetcoleman/baxter_cpp/tree/hydro-devel-sdk0.6.2) - for Baxter 0.6.2 SDK software on ROS Hydro. *NOT SUPPORTED*

## Prerequisites

 * A Baxter with dual parallel electric grippers with SDK v1.0.0 installed
 * (Optional) Asus Xtion Pro Camera
 * [ROS Indigo](http://wiki.ros.org/ROS/Installation) on (suggested) Ubuntu 14.04
 * Install wstool package
    ```
    sudo apt-get install python-wstool
    ```
    
## Baxter Installation

* Create a catkin workspace if you don't already have one (we recommend a separate one for Baxter) 

    ```
    mkdir -p ~/ros/ws_baxter/src
    cd ~/ros/ws_baxter/src
    wstool init .
    ```

* Install Rethink's Baxter SDK as documented below, if you have not already.
    *Note*: These instructions can also be found at [Installing the Research SDK](https://github.com/RethinkRobotics/sdk-docs/wiki/Installing-the-Research-SDK)

    ```
    wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
    ```

* Install these baxter_cpp packages:
    *Note*: replaces Rethink's baxter_common with a version that has parallel electric end effectors modeled

    ```
    wstool merge --merge-replace -y https://raw.github.com/davetcoleman/baxter_cpp/indigo-devel/baxter.rosinstall
    ```

* Optional: Only if you have access to the CU Boulder's private research code (you probably don't):

    ```
    wstool merge https://raw.githubusercontent.com/davetcoleman/baxter_cpp/indigo-devel/baxter_experimental.rosinstall
    ```

* Download the Baxter packages:

    ```	    
    wstool update
    ```

* Setup ROS if you haven't already (you can probably skip this):
    ```
    sudo apt-get update 
    sudo apt-get dist-upgrade
    source /opt/ros/indigo/setup.bash
    rosdep update
    ```

* Install dependencies and build

    ```
    cd ..
    rosdep install --from-paths . --ignore-src --rosdistro indigo -y
    catkin_make
    ```

    Note: You may need to run this command multiple times if there is a message dependency issue. Please report these bugs in the issue tracker.

* Add Baxter setup.bash to your .bashrc (recommended)

    ```
    echo 'source ~/ros/ws_baxter/devel/setup.bash' >> ~/.bashrc
    ```

## Customize for your robot

Every Baxter is factory calibrated for the mouting points of the arms because they are welded on. Therefore, you might want to customize the ``baxter_description/urdf/baxter.urdf`` file to your robot's custom values. To do so:

* Start up your Baxter without launching any ROS nodes on your dev machine
* While connected to Baxter, run the command:
    ```
    rosparam dump my.baxter.urdf /robot_description
    ```   
* Within ``my.baxter.urdf`` find the lines that say:
    ```   
    <joint name="left_torso_arm_mount" type="fixed"> 
    ```   
    and 
    ```   
    <joint name="right_torso_arm_mount" type="fixed"> 
    ```   
* Copy the following ``<origin>`` line to the corresponding location in ``baxter.urdf`` located in ``baxter_description/urdf/baxter.urdf``


## Bringup Baxter

### Hardware

 * Power on baxter

 * Ensure you have the correct ROS_MASTER_URI exported, this depends on your robot serial number. Mine is:
   ```
   export ROS_MASTER_URI=http://011305P0009.local:11311
   ```

   You might also need to set the ROS hostname environment variable if you have not already done so and you have communication issues:
   ```
   export ROS_HOSTNAME=128.138.244.72  # REPLACE WITH YOUR COMPUTER'S IP ADDRESS
   ```

 * Bringup ros_control controllers - starts a position-based trajectory controller. See [Hardware Control Modes](#hardware-control-modes) for other control modes
   ```
   roslaunch baxter_control baxter_hardware.launch
   ```

### Rviz Visualization

This only shows a virtual Baxter in [Rviz](http://www.ros.org/wiki/rviz) without any physics simulator. Instead it uses a ros_control hardware interface that simply loops back to itself. Good for testing MoveIt!.

 * Ensure you have the correct ROS_MASTER_URI exported:
   ```
   export ROS_MASTER_URI=http://localhost:11311
   ```

 * Start visualization:
   ```
   roslaunch baxter_control baxter_visualization.launch
   ```
   **Note:** you will not see anything when you launch this, only the ``/robot/joint_states`` topic starts being published. Proceed to the MoveIt! section, below.

## Start MoveIt!

Works with simulation, hardware or visualization:

 * Start MoveIt!'s move_group Server:

   ```
   roslaunch baxter_moveit_config baxter_moveit.launch
   ```

 * Start Rviz with MoveIt! configured:

   ```
   roslaunch baxter_moveit_config moveit_rviz.launch
   ```

## Block pick and place demo

Picks small blocks located on a table in front of Baxter and places them to Baxter's left. Assumes perfect perception (doesn't have perception) as defined in custom_environment.h.

```
roslaunch baxter_pick_place block_pick_place.launch
```

## Hardware Control Modes

This Baxter repository uses [ros_control](http://wiki.ros.org/ros_control) to send trajectories to Baxter via the joint_trajectory_controller. Trajectories can be executed on Baxter in either position mode or velocity mode. You can easily switch between the two - both are loaded at startup but position is started by default:

 * Position Control

   Load the position controllers (not loaded by default)
   ```
   rosrun controller_manager spawner --stopped position_joint_mode_controller left_position_trajectory_controller right_position_trajectory_controller --namespace /robot &
   ```
   Start the position controllers and stop the velocity controllers
   ```
   rosservice call /robot/controller_manager/switch_controller "{start_controllers: ['position_joint_mode_controller','left_position_trajectory_controller','right_position_trajectory_controller'], stop_controllers: ['velocity_joint_mode_controller','left_velocity_trajectory_controller','right_velocity_trajectory_controller'], strictness: 2}"
   ```
   Plot position error of position-based trajectory controller 
   ```
   roslaunch baxter_control joint_position_left_trajectory_controller.launch
   roslaunch baxter_control joint_position_right_trajectory_controller.launch
   ```

 * Velocity Control
   ```
   rosservice call /robot/controller_manager/switch_controller "{start_controllers: ['velocity_joint_mode_controller','left_velocity_trajectory_controller','right_velocity_trajectory_controller'], stop_controllers: ['position_joint_mode_controller','left_position_trajectory_controller','right_position_trajectory_controller'], strictness: 2}"
   ```
   Plot *position* error of velocity-based trajectory controller 
   ```
   roslaunch baxter_control joint_velocity_left_trajectory_controller.launch
   roslaunch baxter_control joint_velocity_right_trajectory_controller.launch
   ```

 * Torque Control

   TODO

## Test Programs

Send Baxter to random poses using motion planning and obstacle avoidance of a hard-coded planning scene

```
roslaunch baxter_pick_place random_planning.launch
```

## Programmed Buttons

### End Effector Cuff

 * Rectangular button: close end effector
 * Circular button: open end effector

### Back of Robot Shoulders

 * Left shoulder button: enable Baxter
 * Right shoulder button: disable Baxter

## Gazebo Simulation 

**Note:** since Baxter SDK 0.7.0 this has not been maintained

This uses an actual physics engine from the [Gazebo Simulator](http://gazebosim.org/).

 * Ensure you have the correct ROS_MASTER_URI exported:
   ```
   export ROS_MASTER_URI=http://localhost:11311
   ```

 * Start simulation with controllers:
   ```
   roslaunch baxter_gazebo baxter_gazebo.launch
   ```
   By default, an effort-based trajectory controller is started

## License

BSD (New BSD License)

## Develop and Contribute

Please do! See [Contribute](https://github.com/davetcoleman/baxter_cpp/blob/master/CONTRIBUTING.md) page.

