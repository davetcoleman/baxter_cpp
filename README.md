baxter
======

Unofficial Baxter packages that work along side the Rethink SDK, or optionally without it. It is entirely written in C++ and currently contains Gazebo simulation and pick and place MoveIt code for Baxter. On going development continues in the hydro-devel branch by [Dave Coleman](http://davetcoleman.com). 

<img align="right" src="https://raw.github.com/davetcoleman/baxter/hydro-devel/baxter_pick_place/resource/BaxterPickPlace.png" />

### Features

 * Baxter simulated in Gazebo
   * Simulated controllers using [ros_control](http://wiki.ros.org/ros_control)
   * Simulated head display
   * Not implemented yet: simulated cameras, sonars, or other sensors. Feel free to add!
 * Baxter pick and place with MoveIt
   * Generate grasps for simple blocks on a table
   * Execute a pick and place routine
   * Works on hardware, Gazebo and in an Rviz visualization
   * Other tools for testing trajectories
 * Baxter position trajectory controller
   * Uses the ros_control [joint_trajectory_controller](https://github.com/ros-controls/ros_controllers/tree/hydro-devel/joint_trajectory_controller) instead of the python trajectory controller that comes with the SDK

**Note:** This is the ROS Hydro version. See groovy-devel branch for ROS Groovy instructions, although that branch is no longer being actively developed.

### Build Status

[![Build Status](https://travis-ci.org/davetcoleman/baxter.png?branch=hydro-devel)](https://travis-ci.org/davetcoleman/baxter)

## Prerequisites

 * A Baxter with dual parallel electric grippers, or the desire to see one visualized/simulated
 * [ROS Hydro](http://wiki.ros.org/ROS/Installation) on (suggested) Ubuntu 12.04
 * Install wstool package
    ```
    sudo apt-get install python-wstool
    ```

## Install MoveIt From Source

You should install MoveIt from source if:

 * You want to be guaranteed compatibility and the current public moveit_core debian release is not 0.5.4 (check [here](http://ros.org/debbuild/hydro.html) and search for moveit_core, then hover your mouse over the third column under HbinP64. Or check [here](https://github.com/ros-planning/moveit_core/releases))
 * You want to install MoveIt from source for development reasons but also want to ensure your source checkout is compatible with this repo

Otherwise you can just skip this section and install the Baxter code and it will automatically pull the necessary debians for MoveIt

* Setup workspace, download repositories and build

    We recommend you install this in a seperate workspace (following the following insructions) to decrease built times.

    ```
    mkdir -p ~/ros/ws_moveit/src
    cd ~/ros/ws_moveit/src
    wstool init .
    wstool merge https://raw.github.com/davetcoleman/baxter/hydro-devel/moveit.rosinstall
    wstool update
    cd ..
    rosdep install --from-paths src --ignore-src --rosdistro hydro -y
    catkin_make
    ```
    
* Add MoveIt setup.bash to your .bashrc (recommended)

    ```
    echo 'source ~/ros/ws_moveit/devel/setup.bash' >> ~/.bashrc
    ```

## Baxter Installation

* Create a catkin workspace (we recommend a separate one for Baxter) and use wstool to install the individual repositories

    ```
    mkdir -p ~/ros/ws_baxter/src
    cd ~/ros/ws_baxter/src
    wstool init .
    wstool merge https://raw.github.com/davetcoleman/baxter/hydro-devel/baxter.rosinstall
    wstool update
    ```

* **Optional:** install from source the private Rethink [sdk-examples](https://github.com/RethinkRobotics/sdk-examples) repository if you have access.

    ```
    git clone git@github.com:RethinkRobotics/sdk-examples.git -b gazebo_dev
    ```

    Note: CU Boulder users can gain access to the SDK Examples by cloning this repository. Contact [Dave](davetcoleman@gmail.com) with your Github user name if you should have access to this.

    ```
    git clone git@github.com:correlllab/sdk-examples -b gazebo_dev
    ```

* Install dependencies and build

    ```
    cd ..
    rosdep install --from-paths . --ignore-src --rosdistro hydro -y
    catkin_make
    ```

You may need to run this command multiple times if there is a message dependency issue.

* Add Baxter setup.bash to your .bashrc (recommended)

    ```
    echo 'source ~/ros/ws_baxter/devel/setup.bash' >> ~/.bashrc
    ```

## Bringup Baxter

### Hardware

 * Power on baxter

 * Ensure you have the correct ROS_MASTER_URI exported, this depends on your robot serial number:
   ```
   export ROS_MASTER_URI=http://011305P0009.local:11311
   ```

   You might also need to set the ROS hostname environment variable if you have not already done so and you have communication issues:
   ```
   export ROS_HOSTNAME=128.138.244.72  # REPLACE WITH YOUR COMPUTER'S IP ADDRESS
   ```

 * Bringup ros_control controllers - starts a position-based trajectory controller
   ```
   roslaunch baxter_control baxter_hardware.launch
   ```

### Gazebo Simulation 

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

### Rviz Visualization

This only shows a virtual Baxter in [Rviz](http://www.ros.org/wiki/rviz) without any physics simulator. Instead it uses a ros_control hardware interface that simply loops back to itself. Good for testing MoveIt.

 * Ensure you have the correct ROS_MASTER_URI exported:
   ```
   export ROS_MASTER_URI=http://localhost:11311
   ```

 * Start visualization:
   ```
   roslaunch baxter_control baxter_visualization.launch
   ```
   **Note:** you will not see anything when you launch this, only the ``/robot/joint_states`` topic starts being published. Proceed to the MoveIt section, below.

## Start MoveIt

Works with simulation, hardware or visualization:

 * Start MoveIt's move_group Server:

   ```
   roslaunch baxter_moveit_config baxter_moveit.launch
   ```

## Block pick and place demo

Picks small blocks located on a table in front of Baxter and places them to Baxter's left. Assumes perfect perception (doesn't have perception) as defined in custom_environment.h.

```
roslaunch baxter_pick_place block_pick_place.launch
```

## Test Programs

Send Baxter to random poses using motion planning and obstacle avoidance of a hard-coded planning scene

```
roslaunch baxter_pick_place random_planning.launch
```

Send the end effector up and down with a horizontal cartesian path

```
roslaunch baxter_pick_place verticle_approach_test.launch
```

Test grasp poses of blocks

```
roslaunch baxter_pick_place block_grasp_generator_test.launch
```

## Tune Trajectory Controllers

Starts an RQT plot of the position error of the two arms' joints.

```
roslaunch baxter_control joint_position_left_trajectory_controller.launch
roslaunch baxter_control joint_position_right_trajectory_controller.launch
```

## Programmed Buttons

The rectangular and circular buttons on the end effector cuff have been programmed to close and open the end effector, respectively. 

## Helpful Aliases

When using Baxter, it is productive to have command shortcuts for diagnosing baxter. These are the ones I use:

### Turn on and off

    alias be="rostopic pub -1 /robot/set_super_enable std_msgs/Bool True"
    alias bd="rostopic pub -1 /robot/set_super_enable std_msgs/Bool False"
    alias br="rostopic pub -1 /robot/set_super_reset std_msgs/Empty"
    alias bs="rostopic echo -c /sdk/robot/state"
    alias bsu="rosrun baxter_control sonar_enable.py --enable=0"
    alias bsd="rosrun baxter_control sonar_enable.py --enable=1"

### Launch scripts

    alias bhardware="roslaunch baxter_control baxter_hardware.launch"
    alias bm="roslaunch baxter_moveit_config baxter_moveit.launch"
    alias bpp="roslaunch baxter_pick_place block_pick_place.launch"

### Calibrate/Tare

    alias brtare="rosrun tools tare.py -t right"
    alias bltare="rosrun tools tare.py -t left"
    alias brcalibrate="rosrun tools calibrate_arm.py -c right"
    alias blcalibrate="rosrun tools calibrate_arm.py -c left"

### Gripper Control

    alias brgripperstate="rostopic echo -c /sdk/robot/limb/right/accessory/gripper/state"
    alias brgrippercal="rostopic pub -1 /robot/limb/right/accessory/gripper/command_calibrate std_msgs/Empty"
    alias brgripperres="rostopic pub -1 /robot/limb/right/accessory/gripper/command_reset std_msgs/Bool true"
    alias brgripperopen="rostopic pub -1 /robot/limb/right/accessory/gripper/command_release std_msgs/Empty"
    alias brgripperclose="rostopic pub -1 /robot/limb/right/accessory/gripper/command_grip std_msgs/Float32 0"
    alias blgripperstate="rostopic echo -c /sdk/robot/limb/left/accessory/gripper/state"
    alias blgrippercal="rostopic pub -1 /robot/limb/left/accessory/gripper/command_calibrate std_msgs/Empty"
    alias blgripperres="rostopic pub -1 /robot/limb/left/accessory/gripper/command_reset std_msgs/Bool true"
    alias blgripperopen="rostopic pub -1 /robot/limb/left/accessory/gripper/command_release std_msgs/Empty"
    alias blgripperclose="rostopic pub -1 /robot/limb/left/accessory/gripper/command_grip std_msgs/Float32 0"

### View Cameras

    alias brcamera="rosrun image_view image_view image:=/cameras/right_hand_camera/image"
    alias blcamera="rosrun image_view image_view image:=/cameras/left_hand_camera/image"

### Misc

    alias blog="ftp 011305P0009.local"


## License

BSD (New BSD License)

## Develop and Contribute

Please do! See [Contribute](https://github.com/osrf/baxter/blob/master/CONTRIBUTING.md) page.

