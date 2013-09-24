baxter
======

Unofficial Baxter packages that add-on to the Rethink SDK. It is entirely written in C++ and currently contains Gazebo simulation and pick and place MoveIt code for Baxter. 

The groovy-devel branch is no longer being actively developed but on going development continues in the hydro-devel branch by [Dave Coleman](http://davetcoleman.com). 

![alt tag](https://raw.github.com/davetcoleman/baxter/hydro-devel/baxter_pick_place/resource/BaxterPickPlace.png)

**Note:** This is the ROS Hydro readme version. See groovy-devel branch for ROS Groovy instructions.

[![Build Status](https://travis-ci.org/davetcoleman/baxter.png)](https://travis-ci.org/davetcoleman/baxter)

## Prerequisites

 * A Baxter with dual parallel grippers, or the desire to see one in simulation
 * [ROS Hydro](http://wiki.ros.org/ROS/Installation) on (suggested) Ubuntu 12.04
 * Access to the private Rethink [sdk-examples](https://github.com/RethinkRobotics/sdk-examples) repository - we are using the baxter_interface and head_control packages from the SDK. Contact [Dave](davetcoleman@gmail.com) if you should have access to this.
 * Setup Github - the git@github.com urls, below, only work if you have [Setup Github](https://help.github.com/articles/set-up-git) and generated [SSH Keys for Github](https://help.github.com/articles/generating-ssh-keys).
 * Install wstool package
    ```
    sudo apt-get install python-wstool
    ```

## Install MoveIt From Source

**Note:** Currently you can skip this section - the debians are still the same on version 0.5.1 as of Sep 9, 2013. 

We have chosen to freeze the version of MoveIt! we are using for the short-run to ensure compatibilty with our customizations. We recommend you install this in a seperate workspace (following the following insructions) to decrease built times.

* Setup workspace, download repositories and build

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
    
    **Note:** If you run into the error ``moveit_ros_perception: Cannot locate rosdep definition for [pcl]`` a temporary fix is to manually install PCL via ``sudo apt-get install ros-hydro-pcl``


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

* Also install from source the private RethinkRobotics SDK:

    ```
    git clone git@github.com:RethinkRobotics/sdk-examples.git -b gazebo_dev
    ```

    **Note:** CU Boulder users can gain access to the SDK Examples by cloning this repository:

    ```
    git clone git@github.com:correlllab/baxter_sdk -b gazebo_dev
    ```

* Disable duplicate packages

    There is currently a duplication of packages in sdk-examples and baxter_common that must be fixed manually. This issue should be fixed in Rethink's next release of their SDK:

    ```
    cd ~/ros/ws_baxter/
    touch src/sdk-examples/baxter_description/CATKIN_IGNORE
    touch src/sdk-examples/baxter_msgs/CATKIN_IGNORE
    ```

* Install dependencies and build

    ```
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

 * Ensure you have the correct ROS_MASTER_URI exported, this depends on your robot serial number:
   ```
   export ROS_MASTER_URI=http://011305P0009.local:11311
   ```

 * Turn on baxter


 * Optional: Enable robot 
   Most demos in this repo automatically will enable baxter
   ```
   rosrun tools enable_robot.py -e
   ```

### Simulation 

 * Ensure you have the correct ROS_MASTER_URI exported:
   ```
   export ROS_MASTER_URI=http://localhost:11311
   ```

 * Start simulation with controllers:
   ```
   roslaunch baxter_gazebo baxter_world.launch
   ```
   By default the position controllers are started. To switch, use the JointCommandMode topic as documented in the Baxter SDK.

 * Optional: Test/tune the velocity controllers or position controllers using a RQT dashboard GUI. Make sure you are in the right joint command mode when using these:

   ```
   roslaunch baxter_control baxter_sdk_position_rqt.launch
   ```
   or
   ```
   roslaunch baxter_control baxter_sdk_velocity_rqt.launch 
   ```

## Start MoveIt

Works with simulation or hardware:

 * Bringup Baxter Hardware Interface:
   This contains the gripper action server, trajectory controller, head image publisher and sonar muter.

   ```
   roslaunch baxter_control baxter_bringup.launch
   ```

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

## Programmed Buttons

The rectangular and circular buttons on the end effector cuff have been programmed to close and open the end effector, respectively. 

## Helpful Aliases

When using Baxter, it is very productive to have command shortcuts for diagnosing baxter. These are the ones I use:

### Turn on and off

    alias be="rosrun tools enable_robot.py -e"
    alias bd="rosrun tools enable_robot.py -d"
    alias br="rosrun tools enable_robot.py -r"
    alias bs="rostopic echo -c /sdk/robot/state"
    alias bsu="rosrun baxter_control sonar_enable.py --enable=0"
    alias bsd="rosrun baxter_control sonar_enable.py --enable=1"


### Launch scripts

    alias btc="rosrun baxter_interface trajectory_controller.py"
    alias bbu="roslaunch baxter_control baxter_bringup.launch"
    alias bm="roslaunch baxter_moveit_config baxter_moveit.launch"
    alias bpp="roslaunch baxter_pick_place block_pick_place.launch"
    alias bppa="roslaunch baxter_pick_place block_pick_place_all.launch"
    alias bw="rosrun joint_velocity wobbler.py"

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

### Misc

    alias bssh="ssh osrf@011305P0009.local"
    alias blog="ftp 011305P0009.local"


## License

BSD (New BSD License)

## Develop and Contribute

Please do! See [Contribute](https://github.com/osrf/baxter/blob/master/CONTRIBUTING.md) page.

