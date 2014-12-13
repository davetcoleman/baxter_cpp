Baxter_cpp
======

A C++ version of the Baxter SDK that works along side the Rethink SDK. Is inteded to be used with a ros_control implementation of the Baxter controllers that is run *on* the robot (via SSH).

Contains pick and place MoveIt! code for Baxter, as well as some other tools.

On going development continues in the ``development`` branch and contributors are strongly encouraged to send pull requests and use this code. Seriously, please help me improve this code. Attempts are made to keep the ``indigo-devel`` branch stable, but at the end of the day I am an open source graduate student contributor ;-)

<img align="right" src="https://raw.github.com/davetcoleman/baxter_cpp/indigo-devel/baxter_pick_place/resource/BaxterPickPlace.png" />

### Features

 * Actuated fingers using a custom robot state publisher and URDF
 * Baxter ros_control integration on Baxter's internal PC using [baxter_ssh](http://github.com/davetcoleman/baxter_ssh)
 * Baxter pick and place with MoveIt!
   * Generate grasps for blocks on a table
   * Execute a pick and place routine
   * Works on hardware and in an Rviz visualization
   * Other tools for testing trajectories
 * Integrated Asus Xtion Pro depth sensor (Kinect sensor)
   * Displays in MoveIt!

## Prerequisites

 * A Baxter with dual parallel electric grippers with SDK v1.0.0 installed
 * (Optional) Asus Xtion Pro Camera
 * [ROS Indigo](http://wiki.ros.org/ROS/Installation) on Ubuntu 14.04

## Baxter Installation

* Install wstool package

    ```
    sudo apt-get install python-wstool
    ```

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
    wstool merge --merge-replace -y https://raw.github.com/davetcoleman/baxter_cpp/indigo-devel/baxter_cpp/baxter.rosinstall
    ```

* Install ros_control and other low-level components internall on Baxter using the new SSH access:

    Follow instructions on [README of baxter_ssh](http://github.com/davetcoleman/baxter_ssh) repo

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

* Add Baxter setup.bash to your .bashrc (recommended)

    ```
    echo 'source ~/ros/ws_baxter/devel/setup.bash' >> ~/.bashrc
    ```

## Customize for your robot

Every Baxter is factory calibrated for the mouting points of the arms because they are welded on. Therefore, you might want to customize the ``baxter_description/urdf/baxter.urdf`` file to your robot's custom values. To do so:

* Start up your Baxter without launching any ROS nodes on your dev machine
* While connected to Baxter, run the command:
    ```
    rosparam get -p /robot_description | tail -n +2 > my.baxter.urdf
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

 * Ensure you have the correct ROS_MASTER_URI exported, this depends on your robot serial number. CU Boulder's is:
   ```
   export ROS_MASTER_URI=http://011305P0009.local:11311
   ```

   You might also need to set the ROS hostname environment variable if you have not already done so and you have communication issues:
   ```
   export ROS_HOSTNAME=128.138.244.72  # REPLACE WITH YOUR COMPUTER'S IP ADDRESS
   ```

 * Bringup ros_control controllers on Baxter

   Follow instructions on [baxter_ssh](http://github.com/davetcoleman/baxter_ssh)

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

## Gazebo Simulation

This is still in beta. First install the baxter_gazebo stuff. I haven't documented this yet.

   roslaunch baxter_gazebo baxter_world.launch

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

    roslaunch baxter_pick_place block_pick_place.launch

## Hardware Control Modes

Follow instructions on [README of baxter_ssh](http://github.com/davetcoleman/baxter_ssh) repo
	
## Test Programs

Send Baxter to random poses using motion planning and obstacle avoidance of a hard-coded planning scene

```
roslaunch baxter_pick_place random_planning.launch
```

## Programmed Buttons

*Duplicated from [README of baxter_ssh](http://github.com/davetcoleman/baxter_ssh)*

### End Effector Cuff

 * Rectangular button: close end effector
 * Circular button: open end effector

### Back of Robot Shoulders

 * Left shoulder button: enable Baxter
 * Right shoulder button: disable Baxter


## License

BSD (New BSD License)

## Contributors

- @davetcoleman
- @brawner
- @jon-weisz

Please help - see [Contribute](https://github.com/davetcoleman/baxter_cpp/blob/master/CONTRIBUTING.md) page.
