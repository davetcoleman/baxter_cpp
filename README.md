baxter
======

Unofficial Baxter packages that add-on to the Rethink SDK. Currently it mostly contains Gazebo interface stuff, though also some MoveIt code.

## Prequisites

 * ROS Groovy or Hydro
 * [gazebo_ros_pkgs](gazebosim.org/wiki/Tutorials#ROS_Integration) installed with the latest stand-alone version of Gazebo
 * A catkinized version of baxter_msgs (Rethink will officially release this soon to baxter_common, otherwise email davetcoleman@gmail.com)

## Installation

* Create a catkin workspace and cd into it:

```
    cd ~/catkin_ws/src
```

* Checkout this repo

```
    git clone git@github.com:davetcoleman/baxter.git
```

* Also install from source a transmission version of Baxter, moveit_plugins and (optional) some grasping code

```
    git clone git@github.com:davetcoleman/baxter_common.git -b development
    git clone git@github.com:ros-planning/moveit_plugins.git
    git clone git@github.com:davetcoleman/block_grasp_generator.git
    git clone git@github.com:davetcoleman/reflexxes_controllers.git -b action_server
```

* Install dependencies

Groovy:
```
    rosdep install --from-paths . --ignore-src --rosdistro groovy -y
```

Hydro:
```
    rosdep install --from-paths . --ignore-src --rosdistro hydro -y
```

* Build

```
    cd ..
    catkin_make
```

## Run

### Launch Only Baxter in Gazebo:

```
roslaunch baxter_gazebo baxter_world.launch
```

### Launch Baxter in Gazebo with Rethink SDK
Loads position controllers that accept baxter_msgs/JointPositions.msg

```
roslaunch baxter_gazebo baxter_world.launch sdk:=true
```

Test the controllers using RQT to see a "dashboard" for controlling Baxter:

```
roslaunch baxter_control baxter_individual_rqt.launch 
```

This will provide you with easy ways to publish sine wave commands to the actuators, tune the PID controllers and visualize the performance.

## Other Run Commands

### Launch Individual Generic Simulated controllers for Baxter:
Only accepts individual std_msgs/Float32 commands

```
roslaunch baxter_control baxter_individual_control.launch 
```

### Launch RQT 
to see a "dashboard" for controlling Baxter:

```
roslaunch baxter_control baxter_individual_rqt.launch 
```
This will provide you with easy ways to publish sine wave commands to the actuators, tune the PID controllers and visualize the performance.

### Run a Baxter gripper action server:
Note: requires you have a gripper modeled in the Baxter URDF. This version of the URDF is available in the [baxter_with_gripper](https://github.com/davetcoleman/baxter_common/commits/baxter_with_gripper) branch of davetcoleman/baxter_common

```
rosrun baxter_gripper_server gripper_action_server
```

### Launch a trajectory controller that runs a FollowJointTrajectoryAction (Experimental):

First, restart everything, then:

```
roslaunch baxter_gazebo baxter_world.launch trajectory:=true
roslaunch baxter_control baxter_trajectory_rqt.launch
```

## Develop and Contribute

See [Contribute](https://github.com/osrf/baxter/blob/master/CONTRIBUTING.md) page.
