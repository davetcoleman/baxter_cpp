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
    git clone git@github.com:davetcoleman/baxter_common.git
    git clone git@github.com:ros-planning/moveit_plugins.git
    git clone git@github.com:davetcoleman/block_grasp_generator.git
```

* Install dependencies

Groovy:
```
    rosdep install --from-paths src --ignore-src --rosdistro groovy -y
```

Hydro:
```
    rosdep install --from-paths src --ignore-src --rosdistro hydro -y
```

* Build

```
    cd ..
    catkin_make
```

## Run

### Launch Baxter in Gazebo:

```
roslaunch baxter_gazebo baxter_world.launch
```

### Launch Baxter' simulated controllers: 
Note: currently only accepts individual std_msgs/Float32 commands

```
roslaunch baxter_control baxter_control.launch 
```

### Launch RQT 
to see a "dashboard" for controlling Baxter:

```
roslaunch baxter_control baxter_rqt_control.launch 
```

### Run a Baxter gripper action server:
Note: requires you have a gripper modeled in the Baxter URDF. This version of the URDF is available in the [baxter_with_gripper](https://github.com/davetcoleman/baxter_common/commits/baxter_with_gripper) branch of davetcoleman/baxter_common

```
rosrun baxter_gripper_server gripper_action_server
```

## Run Experimental 
aka not working

### Launch a trajectory controller that runs a FollowJointTrajectoryAction:

```
roslaunch baxter_control baxter_reflexxes_control.launch
```



## Develop and Contribute

See [Contribute](https://github.com/osrf/baxter/blob/master/CONTRIBUTING.md) page.
