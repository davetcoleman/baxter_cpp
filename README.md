baxter
======

Unofficial Baxter packages that add-on to the Rethink SDK. Currently it contains Gazebo simulation and pick and place MoveIt code for Baxter

## Prequisites

 * ROS Groovy or Hydro
 * [gazebo_ros_pkgs](gazebosim.org/wiki/Tutorials#ROS_Integration) installed with the latest stand-alone version of Gazebo
 * The gazebo_dev version of [sdk-examples](https://github.com/RethinkRobotics/sdk-examples) - we are using the baxter_interface and head_control packages from the SDK

## Installation

* Create a catkin workspace and cd into it:

```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
```

* Checkout this repo

```
    git clone git@github.com:davetcoleman/baxter.git
```

* Also install from source a few customized repositories:

```
    git clone git@github.com:davetcoleman/baxter_common.git -b baxter_with_gripper
    git clone git@github.com:davetcoleman/block_grasp_generator.git
    git clone git@github.com:ros-controls/ros_controllers -b velocity_position_controller
```

* Install dependencies

Groovy:
```
    cd ~/catkin_ws/
    rosdep install --from-paths . --ignore-src --rosdistro groovy -y
```

Hydro:
```
    cd ~/catkin_ws/
    rosdep install --from-paths . --ignore-src --rosdistro hydro -y
```

* Build

```
    catkin_make
```

## Bringup Baxter in Simulation or Harware

### Hardware

 * Turn on baxter
 * Enable robot:

    ```
    rosrun tools enable_robot.py -e
    ```
 * Temporary: launch gripper server

    ```
    rosrun baxter_gripper_server gripper_action_server
    ```

### Simulation

Without controllers:

```
roslaunch baxter_gazebo baxter_world.launch
```

#### Velocity Controllers

With velocity controllers that accept baxter_msgs/JointVelocities.msg

 * Start simulation with controllers:
   ```
   roslaunch baxter_gazebo baxter_world.launch velocity:=true
   ```

 * Temporary: launch gripper server
    ```
    rosrun baxter_gripper_server gripper_action_server
    ```

Test the controllers using RQT to see a "dashboard" for controlling Baxter:

```
roslaunch baxter_control baxter_sdk_velocity_rqt.launch 
```

#### Position Controllers

With position controllers that accept baxter_msgs/JointPositions.msg

```
roslaunch baxter_gazebo baxter_world.launch position:=true
```

Test the controllers using RQT to see a "dashboard" for controlling Baxter:

```
roslaunch baxter_control baxter_sdk_position_rqt.launch 
```

## Start MoveIt

Works with simulation or hardware:

```
     roslaunch baxter_moveit_config baxter_bringup.launch
```

## Pick and place demo

```
     roslaunch baxter_pick_place baxter_pick_place.launch
```


## Other Run Commands

### Launch Individual Generic Simulated controllers for Baxter:

Only accepts individual std_msgs/Float32 commands

First, restart everything, then:

```
roslaunch baxter_gazebo baxter_world.launch individual:=true
roslaunch baxter_control baxter_individual_rqt.launch 
```

### Launch a trajectory controller that runs a FollowJointTrajectoryAction (Very Experimental):

First, restart everything, then:

```
roslaunch baxter_gazebo baxter_world.launch trajectory:=true
roslaunch baxter_control baxter_trajectory_rqt.launch
```

## Develop and Contribute

See [Contribute](https://github.com/osrf/baxter/blob/master/CONTRIBUTING.md) page.
