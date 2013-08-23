baxter
======

Unofficial Baxter packages that add-on to the Rethink SDK. Currently it contains Gazebo simulation and pick and place MoveIt code for Baxter

## Prequisites

 * ROS Groovy or Hydro
 * Access to the private Rethink [sdk-examples](https://github.com/RethinkRobotics/sdk-examples) repository - we are using the baxter_interface and head_control packages from the SDK

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
    git clone git@github.com:RethinkRobotics/sdk-examples.git -b gazebo_dev
    git clone git@github.com:davetcoleman/baxter_common.git -b dual_parallel_grippers
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

## Bringup Baxter

### Hardware

 * Turn on baxter
 * Enable robot:

    ```
    rosrun tools enable_robot.py -e
    ```

### Simulation 

 * Start simulation with controllers:
   ```
   roslaunch baxter_gazebo baxter_world.launch
   ```

 * Optional: Test/tune the velocity controllers or position controllers using a RQT dashboard GUI

   ```
   roslaunch baxter_control baxter_sdk_velocity_rqt.launch 
   ```
   or
   ```
   roslaunch baxter_control baxter_sdk_position_rqt.launch 
   ```

## Start MoveIt

Works with simulation or hardware:

 * Start MoveIt:

   ```
   roslaunch baxter_moveit_config baxter_bringup.launch
   ```

## Pick and place demo

   ```
   roslaunch baxter_pick_place baxter_pick_place.launch
   ```

## Develop and Contribute

See [Contribute](https://github.com/osrf/baxter/blob/master/CONTRIBUTING.md) page.
