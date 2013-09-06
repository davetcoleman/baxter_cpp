baxter
======

Unofficial Baxter packages that add-on to the Rethink SDK. Currently it contains Gazebo simulation and pick and place MoveIt code for Baxter

NOTE: This is the ROS Hydro readme version. See groovy-devel branch for ROS Groovy instructions.

## Prequisites

 * ROS Hydro
 * Access to the private Rethink [sdk-examples](https://github.com/RethinkRobotics/sdk-examples) repository - we are using the baxter_interface and head_control packages from the SDK

## Installation

* Create a catkin workspace and cd into it:

```
    mkdir -p ~/baxter_ws/src
    cd ~/baxter_ws/src
    catkin_init_workspace
```

* Checkout this repo

```
    git clone git@github.com:davetcoleman/baxter.git -b hydro-devel
```

* Also install from source a few customized repositories:

```
    git clone git@github.com:RethinkRobotics/sdk-examples.git -b gazebo_dev
    git clone git@github.com:davetcoleman/baxter_common.git -b dual_parallel_grippers
    git clone git@github.com:davetcoleman/block_grasp_generator.git -b hydro-devel
    git clone git@github.com:ros-controls/ros_controllers -b velocity_position_controller
```

* Disable duplicate packages

There is currently a duplication of packages in sdk-examples and baxter_common that must be fixed manually. This issue should be fixed in Rethink's next release of their SDK

```
    cd ~/baxter_ws/
    touch src/sdk-examples/baxter_description/CATKIN_IGNORE
    touch src/sdk-examples/baxter_msgs/CATKIN_IGNORE
```

* Install dependencies

```
    rosdep install --from-paths . --ignore-src --rosdistro hydro -y
```

* Build

```
    catkin_make
```

* Add setup.bash to your .bashrc (recommended)

```
    echo 'source ~/baxter_ws/devel/setup.bash' >> ~/.bashrc
```

## Bringup Baxter

### Hardware

 * Ensure you have the correct ROS_MASTER_URI exported, this depends on your robot serial number:
   ```
   export ROS_MASTER_URI=http://011305P0009.local:11311
   ```

 * Turn on baxter

 * Enable robot:
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
