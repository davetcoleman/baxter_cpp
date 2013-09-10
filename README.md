baxter
======

Unofficial Baxter packages that add-on to the Rethink SDK. It is entirely written in C++ and is the location of on going development by [Dave Coleman](http://davetcoleman.com). 

Currently it contains Gazebo simulation and pick and place MoveIt code for Baxter. The more stable version is the groovy-devel branch.

NOTE: This is the ROS Hydro readme version. See groovy-devel branch for ROS Groovy instructions.

## Prequisites

 * A Baxter with dual parallel grippers, or the desire to see one in simulation
 * [ROS Hydro](http://wiki.ros.org/ROS/Installation)
 * Access to the private Rethink [sdk-examples](https://github.com/RethinkRobotics/sdk-examples) repository - we are using the baxter_interface and head_control packages from the SDK. Contact [Dave](davetcoleman@gmail.com) if you should have access to this.
 * Setup Github (recommended) - the git@github.com urls, below, only work if you have [Setup Github](https://help.github.com/articles/set-up-git) and generated [SSH Keys for Github](https://help.github.com/articles/generating-ssh-keys). Otherwise, change the below URLS to say "https://github.com/".

## Installation

* Install wstool package

```
    sudo apt-get install python-wstool
```

* Create a catkin workspace (we recommend a separate one for Baxter) and ``cd`` into it:

```
    mkdir -p ~/ros/baxter_ws/src
    cd ~/ros/baxter_ws/src
    wstool init .
    wstool merge https://raw.github.com/davetcoleman/baxter/hydro-devel/baxter.rosinstall
    wstool update
```

* Also install from source the private RethinkRobotics SDK:

```
    git clone git@github.com:RethinkRobotics/sdk-examples.git -b gazebo_dev
```

* Disable duplicate packages

    There is currently a duplication of packages in sdk-examples and baxter_common that must be fixed manually. This issue should be fixed in Rethink's next release of their SDK:

```
    cd ~/ros/baxter_ws/
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

* Add Baxter setup.bash to your .bashrc (recommended)

```
    echo 'source ~/ros/baxter_ws/devel/setup.bash' >> ~/.bashrc
```

* Install MoveIt! From Source (currently not required - the debians are still the same)

    We have chosen to freeze the version of MoveIt! we are using for the short-run to ensure compatibilty with our customizations. We recommend you install this in a seperate workspace to decrease built times.

```
    mkdir -p ~/ros/moveit_ws/src
    cd ~/ros/moveit_ws/src
    wstool init .
    wstool merge https://raw.github.com/davetcoleman/baxter/hydro-devel/moveit.rosinstall
    wstool update
    cd ..
    rosdep install --from-paths src --ignore-src --rosdistro hydro -y
    catkin_make
```

* Add MoveIt setup.bash to your .bashrc (recommended)

```
    echo 'source ~/ros/moveit_ws/devel/setup.bash' >> ~/.bashrc
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
