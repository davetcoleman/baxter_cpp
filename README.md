baxter
======

ROS Groovy Catkinized version of Baxter.

**Beta release!**

### Changes from RethinkRobotics/sdk-examples

* Uses catkin instead of rosbuild (a few baxter_examples are still using rosbuild)
* All packages are prefixed with 'baxter_' - i.e. ``rosrun baxter_tools enable.py -e``
* Integration of MoveIt!

### Installation

* Create a catkin workspace and cd into it:
    cd ~/catkin_ws/src
* Checkout this repo
    git clone git@github.com:osrf/baxter.git
* Install dependencies
    rosdep install --from-paths src --ignore-src --rosdistro groovy -y
* Build
    cd ..
    catkin_make

### Develop and Contribute

See [Contribute](https://github.com/osrf/baxter/blob/master/CONTRIBUTING.md) page.
