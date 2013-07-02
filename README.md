baxter
======

Non-SDK packages for Baxter - you need access to RethinkRobotics [sdk-examples](https://github.com/RethinkRobotics/sdk-examples) to use the rest of these packages. Currently their SDK is closed source.


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
