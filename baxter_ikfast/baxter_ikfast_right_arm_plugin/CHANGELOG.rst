^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package baxter_ikfast_right_arm_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2016-11-15)
------------------
* [fix] isnan -> std::isnan to fix build errors (`#56 <https://github.com/ros-planning/moveit_robots/issues/56>`_)
* [fix] remove redundant include_directories. 
* [fix] Eigen3. Add C++11 support to fix compile bug (`#50 <https://github.com/ros-planning/moveit_robots/issues/50>`_)
* [fix] Added missing dependency on lapack and blas
* Contributors: Dave Coleman, Michael Goerner, Shingo Kitagawa

1.0.6 (2016-04-19)
------------------

1.0.5 (2016-02-10)
------------------

1.0.4 (2016-01-15)
------------------

1.0.3 (2015-11-02)
------------------
* [fix] Manually move ikfast.h from include/ to include/baxter_ikfast\_{left/right}_arm_plugin/include
* Contributors: Kei Okada

1.0.1 (2015-09-19)
------------------
* Initial binary DEB release
* Contributors: Kyle Maroney, Kei Okada
