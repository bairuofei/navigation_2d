# A modified version for `navigation_2d` package. 

This is a ROS package for [Graph-Based_SLAM-Aware_Exploration](https://github.com/bairuofei/Graph-Based_SLAM-Aware_Exploration). 
It is modified from [navigation_2d](https://github.com/skasperski/navigation_2d), which provides ROS nodes to navigate a mobile robot in a planar environment.

Documentation and Tutorials can be found in the [ROS-Wiki](http://wiki.ros.org/nav2d)

## Main modifications
Main functions of this package is not changed. The modifications mainly include:
1. Add ros publishers for SLAM pose graph visualization;
2. Improve frontier-based method for frontier checking;
3. Record waypoint planning time in fronter-based method.

