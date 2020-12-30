# rosbag-to-kitti
This repository is used for transforming raw rosbag data into KITTI format.

# Usage
Assume that the name of rosbag file is 2019-12-22-11-38-05.bag
```
rosrun rosbag_to_kitti rosbagtokitti _bagName:=2019-12-22-11-38-05
```

# References
[ROS Message Synchronizer](http://wiki.ros.org/message_filters)

[Conversion between quaternions and Euler angles](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)

