convert ROSbag to our lab's format (KITTI alike)

# Description

This package, named rosbag-to-kitti, converts the raw messages in rosbag into KITTI-alike format. 

  - Frequency: ~10Hz. It depends on the topic with lowest frequency. In our case, it's the image topic.
  
  - ROS Topics included:
  
    - "/camera_left/camera/image_raw/compressed"
    
    - "/camera_right/camera/image_raw/compressed"
    
    - "/pandar_points"
    
    - "/piksi/navsatfix_best_fix"
    
    - "/piksi/imu"
    
    - "/piksi/vel_ned"
    
# Usage

If the name of the rosbag is NameofBag.bag

Use it like

```
rosrun rosbag_to_kitti rosbagAPI _bagName:=NameofBag
```

It will create a file named NameofBag in the same directory.

Please make sure that all the topics are publishing meaningful messages, especially for /pandar_points (sometimes it's /pandar_points2)

# Dependencies

```
  rosbag
  cv_bridge
  message_filters
  OpenCV
  PCL
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_conversions
  piksi_rtk_msgs
```

# Data Format

![Data Format](https://github.com/zhao-lab/-trafficnet-2.0/blob/master/preprocess/DataFormat.jpg)
