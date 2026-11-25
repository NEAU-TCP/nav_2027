# isaac_sim_pointcloud_tool

This package converts pointcloud in Ignition Gazebo to velodyne format. This is a ROS package. It subscribes to the LiDAR rostopic which is published by Ignition Gazebo. And it republish a LiDAR rostopoic in Velodyne format.

Gazebo GPU lidar publishes an instantaneous rendered cloud. The output keeps the
input acquisition stamp and sets every relative `time` field to zero. It does not
synthesize a rotating scan from point indices. Small Point-LIO consumes these
offsets in seconds with `point_time_scale: 1.0` and `point_time_relative: true`.
Nonzero synthetic offsets distort motion estimation when the robot moves.

Some SLAM algorithm needs pointcloud in Velodyne format so that it can extract corner points. But Isaac ROS only send pointcloud contains XYZ information. This package helps to convert pointcloud to velodyne format.
