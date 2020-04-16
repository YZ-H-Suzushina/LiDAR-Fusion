#ifndef CALIB_H
#define CALIB_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

namespace lidar_calib
{

class Calibration
{
private:
    ros::Subscriber lidarSub1;

    ros::Subscriber lidarSub2;

    void refPointCloudCb(sensor_msgs::PointCloud2ConstPtr);

    void freePointCloudCb(sensor_msgs::PointCloud2ConstPtr);

public:
    Calibration(ros::NodeHandle &);

    ~Calibration() {}
};

} // namespace lidar_calib

#endif