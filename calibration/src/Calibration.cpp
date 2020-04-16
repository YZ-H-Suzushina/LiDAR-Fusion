#include "../include/calibration/Calibration.h"

namespace lidar_calib
{

Calibration::Calibration(ros::NodeHandle &nh)
{
    lidarSub1 = nh.subscribe("/ns1/rslidar_points", 20, &Calibration::refPointCloudCb, this);
    lidarSub2 = nh.subscribe("/ns2/rslidar_points", 20, &Calibration::freePointCloudCb, this);

    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Calibration::refPointCloudCb(sensor_msgs::PointCloud2ConstPtr rawPoints)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr refLidarCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*rawPoints, *refLidarCloud);


}

void Calibration::freePointCloudCb(sensor_msgs::PointCloud2ConstPtr rawPoints)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr freeLidarCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*rawPoints, *freeLidarCloud);

}

}