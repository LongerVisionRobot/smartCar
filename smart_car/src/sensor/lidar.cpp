#include "ros/ros.h"
#include <string.h> 
#include "sensor_msgs/LaserScan.h"
#include <smart_car/Velocity.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*180./M_PI)
#endif

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*M_PI/180.)
#endif

ros::Publisher lidar_pub;
ros::Subscriber lidar_sub;
// pcl::visualization::PCLVisualizer viewer;

int _isnan(double x) { return x != x; }
int _isinf(double x) { return !_isnan(x) && _isnan(x - x); }

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::LaserScan scan_msg;
    ROS_INFO("Get the lidar data %d", scan->ranges.size());

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor/lidar");
    ROS_INFO("sensor/lidar started.");

    ros::NodeHandle n;
    
    lidar_sub = n.subscribe<sensor_msgs::LaserScan>("/smart_car/scan", 100, scanCallback);
    
    ros::spin();

    ros::shutdown();
    return 0;
}


// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
// int countNum = scan->ranges.size();
// cloud_msg->height = 1;
// cloud_msg->width  = countNum;
// cloud_msg->points.resize(cloud_msg->width * cloud_msg->height);
// for(int i = 0; i < countNum; i++) {
//     float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);  
//     if(scan->ranges[i] && (_isinf(scan->ranges[i])==0)){
//         cloud_msg->points[i].x = scan->ranges[i]*cos(DEG2RAD(degree));
//         cloud_msg->points[i].y = scan->ranges[i]*sin(DEG2RAD(degree));
//         cloud_msg->points[i].z = 0;
//     }
//     else
//     {
//         cloud_msg->points[i].x = 0;
//         cloud_msg->points[i].y = 0;
//         cloud_msg->points[i].z = 0;
//     }
//     ROS_INFO("point i %f %f %f", cloud_msg->points[i].x,cloud_msg->points[i].y,cloud_msg->points[i].z); 
// }
// viewer.removePointCloud("cloud_msg"); 
// viewer.addPointCloud(cloud_msg, "cloud_msg");