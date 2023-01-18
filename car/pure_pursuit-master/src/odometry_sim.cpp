#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>


#include <sensor_msgs/PointCloud2.h>

// 位置
#include<geometry_msgs/PointStamped>

// 速度
#include<geometry_msgs::Vector3Stamped>


// imu
#include <sensor_msgs/Imu.h>


#include <geometry_msgs/TransformStamped.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>

#include <string>
#include <vector>

#include "lidar_gnss_mapping/ins_imu.h"

#ifndef __pi__
#define __pi__

const double PI = 3.1415926535898;

#endif

#define DEG_TO_RAD 0.01745329252
#define EARTH_MAJOR 6378137.0            ///< WGS84 MAJOR AXIS
#define EARTH_MINOR 6356752.31424518    ///< WGS84 MINOR AXIS

Eigen::Vector3d lla_origin_;
Eigen::Vector3d ori_origin_;

// nav_msgs::Odometry gnssOdom;
geometry_msgs::TransformStamped gnssTrans;
ros::Publisher pub_lidar_gnss_odom;


ros::Subscriber sub_lidar;
ros::Publisher lidar_pub;


ros::Subscriber sub_pos;
ros::Subscriber sub_vel;
ros::Subscriber sub_imu;

geometry_msgs::PointStamped  posSub;
geometry_msgs::Vector3Stamped velSub;
sensor_msgs::Imu   imuSub;

ros::Publisher lidar_pub;


// bool isFirst = true;


// Eigen::Vector3d LLA2ECEF(const Eigen::Vector3d &lla)
// {
//     Eigen::Vector3d ecef;
//     double lat = lla.x() * M_PI /180;
//     double lon = lla.y() * M_PI /180;
//     double alt = lla.z();

//     double earth_r =pow(EARTH_MAJOR,2)/sqrt(pow(EARTH_MAJOR * cos(lat),2)+ pow(EARTH_MINOR * sin(lat),2));
//     ecef.x() = (earth_r + alt) * cos(lat)*cos(lon);
//     ecef.y() = (earth_r + alt) * cos(lat)*sin(lon);
//     ecef.z() = (pow(EARTH_MINOR/EARTH_MAJOR,2)*earth_r+alt)*sin(lat);
//     return ecef;
// }

// Eigen::Vector3d ECEF2NEU(const Eigen::Vector3d &ecef)
// {
//     double lat = lla_origin_.x() * M_PI/180;
// 	double lon = lla_origin_.y() * M_PI/180;
	
// 	Eigen::Vector3d t = -LLA2ECEF(lla_origin_);
// 	Eigen::Matrix3d r;
// 	r << -sin(lon), cos(lon), 0,
// 			-cos(lon) * sin(lat), -sin(lat) * sin(lon), cos(lat),
// 			cos(lon) * cos(lat), sin(lon) * cos(lat), sin(lat);
	
// 	Eigen::Vector3d enu;
// 	enu = ecef + t;
// 	enu = r * enu;
// 	return enu;
// }


// void lidar_gps_callback(const sensor_msgs::NavSatFixConstPtr &gnss,const lidar_gnss_mapping::ins_imuConstPtr &imu)
// {   
//     double pitch,yaw,roll;
//     pitch = imu -> pitch;
//     yaw = imu -> yaw;
//     roll = imu -> roll;
//     pitch = pitch * M_PI/180;
//     yaw = yaw *M_PI /180;
//     roll = roll * M_PI /180;
    
//        if(isFirst ==true)
//     {
//         lla_origin_.x() = gnss->latitude;
//         lla_origin_.y() = gnss->longitude;
//         lla_origin_.z() = gnss->altitude;
//         ori_origin_.x() = roll;
//         ori_origin_.y() = pitch;
//         ori_origin_.z() = yaw;

//         isFirst = false;
//     }

//     Eigen::Vector3d lla(gnss->latitude,gnss->longitude,gnss->altitude);
//     Eigen::Vector3d ecef = LLA2ECEF(lla);
//     Eigen::Vector3d neu = ECEF2NEU(ecef);

//     roll = roll - ori_origin_.x();
//     pitch = pitch - ori_origin_.y();
//     yaw = yaw - ori_origin_.z();
//     tf::Quaternion orientation;
//     orientation.setRPY(roll,pitch,yaw);
//     geometry_msgs::Quaternion quat;
//     tf::quaternionTFToMsg(orientation,quat);
//     // gnssOdom.pose.pose.orientation = quat;
//     gnssTrans.transform.rotation = quat;
    
//     gnssTrans.transform.translation.x = neu(0);
//     gnssTrans.transform.translation.y = neu(1);
//     gnssTrans.transform.translation.z = neu(2);
//     // gnssOdom.pose.pose.position.x = neu(0);
//     // gnssOdom.pose.pose.position.y = neu(1);
//     // gnssOdom.pose.pose.position.z = neu(2);
//     gnssTrans.header = gnss->header;

//     pub_lidar_gnss_odom.publish(gnssTrans);
// }

// void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &lasermsg)
// {
//     sensor_msgs::PointCloud2 laserOutMsg;
//     laserOutMsg.header = lasermsg->header;
//     laserOutMsg.data = lasermsg->data;
//     laserOutMsg.fields = lasermsg->fields;
//     laserOutMsg.height = lasermsg->height;
//     laserOutMsg.is_bigendian = lasermsg->is_bigendian;
//     laserOutMsg.is_dense = lasermsg->is_dense;
//     laserOutMsg.point_step = lasermsg->point_step;
//     laserOutMsg.row_step = lasermsg->row_step;
//     laserOutMsg.width = lasermsg->width;
//     lidar_pub.publish(laserOutMsg);

// }

void  posCallback(const geometry_msgs::PointStampedPtr &posmsg)
{

   posSub=posmsg;

}

void  velocityCallback(const geometry_msgs::Vector3StampedPtr &velmsg)
{



}

void  imuCallback(const sensor_msgs::ImuPtr &imumsg)
{



}

int main(int argc,char** argv)
{


    ros::init(argc,argv,"sim_dji_odometry");
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m---->\033[0m sim_dji_odometry_init.");


    pub_odom = nh.advertise<nav_msgs::Odometry>("/odom",1000);
  
    sub_pos = nh.subscribe<geometry_msgs::PointStamped>("/dji_sdk/local_position",5,posCallback);


    sub_vel = nh.subscribe<geometry_msgs::Vector3Stamped>("/dji_sdk/",5,velocityCallback);

    sub_imu = nh.subscribe<sensor_msgs::Imu>("/dji_sdk",5,imuCallback);
    // sub_lidar = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points",5,lidarCallback);
    
    ros::spin();

    return 0;


}