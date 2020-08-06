#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"


ros::Publisher odom_ekf_pub;

void odom_ekf_sub_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
       
   nav_msgs::Odometry odom_ekf;
   //载入里程计时间戳
   odom_ekf.header.stamp = ros::Time::now(); 
   //里程计的父子坐标系
   odom_ekf.header.frame_id = "odom";
   odom_ekf.child_frame_id = "base_footprint";     

    odom_ekf.pose = msg.pose;

    
          //发布里程计
   odom_ekf_pub.publish(odom_ekf);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_ekf_pub");
    ros::NodeHandle nh;

    ros::Subscriber odom_ekf_sub = nh.subscribe("/robot_pose_ekf/odom_combined",1,odom_ekf_sub_callback);
    ros::Publisher odom_ekf_pub = nh.advertise<nav_msgs::Odometry>("/odom_ekf_pub",1);
    
    ros::spin();




        return 0;
}