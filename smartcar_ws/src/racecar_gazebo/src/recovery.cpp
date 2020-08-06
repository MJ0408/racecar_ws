#include"ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/Int16.h"
#include<std_srvs/Empty.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
ros::NodeHandle node;
ros::Publisher Mode_recovery,first_pose_pub;
ros::Subscriber recovery;
double x_pos,y_pos,z_pos;
void recovery_callback(const std_msgs::Int16Ptr& msg)
{
     gazebo_msgs::ModelState model;
     model.model_name = "shcrobot";
     model.pose.position.x = x_pos;
     model.pose.position.y = y_pos;
     model.pose.position.z = z_pos;
     model.pose.orientation.x = 0;
     model.pose.orientation.y = 0;
     model.pose.orientation.z = 0;
     model.pose.orientation.w = 1;

     Mode_recovery.publish(model);

     ros::service::waitForService("/gazebo/reset_world");
     ros::ServiceClient reset_world = node.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
     
     std_srvs::Empty reset;
     reset_world.call(reset);  
     
    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.pose.position.x = x_pos;
    pose_msg.pose.pose.position.y = y_pos;
    pose_msg.pose.pose.position.z = z_pos;
    pose_msg.pose.covariance[0] = 0.25;
    pose_msg.pose.covariance[6 * 1 + 1] = 0.25;
    pose_msg.pose.covariance[6 * 5 + 5] = 0.06853891945200942;
    pose_msg.pose.pose.orientation.z = 0;
    pose_msg.pose.pose.orientation.w = 1;
    
   // first_pose_pub.publish(pose_msg);

       

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "recovery");
    
    if(!node.getParam("x_pos", x_pos))
         x_pos = -0.5;
    if(!node.getParam("y_pos", y_pos))
         y_pos = 0;
    if(!node.getParam("z_pos", z_pos))
         z_pos = 0;
    
    first_pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
    Mode_recovery = node.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1,true);
    recovery = node.subscribe("/recovery",10,recovery_callback);
     
    ros::spin();
    return 0;
}