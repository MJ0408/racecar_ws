#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include"gazebo_msgs/LinkStates.h"
#include <tf/transform_broadcaster.h>
#include"std_msgs/String.h"
#include"std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "math.h"
#include<iostream>
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <message_filters/subscriber.h>  
#include <message_filters/synchronizer.h> 
#include <message_filters/time_synchronizer.h> 
#include <message_filters/sync_policies/approximate_time.h>  


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState,sensor_msgs::Imu> slamSyncPolicy;  

class Echo_Odom
{
public:
        Echo_Odom();
        //virtual ~Echo_Odom();
        
        void ModelStatesCallback(const gazebo_msgs::ModelStatesConstPtr& box_state_current);
        void joint_callback(const sensor_msgs::JointStateConstPtr& msg);
        void Imu_Callback(const sensor_msgs::ImuConstPtr& msg);
        
        
        void combineCallback(const sensor_msgs::JointStateConstPtr& msg_joint, const sensor_msgs::ImuConstPtr& msg_imu);


        void scan_odom_rf2o(const nav_msgs::OdometryConstPtr& msg);

        void Odom_pub();
        void odom_ekf_sub_callback(const geometry_msgs::PoseWithCovarianceStamped & msg);

double V_front,V_rear,Angular_steer;
ros::Time current_time,last_time;
static float ODOM_POSE_COVARIANCE[] ;
static float ODOM_POSE_COVARIANCE2[] ;
std::string odom_type;
       message_filters::Subscriber<sensor_msgs::JointState>* joint_sub;
       message_filters::Subscriber<sensor_msgs::Imu>* Imu_sub;
       message_filters::Synchronizer<slamSyncPolicy>* sync_;  
static float ODOM_TWIST_COVARIANCE[];
static float ODOM_TWIST_COVARIANCE2[];
private:
       ros::NodeHandle nh;
       ros::Publisher odom_pub,odom_ekf_pub,model_states_pub;
       ros::Subscriber model_states_sub,link_sub,odom_ekf_sub;
       ros::Subscriber rf2o_odom;

       geometry_msgs::Twist twist;
       geometry_msgs::Pose Pose;
       nav_msgs::Odometry odom,odom_ekf,scan_odom;
       double V_l_f,V_r_f,V_l_r,V_r_r,left_steer,right_steer;
       double imu_yaw;
       double Vx,Vy,Vz,th;
       geometry_msgs::Quaternion odom_quat;
       static double X_dist,Y_dist,wheelbase;
       double radius_1,radius_2,radius;
       double odom_vx,odom_vy;
};
    
Echo_Odom::Echo_Odom()
{
       //不允许
      //model_states_sub = nh.subscribe("/gazebo/model_states", 1,&Echo_Odom::ModelStatesCallback,this);
      // ros::Subscriber link_sub = nh.subscribe("/gazebo/link_states", 1, link_states_Callback);
      
      nh.param<std::string>("odom_type",odom_type,"scan_odom");
      
      odom_pub = nh.advertise<nav_msgs::Odometry>("/echo_odom",1);
      

      //消息同步  
      joint_sub = new message_filters::Subscriber<sensor_msgs::JointState>(nh,"/racecar/joint_states",1);
      Imu_sub = new message_filters::Subscriber<sensor_msgs::Imu>(nh,"/imu_data",1);
      sync_ = new  message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(1), *joint_sub, *Imu_sub);  
      sync_->registerCallback(boost::bind(&Echo_Odom::combineCallback,this, _1, _2));  
      
      model_states_pub = nh.advertise<nav_msgs::Odometry>("/gazebo_car",1);
      
      rf2o_odom = nh.subscribe("/odom_rf2o",1,&Echo_Odom::scan_odom_rf2o,this);
      //ekf滤波
      odom_ekf_sub = nh.subscribe("/robot_pose_ekf/odom_combined",1,&Echo_Odom::odom_ekf_sub_callback,this);
      odom_ekf_pub = nh.advertise<nav_msgs::Odometry>("/odom_ekf",1);

}

void Echo_Odom::ModelStatesCallback(const gazebo_msgs::ModelStatesConstPtr& model_state_current)
{
    std::vector<std::string> model_names = model_state_current->name;

     for(int i = 0; i < model_names.size(); i++)
    {
        if(model_names[i] == "racecar")
            std::cout<<model_state_current->twist[i]<<std::endl;
            Pose = model_state_current->pose[i];
           // model_states_pub.publish(Pose);imu_yaw
            odom.header.stamp = ros::Time::now(); 
      //里程计的父子坐标系
     odom.header.frame_id = "odom";
     odom.child_frame_id = "base_footprint";       
   //里程计位置数据：x,y,z,方向
     odom.pose.pose.position.x = Pose.position.x+0.5;     
     odom.pose.pose.position.y = Pose.position.y;
     odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;    
     // model_states_pub.publish(odom);   
     }
    
}


void Echo_Odom::joint_callback(const sensor_msgs::JointStateConstPtr& msg)
{
    std::vector<std::string> joint_names = msg->name;
       for(int i = 0; i < joint_names.size(); i++)
    {
        if(joint_names[i] == "left_front_axle")
            {
                V_l_f=msg->velocity[i];
                //std::cout<< msg->velocity[i] <<std::endl;
            }
        if(joint_names[i] == "right_front_axle")
            {
                V_r_f=msg->velocity[i];
                //std::cout<< msg->velocity[i] <<std::endl;
            } 
        if(joint_names[i] == "left_steering_joint")
            {
                left_steer=msg->position[i];
                //std::cout<< msg->velocity[i] <<std::endl;
            }
        if(joint_names[i] == "right_steering_joint")
            {
                right_steer=msg->position[i];
                //std::cout<< msg->velocity[i] <<std::endl;
            }
        if(joint_names[i] == "left_rear_axle")
            {
                V_l_r=msg->velocity[i];
                //std::cout<< msg->velocity[i] <<std::endl;
            }
        if(joint_names[i] == "right_rear_axle")
            {
                V_r_r=msg->velocity[i];
                //std::cout<< msg->velocity[i] <<std::endl;
            } 
    }

        V_front= (V_l_f+V_r_f)/(2*13.694);
        V_rear = (V_l_r+V_r_r)/(2*13.694);
        Angular_steer  = (right_steer+left_steer)/2;
}


void Echo_Odom::Imu_Callback(const sensor_msgs::ImuConstPtr& msg)
{
    imu_yaw = tf::getYaw(msg->orientation );
    //std::cout<<"imu: "<<tf::getYaw(msg->orientation)<<std::endl;

}

void Echo_Odom::combineCallback(const sensor_msgs::JointStateConstPtr& msg_joint, const sensor_msgs::ImuConstPtr& msg_imu)
{
    std::vector<std::string> joint_names = msg_joint->name;
    for(int i = 0; i < joint_names.size(); i++)
    {
        if(joint_names[i] == "left_front_axle")
            {
                V_l_f=msg_joint->velocity[i];
                //std::cout<< msg->velocity[i] <<std::endl;
            }
        if(joint_names[i] == "right_front_axle")
            {
                V_r_f=msg_joint->velocity[i];
                //std::cout<< msg->velocity[i] <<std::endl;
            } 
        if(joint_names[i] == "left_steering_joint")
            {
                left_steer=msg_joint->position[i];
                //std::cout<< msg->velocity[i] <<std::endl;
            }
        if(joint_names[i] == "right_steering_joint")
            {
                right_steer=msg_joint->position[i];
                //std::cout<< msg->velocity[i] <<std::endl;
            }
        if(joint_names[i] == "left_rear_axle")
            {
                V_l_r=msg_joint->velocity[i];
                //std::cout<< msg->velocity[i] <<std::endl;
            }
        if(joint_names[i] == "right_rear_axle")
            {
                V_r_r=msg_joint->velocity[i];
                //std::cout<< msg->velocity[i] <<std::endl;
            } 
    }

    V_front= (V_l_f+V_r_f)/(2*13.694);
    V_rear = (V_l_r+V_r_r)/(2*13.694);
    Angular_steer  = (right_steer+left_steer)/2;

     imu_yaw = tf::getYaw(msg_imu->orientation );

}
void Echo_Odom::scan_odom_rf2o(const nav_msgs::OdometryConstPtr& msg)
{
     scan_odom.pose=msg->pose;
     scan_odom.twist=msg->twist;

      //std::cout<<scan_odom<<std::endl;

}

void Echo_Odom::Odom_pub()
{
    static  tf::TransformBroadcaster odom_broadcaster;
     odom_quat =  tf::createQuaternionMsgFromYaw(imu_yaw);
    if(odom_type=="wheel_odom")
    {
     
     // std::cout<<"odom_v_front: "<<V_front<<std::endl;

          current_time = ros::Time::now();
     double dt = current_time.toSec() - last_time.toSec();
     std::cout<<"dt"<<dt<<std::endl;
     last_time = current_time;
     
     Vx = (V_front * cos(Angular_steer) +V_rear)/2 ;
     //std::cout<<"vx"<<Vx<<std::endl;
     Vy = V_front * sin(Angular_steer) ;
     
     //radius_1=wheelbase/tan(Angular_steer);
     //radius_2 = wheelbase /sin(Angular_steer);
     //radius = (radius_1+radius_2)/2;
     //Vz = Vx/radius ;
     odom_vx = Vx*cos(imu_yaw) - Vy*sin(imu_yaw);
     odom_vy =  Vx*sin(imu_yaw) + Vy*cos(imu_yaw);
     X_dist += odom_vx *dt;
     Y_dist += odom_vy *dt;

     
    //创建一个tf发布需要使用的TransformStamped类型消息
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    //发布坐标变换的父子坐标系
    odom_trans.header.frame_id = "odom";     
    odom_trans.child_frame_id = "base_footprint";       
    //tf位置数据：x,y,z,方向
    odom_trans.transform.translation.x = X_dist;
    odom_trans.transform.translation.y = Y_dist;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;   
            
    //发布tf坐标变化
    //odom_broadcaster.sendTransform(odom_trans);
    //std::cout<<"ok"<<odom_trans.transform<<std::endl;     
   

   //载入里程计时间戳
   odom.header.stamp = ros::Time::now(); 
   //里程计的父子坐标系
   odom.header.frame_id = "odom";
   odom.child_frame_id = "base_footprint";       
   //里程计位置数据：x,y,z,方向
   odom.pose.pose.position.x = X_dist;     
   odom.pose.pose.position.y = Y_dist;
   odom.pose.pose.position.z = 0.0;
   odom.pose.pose.orientation = odom_quat;       
   //载入线速度和角速度
   odom.twist.twist.linear.x =Vx;
   odom.twist.twist.linear.y = 0;
   odom.twist.twist.angular.z = Angular_steer;
   
       for(int i=0;i<36;i++)
       {
           //车静止的时候
            if(V_l_f<0.01&&V_r_f<0.01&&V_l_r<0.01&&V_r_r<0.01){
                 odom.pose.covariance[i] = ODOM_POSE_COVARIANCE2[i];
                 odom.twist.covariance[i] = ODOM_TWIST_COVARIANCE2[i];
            }
            else{
                 odom.pose.covariance[i] = ODOM_POSE_COVARIANCE[i];
                 odom.twist.covariance[i] = ODOM_TWIST_COVARIANCE[i];
            }

       }
      //发布里程计
      odom_pub.publish(odom);
      
    }
    if(odom_type=="scan_odom")
    {

            //创建一个tf发布需要使用的TransformStamped类型消息
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    //发布坐标变换的父子坐标系
    odom_trans.header.frame_id = "odom";     
    odom_trans.child_frame_id = "base_footprint";       
    //tf位置数据：x,y,z,方向
    odom_trans.transform.translation.x = scan_odom.pose.pose.position.x;
    odom_trans.transform.translation.y = scan_odom.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;  
        //发布tf坐标变化
   // odom_broadcaster.sendTransform(odom_trans);
    //std::cout<<"ok"<<odom_trans.transform<<std::endl; 

        //std::cout<<"star scan_odom "<<std::endl;
        scan_odom.header.stamp = ros::Time::now(); 
        //里程计的父子坐标系
       scan_odom.header.frame_id = "odom";
       scan_odom.child_frame_id = "base_footprint";   

       scan_odom.pose.pose.orientation = odom_quat;   
       //   scan_odom.twist.twist.linear.x =Vx;
       //scan_odom.twist.twist.linear.y = 0;
       //scan_odom.twist.twist.angular.z = Angular_steer;
   
        for(int i=0;i<36;i++)
       {
           //车静止的时候
            if(scan_odom.twist.twist.linear.x<0.01&&scan_odom.twist.twist.angular.z<0.01){
                 scan_odom.pose.covariance[i] = ODOM_POSE_COVARIANCE2[i];
                 scan_odom.twist.covariance[i] = ODOM_TWIST_COVARIANCE2[i];
            }
            else{
                 scan_odom.pose.covariance[i] = ODOM_POSE_COVARIANCE[i];
                 scan_odom.twist.covariance[i] = ODOM_TWIST_COVARIANCE[i];
            }
       }
      //std::cout<<scan_odom<<std::endl;
      odom_pub.publish(scan_odom);
    }  
 
}

void Echo_Odom::odom_ekf_sub_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
       
   nav_msgs::Odometry odom_ekf;
   //载入里程计时间戳
   odom_ekf.header.stamp = ros::Time::now(); 
   //里程计的父子坐标系
   odom_ekf.header.frame_id = "odom";
   odom_ekf.child_frame_id = "base_footprint";     

    odom_ekf.pose = msg.pose;
       //载入线速度和角速度
   odom_ekf.twist.twist.linear.x = Vx;
   odom_ekf.twist.twist.linear.y = 0;
   odom_ekf.twist.twist.angular.z = Angular_steer;
    
          //发布里程计
   odom_ekf_pub.publish(odom_ekf);
}



double Echo_Odom::X_dist=0,Echo_Odom::Y_dist=0;
float Echo_Odom::ODOM_POSE_COVARIANCE[]= {1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3};
float Echo_Odom::ODOM_POSE_COVARIANCE2[] = {1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9};
float Echo_Odom::ODOM_TWIST_COVARIANCE[] = {1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9};
float Echo_Odom::ODOM_TWIST_COVARIANCE2[] = {1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9};
double Echo_Odom::wheelbase=0.335;
int main(int argc, char **argv)
{
     ros::init(argc, argv, "echo_odom");
     ros::NodeHandle nh;
     Echo_Odom Odometry;

     ros::Rate r(50);
       while(nh.ok())
       {
          Odometry.Odom_pub();
          ros::spinOnce();
          r.sleep();

       }
 
    


    return 0;
}