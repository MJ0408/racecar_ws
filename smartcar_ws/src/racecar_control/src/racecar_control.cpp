#include"ros/ros.h"
#include <geometry_msgs/Twist.h>
#include"std_msgs/Float64.h"
#include"std_msgs/Float32.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "std_msgs/Int16.h"
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#define pi 3.14159

double wheelbase =0.335,tire_dia=0.14605,k=tire_dia/2;

class Control_server
{
public:
     Control_server();
     

     //cmd_vel 转  ackermann_msgs
     void cmd_callback(const geometry_msgs::TwistConstPtr& msg);
     //正常twist消息转steering
     double convert_trans_rot_vel_to_steering_angle(float v,float omega ,float wheelbase);
     //steering转 w
     double convert_trans_steering_angle_to_rot_vel(float v,float omega ,float wheelbase);
     //关节控制
     void joint_contol(const ackermann_msgs::AckermannDriveStampedConstPtr& msg);
     
     void recovery_callback(const std_msgs::Int16ConstPtr& msg);
     ros::NodeHandle nh;
     ros::Publisher  ackermann_cmd_pub,first_pose_pub; 
     ros::Publisher pub_vel_left_rear_wheel,pub_vel_right_rear_wheel,pub_vel_left_front_wheel,pub_vel_right_front_wheel;
     ros::Publisher pub_pos_left_steering_hinge,pub_pos_right_steering_hinge;
     
     ros::Subscriber  Cmd_vel,vel_output,recovery;
     
     ros::ServiceClient reset_world;
     std_msgs::Float64 throttle,steer,Steer,throttle_front,throttle_rear;
private:
      double x_pos,y_pos,z_pos,W_car;
      double steering;
      double radius;
      double Vz;
     double R;
};


Control_server::Control_server()
  {     
    
        if(!nh.getParam("x_pos", x_pos))
             x_pos = -0.5;
        if(!nh.getParam("y_pos", y_pos))
             y_pos = 0;
        if(!nh.getParam("z_pos", z_pos))
             z_pos = 0;
         
        ackermann_cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/ackermann_cmd_mux/input/navigation", 10, true);
        
        Cmd_vel = nh.subscribe("/cmd_vel",1,&Control_server::cmd_callback,this);
        //vel_output =  nh.subscribe("/racecar/ackermann_cmd_mux/output",1,&Control_server::joint_contol,this);
        recovery = nh.subscribe("/recovery",1,&Control_server::recovery_callback,this);
        
        reset_world = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
        
        first_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);


        pub_vel_left_rear_wheel   = nh.advertise<std_msgs::Float64>("/racecar/left_rear_wheel_velocity_controller/command",1, true);
        pub_vel_right_rear_wheel  = nh.advertise<std_msgs::Float64>("/racecar/right_rear_wheel_velocity_controller/command", 1, true);
        pub_vel_left_front_wheel  = nh.advertise<std_msgs::Float64>("/racecar/left_front_wheel_velocity_controller/command", 1, true);
        pub_vel_right_front_wheel = nh.advertise<std_msgs::Float64>("/racecar/right_front_wheel_velocity_controller/command", 1, true);

        pub_pos_left_steering_hinge  = nh.advertise<std_msgs::Float64>("/racecar/left_steering_hinge_position_controller/command", 1, true);
        pub_pos_right_steering_hinge = nh.advertise<std_msgs::Float64>("/racecar/right_steering_hinge_position_controller/command", 1, true);
  }


void Control_server::joint_contol(const ackermann_msgs::AckermannDriveStampedConstPtr& msg)
{


    // Velocity is in terms of radians per second.
    // Want to go 1 m/s with a wheel of radius 0.05m. This translates to 19.97 radians per second, roughly 20.
    // However, at a multiplication factor of 20 speed is half of what it should be, so doubled to 40. 
   //  w*radius =v   w 角速度

   
     steer.data=msg->drive.steering_angle;
    throttle.data = msg->drive.speed /k;
    throttle_rear.data = throttle.data;
    throttle_front.data = throttle.data;
    /**
    if(steering!=0)
    {
    W_car= convert_trans_steering_angle_to_rot_vel( msg->drive.speed,steer.data,wheelbase);
    throttle_front.data = W_car * (wheelbase/sin(steer.data))/k*2;
    throttle_rear.data = W_car * R/k*2;
    std::cout<<"throttle_front:"<<throttle_front.data*13.694<<"throttle_rear:"<<throttle_rear.data*13.694<<"V_tran:"<<(throttle_front.data * cos(steering) + throttle_rear.data )*13.694<<std::endl;
    }
    
    else
     {   throttle_front.data = throttle.data;
        throttle_rear.data = throttle.data;
    
      }  

      **/ 
      //throttle.data = msg->drive.speed/k;
    //throttle.data= 10;
    //steer.data = msg->drive.steering_angle;
    //std::cout<<"throttle: "<<throttle<<" steer: "<<steer<<std::endl;

    pub_vel_left_rear_wheel.publish(throttle_rear);
    pub_vel_right_rear_wheel.publish(throttle_rear);
    pub_vel_left_front_wheel.publish(throttle_front);
    pub_vel_right_front_wheel.publish(throttle_front);
    
    pub_pos_left_steering_hinge.publish(steer);
    pub_pos_right_steering_hinge.publish(steer);
 
     
}

double Control_server::convert_trans_rot_vel_to_steering_angle(float v,float omega ,float wheelbase)
{
    
   if(v==0||omega==0)
      return 0;
    radius = v / omega;
    return atan(wheelbase / radius);

}
double Control_server::convert_trans_steering_angle_to_rot_vel(float v,float steering,float wheelbase)
{
    if(steering==0)
      return 0;
    R=wheelbase/tan(steering);
    Vz=v/R;
     return Vz;
}

void Control_server::cmd_callback(const geometry_msgs::TwistConstPtr& msg)
{
   /**twist 转 阿克曼 消息
    //steering = convert_trans_rot_vel_to_steering_angle(msg->linear.x,msg->angular.z,wheelbase);
    steering = msg->angular.z;
    ackermann_msgs::AckermannDriveStamped ackermann_cmd;
    ackermann_cmd.header.stamp = ros::Time::now();
    ackermann_cmd.drive.speed = msg->linear.x ;
    ackermann_cmd.drive.steering_angle = steering;
    //ackermann_cmd_pub.publish(ackermann_cmd);
     **/
     
    steer.data=msg->angular.z;
    throttle.data = msg->linear.x /k;
    throttle_rear.data = throttle.data;
    throttle_front.data = throttle.data;
    pub_vel_left_rear_wheel.publish(throttle);
    pub_vel_right_rear_wheel.publish(throttle);
    pub_vel_left_front_wheel.publish(throttle);
    pub_vel_right_front_wheel.publish(throttle); 
    pub_pos_left_steering_hinge.publish(steer);
    pub_pos_right_steering_hinge.publish(steer);
  
    
}
void Control_server::recovery_callback(const std_msgs::Int16ConstPtr& msg)
{

     std::cout<<"x_pos:"<<x_pos<<"y_pos:"<<y_pos<<"z_pos:"<<z_pos<<std::endl;
     ros::service::waitForService("/gazebo/reset_world");
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

     first_pose_pub.publish(pose_msg);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "racecar_Control");
    Control_server go;
    
     ros::AsyncSpinner spinner(4); // Use 4 threads
     spinner.start();
     ros::waitForShutdown();
     spinner.stop();
    ros::spin();

    return 0;
}