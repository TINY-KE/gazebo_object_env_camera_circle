#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gazebo_msgs/SetModelState.h>

// tf
#include <vector>
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"


geometry_msgs::PoseWithCovariance pose_with_covariance;
geometry_msgs::PoseWithCovariance footprint__pose;
geometry_msgs::Twist mCmd_vel;
bool bVel = false;
bool bInit = false;
ros::Time last_time, current_time;
gazebo_msgs::SetModelState baselink_to_camerabody_to_gazebo( gazebo_msgs::SetModelState& objstate){
  double pitch = 28.0;
  double distance = 2.0;
  double height = 1.0;
  double angle = 0;
  gazebo_msgs::SetModelState objstate_gazebo = objstate;

  double yaw = tf::getYaw(objstate.request.model_state.pose.orientation) ;
  tf::Quaternion q_init_gazebo = tf::createQuaternionFromRPY(0, pitch*M_PI/180.0, yaw);
  objstate_gazebo.request.model_state.pose.orientation.w = q_init_gazebo.w();
  objstate_gazebo.request.model_state.pose.orientation.x = q_init_gazebo.x();
  objstate_gazebo.request.model_state.pose.orientation.y = q_init_gazebo.y();
  objstate_gazebo.request.model_state.pose.orientation.z = q_init_gazebo.z();
  return objstate_gazebo;
}

void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  ROS_INFO("I heard: pose with stamp %i", msg.header.stamp);
  if(!bInit)
  {
    footprint__pose = msg.pose;
    bInit = true;
  }
  std::cout<<"x3:"<<msg.pose.pose.position.x<<",y3:"<<msg.pose.pose.position.y<<std::endl;
  pose_with_covariance = msg.pose;
  std::cout<<"x2:"<<pose_with_covariance.pose.position.x<<",y2:"<<pose_with_covariance.pose.position.y<<std::endl;
}

void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("I heard: cmd vel %f %f", msg->linear.x, msg->angular.z);
  mCmd_vel = *msg;
  bVel = true;
  last_time = current_time;
  current_time = ros::Time::now();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry2_publisher");

  // 1.ros的基本设置
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub_init_pose = n.subscribe("/initialpose", 1000, initPoseCallback);
  ros::Subscriber sub_cmd_vel   = n.subscribe("/cmd_vel", 1000, cmdvelCallback);
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster camera_broadcaster;
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate r(10);

  // 2.初始化参数
  pose_with_covariance.pose.position = geometry_msgs::Point();
  pose_with_covariance.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  mCmd_vel.linear.x = 0;
  mCmd_vel.angular.z = 0;
  // 3.起点的位姿
  double distance = 2.0;
  double height = 1.0;
  double angle = 0;
  double initx = distance*cos(angle);
  double inity = distance*sin(angle);
  double initz = height;
  

  // 4.gazebo 模型控制的ros节点
  gazebo_msgs::SetModelState objstate;
  double pitch = 28.0;
  objstate.request.model_state.model_name = "mobile_base";  
  objstate.request.model_state.pose.position.x = distance*cos(angle);
  objstate.request.model_state.pose.position.y = distance*sin(angle);
  objstate.request.model_state.pose.position.z = height;
  tf::Quaternion q_init = tf::createQuaternionFromRPY(0, 0, angle+M_PI); //tf::createQuaternionFromRPY(0, pitch*M_PI/180.0, angle+M_PI);
  objstate.request.model_state.pose.orientation.w = 0;
  objstate.request.model_state.pose.orientation.x = -0.241922;  
  objstate.request.model_state.pose.orientation.y = 0;
  objstate.request.model_state.pose.orientation.z = 0.970296;
  objstate.request.model_state.twist.linear.x = 0.0;
  objstate.request.model_state.twist.linear.y = 0.0;
  objstate.request.model_state.twist.linear.z = 0.0;
  objstate.request.model_state.twist.angular.x = 0.0;
  objstate.request.model_state.twist.angular.y = 0.0;
  objstate.request.model_state.twist.angular.z = 0.0;
  objstate.request.model_state.reference_frame = "world";
  // gazebo_msgs::SetModelState objstate_gazebo = footprint_to_camera(objstate);
  // client.call(objstate);

  // auto current_pose = objstate.request.model_state.pose;
  ROS_INFO("waiting");
  
  while(n.ok() /* && bInit && bVel */ ){
    
    // 如果接收到了速度
    if( bInit && bVel ){

      ROS_INFO("running1");
      bVel = false;  //同一条/cmd_vel只使用一次
      // if( !bVel )
      //   continue;

      // 5.1 (1) 使用orbslam的结果更新位姿
      // double x = pose_with_covariance.pose.position.x + control_frequency*cmd_vel.linear.x * cos(tf::getYaw(pose_with_covariance.pose.orientation));
      // double y = pose_with_covariance.pose.position.y + control_frequency*cmd_vel.linear.x * sin(tf::getYaw(pose_with_covariance.pose.orientation));
      // double yaw = tf::getYaw(pose_with_covariance.pose.orientation) + control_frequency*cmd_vel.angular.z;
      // geometry_msgs::Quaternion Quaternion_odom_robot = tf::createQuaternionMsgFromYaw(yaw); //since all odometry is 6DOF we'll need a quaternion created from yaw
      // std::cout<<"x:"<<x<<",y:"<<y<<std::endl;
      // std::cout<<"x1:"<<pose_with_covariance.pose.position.x<<",y1:"<<pose_with_covariance.pose.position.y<<std::endl;

      // 5.1 (2) 使用cmdvel更新位姿
      // double control_frequency = 0.1;
      // double x = footprint__pose.pose.position.x + control_frequency*mCmd_vel.linear.x * cos(tf::getYaw(footprint__pose.pose.orientation));
      // double y = footprint__pose.pose.position.y + control_frequency*mCmd_vel.linear.x * sin(tf::getYaw(footprint__pose.pose.orientation));
      // double yaw = tf::getYaw(footprint__pose.pose.orientation) + control_frequency*mCmd_vel.angular.z;
      // geometry_msgs::Quaternion Quaternion_odom_robot = tf::createQuaternionMsgFromYaw(yaw); //since all odometry is 6DOF we'll need a quaternion created from yaw
      // std::cout<<"x:"<<x<<",y:"<<y<<std::endl;
      // std::cout<<"x1:"<<footprint__pose.pose.position.x<<",y1:"<<footprint__pose.pose.position.y<<std::endl;

      // 5.1 (3) 使用cmdvel更新位姿
      double yaw = tf::getYaw(footprint__pose.pose.orientation);
      double dt = (current_time - last_time).toSec();
      double delat_x = (mCmd_vel.linear.x * cos(yaw) - mCmd_vel.linear.y * sin(yaw)) * dt;
      double delat_y = (mCmd_vel.linear.x * sin(yaw) + mCmd_vel.linear.y * cos(yaw)) * dt;
      double delat_yaw = mCmd_vel.angular.z * dt;
      footprint__pose.pose.position.x += delat_x;
      footprint__pose.pose.position.y += delat_y;
      yaw += delat_yaw;
      geometry_msgs::Quaternion Quaternion_odom_robot = tf::createQuaternionMsgFromYaw(yaw); //since all odometry is 6DOF we'll need a quaternion created from yaw
      footprint__pose.pose.orientation = Quaternion_odom_robot;

      // 5.2 更新速度
      double vx = mCmd_vel.linear.x;
      double vy = 0.0;
      double vth = mCmd_vel.angular.z;
      

      // 5.4 发布机器人底盘和odom的tf变换
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";
      odom_trans.transform.translation.x = footprint__pose.pose.position.x;
      odom_trans.transform.translation.y = footprint__pose.pose.position.y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = footprint__pose.pose.orientation; //Quaternion_odom_robot;
      // odom_broadcaster.sendTransform(odom_trans);

      // 5.5 发布camera_depth_optical_frame和base_link的tf变换
      geometry_msgs::TransformStamped camera_trans;
      camera_trans.header.stamp = current_time;
      camera_trans.header.frame_id = "base_link";
      camera_trans.child_frame_id = "camera_depth_optical_frame";
      camera_trans.transform.translation.x = 0.0;
      camera_trans.transform.translation.y = 0.0;
      camera_trans.transform.translation.z = 1.2;
      geometry_msgs::Quaternion Quaternion_robot_camera;
      Quaternion_robot_camera.x = 0.606109;
      Quaternion_robot_camera.y = -0.606109;
      Quaternion_robot_camera.z = 0.364187;
      Quaternion_robot_camera.w = -0.364187;
      camera_trans.transform.rotation = Quaternion_robot_camera;
      // camera_broadcaster.sendTransform(camera_trans);

      // 5.6 发布里程计信息【似乎没什么用】
      // nav_msgs::Odometry odom;
      // odom.header.stamp = current_time;
      // odom.header.frame_id = "odom";
      // odom.pose.pose.position.x = x;
      // odom.pose.pose.position.y = y;
      // odom.pose.pose.position.z = 0.0;
      // odom.pose.pose.orientation = Quaternion_odom_robot;
      // odom.child_frame_id = "base_link";
      // odom.twist.twist.linear.x = vx;
      // odom.twist.twist.linear.y = vy;
      // odom.twist.twist.angular.z = vth;
      // odom_pub.publish(odom);
      // last_time = current_time;
      // pose_with_covariance.pose = odom.pose.pose;


      // 5.7 更新gazebo的模型位置
      objstate.request.model_state.pose.position.x = odom_trans.transform.translation.x;// + initx;
      objstate.request.model_state.pose.position.y = odom_trans.transform.translation.y;// + inity;
      objstate.request.model_state.pose.position.z = 1.0;
      objstate.request.model_state.pose.orientation.w = odom_trans.transform.rotation.w;
      objstate.request.model_state.pose.orientation.x = odom_trans.transform.rotation.x;
      objstate.request.model_state.pose.orientation.y = odom_trans.transform.rotation.y;
      objstate.request.model_state.pose.orientation.z = odom_trans.transform.rotation.z;
      // gazebo_msgs::SetModelState 
      objstate = baselink_to_camerabody_to_gazebo(objstate);
      client.call(objstate);
      
    }
    else if(bInit && !bVel){
      ROS_INFO("running2");
      // 6.1 更新位姿
      double x = footprint__pose.pose.position.x ;
      double y = footprint__pose.pose.position.y ;
      double yaw = tf::getYaw(footprint__pose.pose.orientation);
      geometry_msgs::Quaternion Quaternion_odom_robot = tf::createQuaternionMsgFromYaw(yaw); //since all odometry is 6DOF we'll need a quaternion created from yaw
      std::cout<<"x:"<<x<<",y:"<<y<<std::endl;
      std::cout<<"x1:"<<footprint__pose.pose.position.x<<",y1:"<<footprint__pose.pose.position.y<<std::endl;

      
      // 5.2 发布机器人底盘和odom的tf变换
      current_time = ros::Time::now();

      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";
      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = Quaternion_odom_robot;
      // odom_broadcaster.sendTransform(odom_trans);

      // 5.3 发布camera_depth_optical_frame和base_link的tf变换
      geometry_msgs::TransformStamped camera_trans;
      camera_trans.header.stamp = current_time;
      camera_trans.header.frame_id = "base_link";
      camera_trans.child_frame_id = "camera_depth_optical_frame";
      camera_trans.transform.translation.x = 0.0;
      camera_trans.transform.translation.y = 0.0;
      camera_trans.transform.translation.z = 1.2;
      geometry_msgs::Quaternion Quaternion_robot_camera;
      Quaternion_robot_camera.w = -0.364187;
      Quaternion_robot_camera.x = 0.606109;
      Quaternion_robot_camera.y = -0.606109;
      Quaternion_robot_camera.z = 0.364187;
      camera_trans.transform.rotation = Quaternion_robot_camera;
      // camera_broadcaster.sendTransform(camera_trans);
      
    }

    ros::spinOnce();               // check for incoming messages
    r.sleep();
  }
}
