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
geometry_msgs::Twist cmd_vel;
bool bVel = true;
bool bInit = false;


void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  std::cout<<"收到 init pose"<<std::endl;
  ROS_INFO("I heard: pose with stamp %i", msg.header.stamp);
  // if(!bInit)
  {
    pose_with_covariance = msg.pose;
    bInit = true;
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry2_publisher");

  // 1.ros的基本设置
  ros::NodeHandle n;
  ros::Subscriber sub_init_pose = n.subscribe("/initialpose", 1000, initPoseCallback);
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster camera_broadcaster;
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate r(10);

  // 2.初始化相机位姿
  pose_with_covariance.pose.position = geometry_msgs::Point();
  pose_with_covariance.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  // 3.
  double control_frequency = 0.1;

  // 4.gazebo 模型控制： 根据？？？？  
  // gazebo_msgs::SetModelState objstate;
  // cmd_vel.linear.x = 0.0;
  // cmd_vel.angular.z = 0.0;
  // double pitch = 28.0;
  // double distance = 2.0;
  // double height = 1.0;
  // double angle = 0;
  // objstate.request.model_state.model_name = "mobile_base";  
  // objstate.request.model_state.pose.position.x = distance*cos(angle);
  // objstate.request.model_state.pose.position.y = distance*sin(angle);
  // objstate.request.model_state.pose.position.z = 0.0;
  // tf::Quaternion q_init = tf::createQuaternionFromRPY(0, 0, angle+M_PI); //tf::createQuaternionFromRPY(0, pitch*M_PI/180.0, angle+M_PI);
  // objstate.request.model_state.pose.orientation.w = q_init.w();
  // objstate.request.model_state.pose.orientation.x = q_init.x();
  // objstate.request.model_state.pose.orientation.y = q_init.y();
  // objstate.request.model_state.pose.orientation.z = q_init.z();
  // objstate.request.model_state.twist.linear.x = 0.0;
  // objstate.request.model_state.twist.linear.y = 0.0;
  // objstate.request.model_state.twist.linear.z = 0.0;
  // objstate.request.model_state.twist.angular.x = 0.0;
  // objstate.request.model_state.twist.angular.y = 0.0;
  // objstate.request.model_state.twist.angular.z = 0.0;
  // objstate.request.model_state.reference_frame = "world";
  // gazebo_msgs::SetModelState objstate_gazebo = footprint_to_camera(objstate);
  // client.call(objstate_gazebo);

  // auto current_pose = objstate.request.model_state.pose;

  // 5. 如果初始位置已成功， 且 接收到了movebase的速度
  while(n.ok() /* && bInit && bVel */ ){
    if( !bInit )
      continue;

    // if( !bVel )
    //   continue;

    // 5.1  更新位姿
    double x = pose_with_covariance.pose.position.x + control_frequency*cmd_vel.linear.x * cos(tf::getYaw(pose_with_covariance.pose.orientation));
    double y = pose_with_covariance.pose.position.y + control_frequency*cmd_vel.linear.x * sin(tf::getYaw(pose_with_covariance.pose.orientation));
    double yaw = tf::getYaw(pose_with_covariance.pose.orientation) + control_frequency*cmd_vel.angular.z;
    geometry_msgs::Quaternion Quaternion_odom_robot = tf::createQuaternionMsgFromYaw(yaw); //since all odometry is 6DOF we'll need a quaternion created from yaw
    
    // 5.2 更新速度
    double vx = cmd_vel.linear.x;
    double vy = 0.0;
    double vth = cmd_vel.angular.z;

    // 5.3 更新时间
    current_time = ros::Time::now();

    // 5.4 发布机器人底盘和odom的tf变换
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = Quaternion_odom_robot;
    odom_broadcaster.sendTransform(odom_trans);

    // 5.5 发布camera_depth_optical_frame和base_link的tf变换
    geometry_msgs::TransformStamped camera_trans;
    camera_trans.header.stamp = current_time;
    camera_trans.header.frame_id = "base_link";
    camera_trans.child_frame_id = "camera_depth_optical_frame";
    camera_trans.transform.translation.x = 0.0;
    camera_trans.transform.translation.y = 1.0;
    camera_trans.transform.translation.z = 1.2;
    geometry_msgs::Quaternion Quaternion_robot_camera;
    Quaternion_robot_camera.x = 0.606109;
    Quaternion_robot_camera.y = -0.606109;
    Quaternion_robot_camera.z = 0.364187;
    Quaternion_robot_camera.w = -0.364187;
    camera_trans.transform.rotation = Quaternion_robot_camera;
    camera_broadcaster.sendTransform(camera_trans);

    // 5.6 发布odom的目标位置
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = Quaternion_odom_robot;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    odom_pub.publish(odom);



    last_time = current_time;
    pose_with_covariance.pose = odom.pose.pose;
    // 5.7 更新gazebo的模型位置
    // objstate.request.model_state.pose = odom.pose.pose;
    // objstate.request.model_state.pose.position.z = 2.54855; // height in gazebo
    // gazebo_msgs::SetModelState objstate_gazebo = footprint_to_camera(objstate);
    // client.call(objstate_gazebo);

    // 发布相机相对于odom的变换
    // tf2_ros::StaticTransformBroadcaster broadcaster;
    // geometry_msgs::TransformStamped ts;
    // ts.header.seq=100;
    // ts.header.stamp=ros::Time::now();
    // ts.header.frame_id="odom";
    // ts.child_frame_id="camera_depth_optical_frame";
    // ts.transform.translation.x= ;
    // ts.transform.translation.y=0.0;
    // ts.transform.translation.z=0.0;
    // // ts.transform.rotation.x=0;
    // // ts.transform.rotation.y=0;
    // // ts.transform.rotation.z=0;
    // // tf2::Quaternion qtn;
    // // qtn.setRPY(0,0,0);
    // ts.transform.rotation.x=qtn.getX();
    // ts.transform.rotation.y=qtn.getY();
    // ts.transform.rotation.z=qtn.getZ();
    // ts.transform.rotation.w=qtn.getW();
    // broadcaster.sendTransform(ts);


    ros::spinOnce();               // check for incoming messages
    r.sleep();
  }
}