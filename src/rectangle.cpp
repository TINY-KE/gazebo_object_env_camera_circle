#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <math.h>
#include <assert.h>
#include <iostream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include "tf/transform_datatypes.h"

 gazebo_msgs::SetModelState publish(double x,  double y, double z, double yaw){
    gazebo_msgs::SetModelState objstate;
    objstate.request.model_state.model_name = "mobile_base";//"acircles_pattern_0";
    objstate.request.model_state.pose.position.x = x;
    objstate.request.model_state.pose.position.y = y;
    objstate.request.model_state.pose.position.z = z;

    tf::Quaternion q_init = tf::createQuaternionFromRPY(0, 35.0/180.0*M_PI, yaw);
    objstate.request.model_state.pose.orientation.w = q_init.w();
    objstate.request.model_state.pose.orientation.x = q_init.x();
    objstate.request.model_state.pose.orientation.y = q_init.y();
    objstate.request.model_state.pose.orientation.z = q_init.z();
    objstate.request.model_state.twist.linear.x = 0.0;
    objstate.request.model_state.twist.linear.y = 0.0;
    objstate.request.model_state.twist.linear.z = 0.0;
    objstate.request.model_state.twist.angular.x = 0.0;
    objstate.request.model_state.twist.angular.y = 0.0;
    objstate.request.model_state.twist.angular.z = 0.0;
    objstate.request.model_state.reference_frame = "world";

    std::cout<<"imu pose x:"<<objstate.request.model_state.pose.position.x
                <<",y:"<<objstate.request.model_state.pose.position.y
                <<",z:"<<objstate.request.model_state.pose.position.z
                <<",qw:"<<objstate.request.model_state.pose.orientation.w
                <<",qx:"<<objstate.request.model_state.pose.orientation.x
                <<",qy:"<<objstate.request.model_state.pose.orientation.y
                <<",qz:"<<objstate.request.model_state.pose.orientation.z
                <<std::endl;
    return objstate;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_set_states_publisher");
 
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
      


    double half_wide = 1.0;
    double half_length = 1.0;
    double height = 1.0;
    double step = 0.001;
    gazebo_msgs::SetModelState objstate = publish( half_wide, half_length, height, M_PI);
    client.call(objstate);
    std::cout<<"INIT POSE ----------------------"<<std::endl;
    std::cout<<"按任意键继续 ----------------------"<<std::endl;
    double cout = 0;

    ros::Rate loop_rate(300);
    std::cin.get();
    while(ros::ok()) {
        double start_angle, end_angle, end_pos_x, end_pos_y;
        double backAngle = M_PI*0.2;
        //0
        start_angle=M_PI, end_angle=M_PI*1.5-backAngle, end_pos_x=half_wide, end_pos_y=half_length;
        for(double angle=start_angle; angle<=end_angle; angle+=step){
            gazebo_msgs::SetModelState objstate = publish( end_pos_x, end_pos_y, height, angle);
            client.call(objstate);
            loop_rate.sleep();
        }
        start_angle=M_PI*1.5-backAngle, end_angle=M_PI, end_pos_x=half_wide, end_pos_y=half_length;
        for(double angle=start_angle; angle>=end_angle; angle-=step){
            gazebo_msgs::SetModelState objstate = publish( end_pos_x, end_pos_y, height, angle);
            client.call(objstate);
            loop_rate.sleep();
        }

        //1
        for(double x= half_wide; x>=-half_wide; x-=step){
            gazebo_msgs::SetModelState objstate = publish( x, half_length, height, M_PI);
            client.call(objstate);
            loop_rate.sleep();
        }
        start_angle=M_PI, end_angle=M_PI*2.0-backAngle, end_pos_x=-half_wide, end_pos_y=half_length;
        for(double angle=start_angle; angle<=end_angle; angle+=step){
            gazebo_msgs::SetModelState objstate = publish( end_pos_x, end_pos_y, height, angle);
            client.call(objstate);
            loop_rate.sleep();
        }
        start_angle=M_PI*2.0-backAngle, end_angle=M_PI*1.5, end_pos_x=-half_wide, end_pos_y=half_length;
        for(double angle=start_angle; angle>=end_angle; angle-=step){
            gazebo_msgs::SetModelState objstate = publish( end_pos_x, end_pos_y, height, angle);
            client.call(objstate);
            loop_rate.sleep();
        }

        //2
        for(double y= half_length; y>=-half_length; y-=step){
            gazebo_msgs::SetModelState objstate = publish( -half_wide, y, height, M_PI*1.5);
            client.call(objstate);
            loop_rate.sleep();
        }
        start_angle=M_PI*1.5, end_angle=M_PI*2.5-backAngle, end_pos_x=-half_wide, end_pos_y=-half_length;
        for(double angle=start_angle; angle<=end_angle; angle+=step){
            gazebo_msgs::SetModelState objstate = publish( end_pos_x, end_pos_y, height, angle);
            client.call(objstate);
            loop_rate.sleep();
        }
        start_angle=M_PI*2.5-backAngle, end_angle=M_PI*2.0, end_pos_x=-half_wide, end_pos_y=-half_length;
        for(double angle=start_angle; angle>=end_angle; angle-=step){
            gazebo_msgs::SetModelState objstate = publish( end_pos_x, end_pos_y, height, angle);
            client.call(objstate);
            loop_rate.sleep();
        }

        //3
        for(double x= -half_wide; x<=half_wide; x+=step){
            gazebo_msgs::SetModelState objstate = publish( x, -half_length, height, 0.0);
            client.call(objstate);
            loop_rate.sleep();
        }
        start_angle=0, end_angle=M_PI-backAngle, end_pos_x=half_wide, end_pos_y=-half_length;
        for(double angle=start_angle; angle<=end_angle; angle+=step){
            gazebo_msgs::SetModelState objstate = publish( end_pos_x, end_pos_y, height, angle);
            client.call(objstate);
            loop_rate.sleep();
        }
        start_angle=M_PI-backAngle, end_angle=M_PI*0.5, end_pos_x=half_wide, end_pos_y=-half_length;
        for(double angle=start_angle; angle>=end_angle; angle-=step){
            gazebo_msgs::SetModelState objstate = publish( end_pos_x, end_pos_y, height, angle);
            client.call(objstate);
            loop_rate.sleep();
        }

        //4
        for(double y= -half_length; y<=half_length; y+=step){
            gazebo_msgs::SetModelState objstate = publish( half_wide, y, height, M_PI*0.5);
            client.call(objstate);
            loop_rate.sleep();
        }
        start_angle=M_PI*0.5, end_angle=M_PI*1.5-backAngle, end_pos_x=half_wide, end_pos_y=half_length;
        for(double angle=start_angle; angle<=end_angle; angle+=step){
            gazebo_msgs::SetModelState objstate = publish( end_pos_x, end_pos_y, height, angle);
            client.call(objstate);
            loop_rate.sleep();
        }
        start_angle=M_PI*1.5-backAngle, end_angle=M_PI, end_pos_x=half_wide, end_pos_y=half_length;
        for(double angle=start_angle; angle>=end_angle; angle-=step){
            gazebo_msgs::SetModelState objstate = publish( end_pos_x, end_pos_y, height, angle);
            client.call(objstate);
            loop_rate.sleep();
        }


        //ros::spinOnce();
    }
    ROS_INFO("end service");
    return 0;
}
