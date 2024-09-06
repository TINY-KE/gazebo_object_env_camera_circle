#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
// #include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/sensors.hh>
// #include <gazebo/math/gzmathc.hh>
// #include <gazebo/gui/gui.hh>
#include <ignition/math/Vector3.hh>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
ros::Publisher publisher_object;


geometry_msgs::Point corner_to_marker(geometry_msgs::Point& oldp, ignition::math::Pose3d& pose){
        geometry_msgs::Point newp;
        newp.x = oldp.x + pose.Pos().X() ;
        newp.y = oldp.y + pose.Pos().Y() ;
        newp.z = oldp.z + pose.Pos().Z() ;
        return newp;
}



int main(int argc, char **argv)
{
    ros::init ( argc, argv, "gazebo_world_parser" );
    ros::NodeHandle nh;
    publisher_object = nh.advertise<visualization_msgs::Marker>("objectmap", 1000);
    // Load Gazebo
    // gazebo::client::setup(argc, argv);
    // gazebo::setupClient(argc, argv);
    // gazebo::setupServer
    // gazebo::client::setup();
    // Load Gazebo
    // gazebo::load(argc, argv);

    // // Initialize Gazebo
    // gazebo::init();

    // Load Gazebo
    // gazebo::set
    // gazebo::load(argc, argv);
    
    // Load gazebo
    gazebo::setupServer(argc, argv);

    // Create a world and get the models
    // gazebo::physics::WorldPtr world = gazebo::physics::get_world();
    gazebo::physics::WorldPtr world = gazebo::loadWorld("/home/zhjd/workspace/ws_huchunxu/src/ros_exploring/my_mobilearm/ASLAM_gazebo_world/world/nine_highdesk.world");//worlds/empty.world
    gazebo::physics::Model_V models = world->Models();
    ros::Rate rate(0.1);
    // // Loop through each model and get its pose and size
    while (nh.ok()){    
        for (auto model : models) {
            // Get model pose
            ignition::math::Pose3d pose = model->WorldPose();
            double x = pose.Pos().X();
            double y = pose.Pos().Y();
            double z = pose.Pos().Z();  
            
            double yaw = pose.Rot().Yaw();  

            int id = 0;

            // Get model size
            double width ;
            double height ;
            double depth ;
            gazebo::physics::Link_V links = model->GetLinks();
            for (auto link : links) {
                gazebo::physics::Collision_V collisions = link->GetCollisions();
                int i = 0 ;
                for (auto collision : collisions) {
                    ignition::math::Box box = collision->BoundingBox();
                    width = box.XLength();
                    height = box.YLength();
                    depth = box.ZLength();

                    // Print model information
                    std::cout << "Model num:"<< i<< " ,name: " << model->GetName() << std::endl;
                    std::cout << "Position: x=" << x << " y=" << y << " z=" << z << std::endl;
                    std::cout << "Orientation: yaw=" << yaw << std::endl;
                    std::cout << "Size: width=" << width << " height=" << height << " depth=" << depth << std::endl;
                    std::cout << std::endl;

                    

                    i++;
                }
            }

            //publish rviz 
            visualization_msgs::Marker marker;
            marker.id = id++;//++object_id_init;//object_id_init + i;
            float mObject_Duration=1;
            // marker.lifetime = ros::Duration(mObject_Duration);
            marker.header.frame_id= "map";
            marker.header.stamp=ros::Time::now();
            marker.type = visualization_msgs::Marker::LINE_LIST; //LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.r = 255.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
            marker.scale.x = 0.01;
            //     8------7
            //    /|     /|
            //   / |    / |
            //  5------6  |
            //  |  4---|--3
            //  | /    | /
            //  1------2
            // lenth ：corner_2[0] - corner_1[0]
            // width ：corner_2[1] - corner_3[1]
            // height：corner_2[2] - corner_6[2]
            double width_half = width/2.0;
            double height_half = height/2.0;
            double depth_half = depth/2.0;

            geometry_msgs::Point p1;   p1.x = -1*height_half;   p1.y = width_half;      p1.z = -1*depth_half; 
            geometry_msgs::Point p2;   p2.x = -1*height_half;   p2.y = -1*width_half;   p2.z = -1*depth_half; 
            geometry_msgs::Point p3;   p3.x = height_half;      p3.y = -1*width_half;   p3.z = -1*depth_half; 
            geometry_msgs::Point p4;   p4.x = height_half;      p4.y = width_half;      p4.z = -1*depth_half; 
            geometry_msgs::Point p5;   p5.x = -1*height_half;   p5.y = width_half;      p5.z = 1*depth_half; 
            geometry_msgs::Point p6;   p6.x = -1*height_half;   p6.y = -1*width_half;   p6.z = 1*depth_half; 
            geometry_msgs::Point p7;   p7.x = height_half;      p7.y = -1*width_half;   p7.z = 1*depth_half; 
            geometry_msgs::Point p8;   p8.x = height_half;      p8.y = width_half;      p8.z = 1*depth_half; 
            
            
            
            marker.points.push_back(corner_to_marker(p1, pose));
            marker.points.push_back(corner_to_marker(p2, pose));
            marker.points.push_back(corner_to_marker(p2, pose));
            marker.points.push_back(corner_to_marker(p3, pose));
            marker.points.push_back(corner_to_marker(p3, pose));
            marker.points.push_back(corner_to_marker(p4, pose));
            marker.points.push_back(corner_to_marker(p4, pose));
            marker.points.push_back(corner_to_marker(p1, pose));

            marker.points.push_back(corner_to_marker(p5, pose));
            marker.points.push_back(corner_to_marker(p1, pose));
            marker.points.push_back(corner_to_marker(p6, pose));
            marker.points.push_back(corner_to_marker(p2, pose));
            marker.points.push_back(corner_to_marker(p7, pose));
            marker.points.push_back(corner_to_marker(p3, pose));
            marker.points.push_back(corner_to_marker(p8, pose));
            marker.points.push_back(corner_to_marker(p4, pose));

            marker.points.push_back(corner_to_marker(p5, pose));
            marker.points.push_back(corner_to_marker(p6, pose));
            marker.points.push_back(corner_to_marker(p6, pose));
            marker.points.push_back(corner_to_marker(p7, pose));
            marker.points.push_back(corner_to_marker(p7, pose));
            marker.points.push_back(corner_to_marker(p8, pose));
            marker.points.push_back(corner_to_marker(p8, pose));
            marker.points.push_back(corner_to_marker(p5, pose));

            publisher_object.publish(marker);
            // publish rviz end
            rate.sleep();
        }  
    }

    // Clean up
    gazebo::shutdown();
    return 0;
}
