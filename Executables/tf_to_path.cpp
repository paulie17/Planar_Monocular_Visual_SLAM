#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>


int main(int argc, char** argv){

    ros::init(argc, argv, "tf_listener");
    ros::NodeHandle n;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Publisher gt_path_pub = n.advertise<nav_msgs::Path>("gt_path", 10);
    nav_msgs::Path gt_path;
    gt_path.header.stamp = ros::Time::now();
    gt_path.header.frame_id = "camera_odom_frame";
    ros::Rate rate(10.0);
    while (n.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
        transformStamped = tfBuffer.lookupTransform("camera_odom_frame", "camera_pose_frame",
                                ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        
        geometry_msgs::PoseStamped pose;
        // transformStamped.transform.rotation
        pose.header.stamp = transformStamped.header.stamp;
        pose.header.frame_id = "camera_pose_frame";
        pose.pose.position.x = 0.0; 
        pose.pose.position.z = 0.0;    
        pose.pose.position.y = 0.0;  
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;

        tf2::doTransform(pose,pose,transformStamped);         
        gt_path.poses.push_back(pose);
        gt_path_pub.publish(gt_path);

        rate.sleep();
    }

    return 0;
}