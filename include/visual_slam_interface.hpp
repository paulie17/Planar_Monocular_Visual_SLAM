#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
// #include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <eigen_conversions/eigen_msg.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// #include <cameraManager.hpp>
#include <condensed_optimizer.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace planar_monocular_slam{

    class FrameProc{
        //clas that processes frames
        public:
        
        FrameProc(ros::NodeHandle* nodehandle);

        void addFrame(const sensor_msgs::ImageConstPtr& msg,const nav_msgs::Odometry::ConstPtr& odometry);
        
        void visualizeMap();

        void visualizeTrajectory();

        private:
        ros::NodeHandle nh_;

        ros::Publisher marker_pub_; // publish 3d map points for visualization in rviz

        ros::Publisher odom_path_pub_; // publisher of path according to odometry measurements
        ros::Publisher opt_path_pub_; // publisher of optimized path

        message_filters::Subscriber<sensor_msgs::Image> image_sub_;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry > MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;

        CameraManager cams_;
        condensed_optimizer opt_;

        nav_msgs::Path odom_path_;
    }; // class FrameProc


} // namespace visual_slam