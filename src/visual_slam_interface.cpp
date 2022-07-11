#include <visual_slam_interface.hpp>

namespace planar_monocular_slam{

    FrameProc::FrameProc(ros::NodeHandle* nodehandle,std::string opt_type_string):
            nh_(*nodehandle),
            cams_(),
            opt_(cams_.keyframes_vector_(),cams_.map()),
            opt_type_(opt_type_string)
        {
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("map_points", 1);

            odom_path_pub_ = nh_.advertise<nav_msgs::Path>("/odom_path", 1);
            opt_path_pub_ = nh_.advertise<nav_msgs::Path>("/opt_path", 1);
            ba_server = nh_.advertiseService("request_ba", &FrameProc::ba_service,this);

            image_sub_.subscribe(nh_, "/camera/image_raw", 1);
            odom_sub_.subscribe(nh_,"/odom",1);
            sync_.reset(new Sync(MySyncPolicy(10),image_sub_,odom_sub_));   
            sync_->registerCallback(boost::bind(&FrameProc::addFrame, this, _1, _2));

            odom_path_.header.stamp = ros::Time::now();
            odom_path_.header.frame_id = "my_frame";

            Eigen::Matrix4d cam_to_robot;
            cam_to_robot << 0, 0,1,0,
                            -1,0,0,0,
                            0,-1,0,0,
                            0, 0,0,1;
            cams_.addExtrinsics(cam_to_robot);

        }
        
    void FrameProc::visualizeMap(){

        visualization_msgs::Marker points_list;
        points_list.header.frame_id= "my_frame";
        points_list.header.stamp= ros::Time::now();
        points_list.ns= "points";
        points_list.action= visualization_msgs::Marker::ADD;
        points_list.pose.orientation.w= 1.0;

        points_list.id = 0;

        points_list.type = visualization_msgs::Marker::POINTS;

        // POINTS markers use x and y scale for width/height respectively
        points_list.scale.x = 0.05;
        points_list.scale.y = 0.05;

        // Points are green
        points_list.color.r = 1.0f;
        points_list.color.a = 1.0;

        // Create the vertices for the points and lines
        for (auto& allpoints: cams_.map()->map_points)
        {
        if(allpoints.second->observed_frames.size()>4){

            geometry_msgs::Point p;
            p.x = allpoints.second->p_world.x();
            p.y = allpoints.second->p_world.y();
            p.z = allpoints.second->p_world.z();

            points_list.points.push_back(p);
            }
        }

        marker_pub_.publish(points_list);
    }

    void FrameProc::visualize_condensed_Trajectory(){

        nav_msgs::Path opt_path_;
        opt_path_.header.stamp = ros::Time::now();
        opt_path_.header.frame_id = "my_frame";

        for (int i = 0; i < cams_.n_of_cams(); i++){
            if (cams_.keyframes_vector_().at(i)->condensed_flag){
            geometry_msgs::PoseStamped  pose;
            tf::poseEigenToMsg(t2t3d(cams_.keyframes_vector_().at(i)->robot_pose),pose.pose);
            pose.header.frame_id= "my_frame";
            pose.header.stamp= time_stamps_vector_[cams_.keyframes_vector_().at(i)->seq];
            opt_path_.poses.push_back(pose);
            }

        }

        opt_path_pub_.publish(opt_path_);
    }

    void FrameProc::visualize_full_Trajectory(){

        nav_msgs::Path opt_path_;
        opt_path_.header.stamp = ros::Time::now();
        opt_path_.header.frame_id = "my_frame";

        for (int i = 0; i < cams_.n_of_cams(); i++){
            geometry_msgs::PoseStamped  pose;
            tf::poseEigenToMsg(t2t3d(cams_.keyframes_vector_().at(i)->robot_pose),pose.pose);
            pose.header.frame_id= "my_frame";
            pose.header.stamp= time_stamps_vector_[cams_.keyframes_vector_().at(i)->seq];
            opt_path_.poses.push_back(pose);

        }

        opt_path_pub_.publish(opt_path_);
    }

    bool FrameProc::ba_service(     planar_monocular_slam_thesis::BARequest::Request &req,
                                    planar_monocular_slam_thesis::BARequest::Response &res){
        ROS_INFO("Starting full bundle adjustment: ");
        // cams_.fullBA(req.iterations,true);          
        cams_.pgo(req.iterations,true);      
        ROS_INFO("Completed full bundle adjustment with %d iterations.", req.iterations);
        visualizeMap();
        visualize_full_Trajectory();
        res.success = true;
        return true;
    }

    void FrameProc::addFrame(const sensor_msgs::ImageConstPtr& msg,const nav_msgs::Odometry::ConstPtr& odometry){

        cv_bridge::CvImagePtr cv_ptr;

        try
        {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }

        if (cams_.n_of_cams() == 0){

            boost::shared_ptr<sensor_msgs::CameraInfo const> shared_camera_infos;
            shared_camera_infos = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera_info",nh_);
            start_ = ros::WallTime::now();
            time_stamps_vector_.push_back(msg->header.stamp);
            // Eigen::Matrix3d K;
            Eigen::Matrix<double,3,4,Eigen::RowMajor> P( shared_camera_infos->P.data() );
            // K = P.block<3,3>(0,0);
            // std::cout << K ;
            Eigen::Isometry3d pose_eigen;
            Eigen::Isometry2d pose_eigen_2d;
            tf::poseMsgToEigen(odometry->pose.pose,pose_eigen);
            pose_eigen_2d = t3t2d(pose_eigen);
            cams_.addImageSize(double(shared_camera_infos->height), double(shared_camera_infos->width));
            cams_.addCameraMatrix(P);
            cams_.addCamera(cv_ptr->image,pose_eigen_2d);  

            if (!opt_type_.compare("condensed"))
                {
                    opt_.local_maps_manager();
                }         
            
            geometry_msgs::PoseStamped pose;

            pose.header.stamp = msg->header.stamp;
            pose.header.frame_id = "my_frame";
            pose.pose = odometry->pose.pose;            
            odom_path_.poses.push_back(pose);
            odom_path_pub_.publish(odom_path_);

            if (!opt_type_.compare("condensed"))
                {
                    // visualize_condensed_Trajectory();
                    visualize_full_Trajectory();
                }
            else if(!opt_type_.compare("full")){
                visualize_full_Trajectory();
            }
            
        }
        else{
            // if (cams_.checkTransform(cv_ptr->image)){
            if (cams_.checkTransform(odometry->pose.pose,cv_ptr->image)){    
                ROS_INFO("Adding keyframe!");

                geometry_msgs::PoseStamped pose;

                pose.header.stamp = msg->header.stamp;
                pose.header.frame_id = "my_frame";
                pose.pose = odometry->pose.pose;            
                odom_path_.poses.push_back(pose);
                odom_path_pub_.publish(odom_path_);    

                time_stamps_vector_.push_back(msg->header.stamp);

                Eigen::Isometry3d pose_eigen;
                Eigen::Isometry2d pose_eigen_2d;
                tf::poseMsgToEigen(odometry->pose.pose,pose_eigen);
                pose_eigen_2d = t3t2d(pose_eigen);
                cams_.addCamera(cv_ptr->image,pose_eigen_2d);

                if (!opt_type_.compare("condensed"))
                {
                    opt_.local_maps_manager();
                    // visualize_condensed_Trajectory();
                    visualize_full_Trajectory();
                }
                else if(!opt_type_.compare("full")){
                    cams_.fullBA(1,false);
                    visualize_full_Trajectory();
                }

                visualizeMap();

            } else {
                ROS_INFO("Not a keyframe.");
            }
            // ros::shutdown();
        }            
        ros::WallDuration till_now_ = ros::WallTime::now() - start_;
        ROS_INFO_STREAM("Duration till now: " << till_now_.toSec());
        ROS_INFO_STREAM("Number of keyframes added:" << cams_.n_of_cams());
        ROS_INFO_STREAM("Number of local_maps written in the condensed optimizers:" << opt_.n_of_local_maps());

    } // addFrame

} // namespace visual_slam