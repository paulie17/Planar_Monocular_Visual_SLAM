#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <queue>
#include <eigen_conversions/eigen_msg.h>
#include "obindex2/binary_index.h"

namespace visual_slam{

    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    struct video_frame{

        std::vector<cv::KeyPoint> Keypoints;
        cv::Mat KeypointsDescriptors;
        std::vector<int> KeypointsIDs;
        std::vector<cv::DMatch> KeypointsIDs_matches;
        std::map<int,int> Kpts_pcl_map;
        // Vector6d odom;
        Eigen::Isometry3d pose;

    }
    const int MAX_FEATURES = 500;
    class FrameCapture{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            FrameCapture(ros::NodeHandle* nodehandle,int frames):
            nh_(*nodehandle),
            max_frames(frames),
            counter(0),
            index_(16, 150, 4, obindex2::MERGE_POLICY_AND, true)
            {
                image_sub_.subscribe(nh_, "/camera/image_raw", 1);
                info_sub_.subscribe(nh_,  "/camera_info", 1);
                odom_sub_.subscribe(nh_,"/odom",1);
                sync_.reset(new Sync(MySyncPolicy(10),image_sub_,info_sub_,odom_sub_));   
                sync_->registerCallback(boost::bind(&FrameCapture::addFrame, this, _1, _2, _3));

                cam_to_robot_ << 0, 0,1,0,
                                 -1,0,0,0,
                                 0,-1,0,0,
                                 0, 0,0,1;
            }

            void addFrame(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info, 
                                const nav_msgs::Odometry::ConstPtr& odometry)
            {
                ROS_INFO("The synchronized callback is working!!");                
                // declare video_frame object
                video_frame frame;

                // convert odom to Eigen and add it to the object
                // Vector6d odom_eigen;
                Eigen::Isometry3d pose_eigen;
                // twistMsgToEigen(odometry->twist.twist,odom_eigen);
                poseMsgToEigen(odometry->pose.pose,pose_eigen);
                // frame.odom = odom_eigen;
                frame.pose = pose_eigen;

                // detect orb keypoints and add to object
                cv_bridge::CvImagePtr cv_ptr;
                try
                {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                }
                catch (cv_bridge::Exception& e)
                {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
                }

                std::vector<cv::KeyPoint> objectKeypoints;
                cv::Ptr<cv::Feature2D> detector = cv::ORB::create(MAX_FEATURES);
                cv::KeyPointsFilter::retainBest(kps, MAX_FEATURES/2);

                detector->detect(cv_ptr->image, objectKeypoints);
                frame.Keypoints = objectKeypoints;

                // compute orb keypoints descriptors and add to object
                cv::Mat descriptors;
                detector->compute(cv_ptr->image,objectKeypoints,descriptors);
                frame.KeypointsDescriptors = descriptors;                

                // assign IDs to keypoints
                if (counter == 0){
                    index_.addImage(frames_.size()-1, objectKeypoints, descriptors);                    
                    std::vector<cv::DMatch> matches;
                    find_matches(descriptors,&matches);
                    frames.KeypointsIDs = matches;
                    counter ++;
                }
                else{
                    
                    std::vector<cv::DMatch> matches;
                    find_matches(descriptors,&matches);
                    // Updating the index
                    // Matched descriptors are used to update the index and the remaining ones
                    // are added as new visual words
                    index_.addImage(counter, objectKeypoints, descriptors, matches);

                    find_matches(descriptors,&matches);

                    frames.KeypointsIDs = matches;
                    counter ++;
                }

                // add current frame to queue
                frames_.push(frame);
                if (frames_.size() > max_frames_){
                    frames_.pop();
                }

                // Update Keypoints IDs
                update_kptsIDs();

                // triangulate/optimize 

                //#1 when a new ID is added, if there are at least two observations, triangulate and add to the pointcloud
                //#2 when the triangulation is completed add a map from keypoint index to pointcloud index (this will not depend on the database ID).
                //#3 optimize current guess doing bundle adjustment.

                // publish to pointcloud
            
            }


        private:            
            ros::NodeHandle nh_;
            std::queue<video_frame> frames_;
            const int max_frames_;
            int counter_;
            obindex2::ImageIndex index_;

            Eigen::Matrix4d cam_to_robot_;
            Eigen::MatrixXd P_;
            
            // Landmarks Pointcloud Vector3dVector landmarks_;
            // Trajectory points Vector3dVector trajectory_
            // BA solver ba_solver_;

            message_filters::Subscriber<sensor_msgs::Image> image_sub_;
            message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
            message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;

            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, nav_msgs::Odometry > MySyncPolicy;
            typedef Synchronizer<MySyncPolicy> Sync;
            boost::shared_ptr<Sync> sync_;

            void filterMatches(const std::vector<std::vector<cv::DMatch> >& matches_feats,std::vector<cv::DMatch>* matches);
            void find_matches(const cv::Mat dscs,std::vector<cv::DMatch>* matches);
            void convert_matches(video_frame& frame);
            void update_kptsIDs();
    }


}