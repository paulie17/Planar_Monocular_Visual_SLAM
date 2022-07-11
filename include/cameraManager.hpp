#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <utils.hpp>
#include <Map.hpp>

#include <g2otypes.hpp>

namespace planar_monocular_slam{    

    struct Camera{

        typedef std::shared_ptr<Camera> Ptr;
        typedef std::shared_ptr<const Camera> ConstPtr;

        int seq;
        bool condensed_flag = false;
        cv::Mat dscs;
        std::vector<cv::Point2f> kpts;
        Eigen::Isometry2d robot_pose; // robot pose from odometry when the frame has been captured in SE(2)
        Eigen::Vector3d odom_displ; // odometry displacement wrt previous keyframe
        std::unordered_map<unsigned,MapPoint::Ptr> observed_points; // map element: kpt_id->MapPoint
        Eigen3_4d p_matrix; //projection_matrix (world to pixel)
        std::vector<unsigned> unassigned_kpts; // vector with indices of unassigned keypoints

        Camera(cv::Mat& descriptors,std::vector<cv::Point2f>& keypoints,Eigen::Isometry2d& pose, Eigen::Matrix<double, 3, 4>& projection):
            dscs(descriptors),
            kpts(keypoints),
            robot_pose(pose),
            p_matrix(projection)
            {}

    }; // struct Camera

    class CameraManager{
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        CameraManager():            
            map_ ( new world_Map ),
            matcher_ ( new cv::flann::LshIndexParams ( 5,10,2 ))
        {
            orb_ = cv::ORB::create(MAX_FEATURES_);
            // matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
            clahe_ = cv::createCLAHE();
            clahe_->setClipLimit(4);
            
        };

        int n_of_cams(){
            return camera_vector_.size();
        };
        void addCamera(cv::Mat& image,Eigen::Isometry2d& pose);
        void addCameraMatrix(const Eigen3_4d& mat){
            P_ = mat;
        }
        void addExtrinsics(const Eigen::Matrix4d cam_to_robot){
            extrinsics_ = cam_to_robot;
        }
        void addImageSize(double height, double width){
            height_ = height; 
            width_ = width; 
        }

        void filterMatches(const std::vector<std::vector<cv::DMatch> >& matches_feats,std::vector<cv::DMatch>& matches); // Filter matches between features using Lowe's 
                                                                                                                         // ratio test

        bool checkTransform(const cv::Mat& image); // check whether there is a significant parallax (Homography vs Fundamental matrix inliers)

        bool checkTransform(const geometry_msgs::Pose& odom_pose, const cv::Mat& image); // check whether the robot has translated significantly from the last keyframe

        Eigen::Vector3d LinearLSTriangulation(  const cv::Point2f& img_pt1,       // image point (u,v)
                                                const Eigen3_4d& proj_mat1,       //camera 1 matrix
                                                const cv::Point2f& img_pt2,      //image point in 2nd camera
                                                const Eigen3_4d& proj_mat2       //camera 2 matrix
                                            );
        bool isInFrame ( const Eigen::Vector3d& pt_world );

        void matchNewKeypoints();   // match features of the last added camera frame with its previous, with candidate map point and with the unassigned keypoints
                                    // from the second last added frame.

        void fullBA(int iterations, bool verbose); // do full Bundle Adjustment with all points found and full trajectory

        void pgo(int iterations, bool verbose); // do full Bundle Adjustment with all points found and full trajectory

        world_Map::ConstPtr map(){
            return std::const_pointer_cast<const world_Map>(map_);
        }

        Camera::ConstPtr last_keyframe(){
            return std::const_pointer_cast<const Camera>(camera_vector_.back());
        }

        std::vector<Camera::Ptr>& keyframes_vector_(){
            return camera_vector_;
        }

        private:

        std::vector<Camera::Ptr> camera_vector_;
        world_Map::Ptr map_;          
        cv::Ptr<cv::CLAHE> clahe_;
        cv::Ptr<cv::ORB> orb_;
        cv::FlannBasedMatcher matcher_;
        cv::Mat latest_inliers_;

        Eigen3_4d P_;
        Eigen::Matrix4d extrinsics_;
        double height_;
        double width_;


        const int MAX_FEATURES_ = 250;
        unsigned long last_triangulated_point = 0;

    }; // class CameraManager

} // namespace visual_slam  