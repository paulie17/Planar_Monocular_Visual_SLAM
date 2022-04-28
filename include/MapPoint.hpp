#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <opencv2/opencv.hpp>

namespace visual_slam{

    struct Camera;

    struct MapPoint{


        typedef std::shared_ptr<MapPoint> Ptr;

        unsigned long id;
        Eigen::Vector3d p_world; // world coordinates of the point
        cv::Mat descriptor; // descriptor of the point
        std::list<Camera*>    observed_frames; // list of frames where the point has been observed
        
        MapPoint();
        MapPoint( 
            unsigned long id, 
            const Eigen::Vector3d& position, 
            const cv::Mat& descriptor,
            Camera* frame             
        );

        void insert_frame(Camera* frame){
            observed_frames.push_back(frame);
        };

    }; // struct MapPoint


} // namespace visual_slam