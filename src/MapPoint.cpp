#include <MapPoint.hpp>

namespace planar_monocular_slam{

    MapPoint::MapPoint()
    : id(-1), p_world(Eigen::Vector3d(0,0,0)) 
    {}

    MapPoint::MapPoint ( long unsigned int id, const Eigen::Vector3d& position, const cv::Mat& descriptor , Camera* frame)
    : id(id), p_world(position), descriptor(descriptor)
    {
        observed_frames.push_back(frame);
    }

}