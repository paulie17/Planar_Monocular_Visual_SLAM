#include <visual_slam_interface.hpp>

using namespace planar_monocular_slam;
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planar_monocular_slam_node");
  string param;
  ros::NodeHandle nh("~");
  nh.getParam("param", param);

  FrameProc FP(&nh,param);

  ros::spin();
  return 0;
} 