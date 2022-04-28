#include <visual_slam_interface.hpp>

using namespace visual_slam;
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  
  ros::NodeHandle nh;

  FrameProc FP(&nh);

  ros::spin();
  return 0;
} 