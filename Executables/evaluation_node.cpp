#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <cmath>

class Evaluator{

    ros::NodeHandle nh_;
    ros::Subscriber gt_path_sub_;
    ros::Subscriber opt_path_sub_;

    nav_msgs::Path groundtruth_;
    nav_msgs::Path optimized_;

    public:
    Evaluator(ros::NodeHandle* nodehandle):
    nh_(*nodehandle)
    {
        gt_path_sub_ = nh_.subscribe<nav_msgs::Path>("/gt_path",5,&Evaluator::gt_callback,this);
        opt_path_sub_ = nh_.subscribe<nav_msgs::Path>("/opt_path",5,&Evaluator::opt_callback,this);
    }

    void gt_callback(const nav_msgs::PathConstPtr& gt){
        groundtruth_ = *gt;
    };

    void opt_callback(const nav_msgs::PathConstPtr& opt){
        optimized_ = *opt;
        rms_error();
    };

    void rms_error(){

        double rms_error = 0.0;
        double N = 0.0;
        int64_t previous_difference;
        ros::Duration stamps_difference;

        for (int i = 0; i < optimized_.poses.size(); i++){

            previous_difference = 1e12;
            ros::Time stamp_i = optimized_.poses[i].header.stamp;
            for (int j = 0; j < groundtruth_.poses.size(); j++){
                stamps_difference = stamp_i - groundtruth_.poses[j].header.stamp;
                // ROS_INFO_STREAM(stamps_difference.toSec());
                
                if(abs(stamps_difference.toNSec()) > previous_difference){
                    rms_error += pow((optimized_.poses[i].pose.position.x - groundtruth_.poses[j-1].pose.position.x),2) + pow((optimized_.poses[i].pose.position.y - groundtruth_.poses[j-1].pose.position.y),2);
                    N = N+1;
                    break;
                }
                previous_difference = abs(stamps_difference.toNSec());
            }

        }
        rms_error /= N;
        rms_error = sqrt(rms_error);
        ROS_INFO_STREAM("RMS Error of optimized trajectory: " << rms_error);

    }

};


int main(int argc, char** argv){

    ros::init(argc, argv, "evaluation");
    ros::NodeHandle nh;
    Evaluator ev(&nh);
    ros::spin();

    return 0;
}
