#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "obindex2/binary_index.h"


static const std::string OPENCV_WINDOW = "Image window";
const int MAX_FEATURES = 500;
int counter = 0;

void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info, 
              const nav_msgs::Odometry::ConstPtr& odometry)
{
  ROS_INFO("The synchronized callback is working!!");
}

class ImageProc
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Ptr<cv::CLAHE> clahe;
  cv::Ptr<cv::Feature2D> detector;
  obindex2::ImageIndex index_;

public:
  ImageProc(ros::NodeHandle* nodehandle)
    : nh_(*nodehandle),it_(nh_),index_(16, 150, 4, obindex2::MERGE_POLICY_AND, true) 
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ImageProc::ORBfeatures, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    
    clahe = cv::createCLAHE();
    clahe->setClipLimit(4);

    detector = cv::ORB::create(MAX_FEATURES);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageProc()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void ORBfeatures(const sensor_msgs::ImageConstPtr& msg)
  {
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
    cv::Mat clahe_output;

    if (counter == 0){
      // Adding image 0
      // Detecting and describing keypoints
      std::vector<cv::KeyPoint> kps0;
      cv::Mat dscs0;
      cv::Mat image0 = cv_ptr->image;
      clahe->apply(image0,clahe_output);
      detector->detect(clahe_output, kps0);
      cv::KeyPointsFilter::retainBest(kps0, MAX_FEATURES/2);
      detector->compute(clahe_output, kps0, dscs0);
      // Adding the image to the index.
      index_.addImage(0, kps0, dscs0);
      counter ++;
    }else{
      cv::Mat img = cv_ptr->image;
      std::vector<cv::KeyPoint> kps;
      cv::Mat dscs;
      clahe->apply(img,clahe_output);
      detector->detect(clahe_output, kps);
      cv::KeyPointsFilter::retainBest(kps, MAX_FEATURES/2);
      detector->compute(clahe_output, kps, dscs);
      // Matching the descriptors
      std::vector<std::vector<cv::DMatch> > matches_feats;
      // Searching the query descriptors against the features
      index_.searchDescriptors(dscs, &matches_feats, 2, 64);
      // Filtering matches according to the ratio test
      std::vector<cv::DMatch> matches;
      for (unsigned m = 0; m < matches_feats.size(); m++) {
        if (matches_feats[m][0].distance < matches_feats[m][1].distance * 0.8) {
          matches.push_back(matches_feats[m][0]);
        }
      }
      std::vector<obindex2::ImageMatch> image_matches;   
      // We look for similar images according to the good matches found
      index_.searchImages(dscs, matches, &image_matches);
      ROS_INFO_STREAM("Current frame id: " << counter); 
      int most_recent_candidate;
    
      // Showing results
      for (int j = 0; j < std::min(5, static_cast<int>(image_matches.size()));
                                                                            j++) {                                                                             
        ROS_INFO_STREAM("Cand: " << image_matches[j].image_id <<  ", " <<
                    "Score: " << image_matches[j].score << std::endl);
      }     
      
      // Updating the index
      // Matched descriptors are used to update the index and the remaining ones
      // are added as new visual words
      index_.addImage(counter, kps, dscs, matches);
      ROS_INFO_STREAM("Total features found in the image: " <<
                                            kps.size() << std::endl);
      ROS_INFO_STREAM("Total matches found against the index: " <<
                                            matches.size() << std::endl);
      ROS_INFO_STREAM("Total index size AFTER UPDATE: " <<
                                            index_.numDescriptors() << std::endl;);
      // Printing percentage of matchings with the previous image
      std::unordered_map<unsigned, obindex2::PointMatches> point_matches;
      index_.getMatchings(kps, matches, &point_matches);
      obindex2::PointMatches pmatches = point_matches[counter-1];

      ROS_INFO_STREAM("Percentage of matchings with the previous image: " << float(float(pmatches.query.size()))/float(kps.size()));   
      cv::Mat outimg;
      cv::drawKeypoints( cv_ptr->image, kps, outimg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );

      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, outimg);
      cv::waitKey(3);

      // Output modified video stream
      image_pub_.publish(cv_ptr->toImageMsg());

      // Reindexing features every 250 images
      if (counter % 250 == 0) {
        index_.rebuild();
      }

      counter++;
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  
  ros::NodeHandle nh;

  ImageProc IP(&nh);

  // message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/image_raw", 1);
  // message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "/camera_info", 1);
  // message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh,"/odom",1);
  
  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, nav_msgs::Odometry > MySyncPolicy;
  // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub,info_sub,odom_sub); 
  // sync.registerCallback(boost::bind(&callback, _1, _2, _3)); 

  ros::spin();
  return 0;
} 
