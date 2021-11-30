/*
 * 作者：李阳
 * 
 * 本文件是main函数所在文件，能够在ROS的平台上实现消息的订阅和发布，实现的是一个具有前端
 * 和后端的视觉SLAM功能。
 * 单元测试使用的是catch2: https://github.com/catchorg/Catch2
 * 日志使用的是Google的glog
 */

#include <sstream>
#include <string>
#include <glog/logging.h>
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include "gflags.h"
#include "visual_odometry.h"

// #define _GLIBCXX_USE_CXX11_ABI 0

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicyT;

void images_callback(const sensor_msgs::ImageConstPtr msg1, const sensor_msgs::ImageConstPtr msg2);



int main(int argc, char **argv) {
  //初始化ROS
  ros::init(argc, argv, "my_slam");
  ros::NodeHandle n;
  //gflag
  google::ParseCommandLineFlags(&argc, &argv, true);
  //Glog
  FLAGS_log_dir = FLAGS_myslamlog_dir;
  //std::string leftTopic("/zed2i/zed_node/left_raw/image_raw_color");
  //std::string rightTopic("/zed2i/zed_node/right_raw/image_raw_color");
  
  google::InitGoogleLogging(argv[0]);

  LOG(INFO) << "App: my_slam is running which is a ROS node.";


  //初始化VO
  std::string conf("/home/minerva/minerva/Dai_Yang/catkin_ws/src/my_slam/conf/myslam.conf");
  my_slam::VisualOdometry::Ptr vo(new my_slam::VisualOdometry(conf));
  vo->Init();

  //ROS接口采集图像数据
  message_filters::Subscriber<sensor_msgs::Image>* left_img_sub_;
  message_filters::Subscriber<sensor_msgs::Image>* right_img_sub_;
  left_img_sub_ 
      = new message_filters::Subscriber<sensor_msgs::Image>(n,FLAGS_leftImgTopic, 1);
  right_img_sub_ 
      = new message_filters::Subscriber<sensor_msgs::Image>(n,FLAGS_rightImgTopic, 1);

  message_filters::Synchronizer<SyncPolicyT> *image_synchronizer_ = 
      new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *left_img_sub_, *right_img_sub_);
  //回调函数，用于接收ROS的数据
  image_synchronizer_->registerCallback(
      boost::bind(&my_slam::VisualOdometry::AddRosData, vo, _1, _2)); 

  //启动视觉里程计
  vo->Run();

  //启动ROS
  ros::spin();

  return 0;
}






