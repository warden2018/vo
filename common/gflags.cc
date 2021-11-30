//
// Created by yang on 2021/3/29.
//
#include "gflags.h"
//下面是视觉SLAM十四讲中前端光流跟踪的参数
DEFINE_string(left_img_file, "../left.png","left image file dir.");
DEFINE_string(disparity_file, "../disparity.png","Disparity image of the left and right image.");
DEFINE_int32(nPoints,2000,"Sample number in an image.");
DEFINE_int32(nBorder,20,"Border size of sample points in image 1. To avoid too close to the border.");
DEFINE_int32(halfPatchSize,3,"Half patch size for an interest patch in image.");
DEFINE_int32(interations,10,"Iterations for the GN method.");
DEFINE_int32(img_number,5,"Image number of the camera2……");
DEFINE_int32(pyramids,4,"How many pyramids in DirectPoseEstimationMultiLayer.");
DEFINE_double(pyr_scale,0.5,"The scale between the neighbor pyramid images.");

//glog
DEFINE_string(myslamlog_dir,"/tmp","The dir of the log files.");
DEFINE_string(left_camera_params,"/home/minerva/minerva/Dai_Yang/catkin_ws/src/my_slam/params/zed2i_left_2k.yaml","");
DEFINE_string(right_camera_params,"/home/minerva/minerva/Dai_Yang/catkin_ws/src/my_slam/params/zed2i_right_2k.yaml","");
//Tracking参数
DEFINE_int32(nFeatures,200,"");
DEFINE_int32(nFeatureIinit,100,"");
DEFINE_int32(nFeaturesTracking,50,"");
DEFINE_int32(nFeaturesTrackingBad,20,"");
DEFINE_int32(nFeaturesKeyframe,80,"");
DEFINE_double(gftt_qualityLevel,0.01,"角点可以接受的最小特征值");
DEFINE_double(gftt_minDistance,1,"关键点的最小距离");
DEFINE_double(maskNeigbourPixels,20,"新关键帧里面，需要覆盖已经和前一帧匹配了的特征点周围多少像素的正方形");

//backend optimization params
DEFINE_bool(useDense,false,"True for using LinearSolverDense to solve BA, false for using LinearSolverCholmod or Eigen.");
DEFINE_double(2Dhuber95,5.99,"鲁棒核函数的参数，2维向量，卡方分布p值0.05");
DEFINE_double(3Dhuber95,7.81,"鲁棒核函数的参数，3维向量，卡方分布p值0.05");

//ROS Interfaces
DEFINE_string(leftImgTopic,"/zed2i/zed_node/left_raw/image_raw_color","Unrectified left color image.");
DEFINE_string(rightImgTopic,"/zed2i/zed_node/right_raw/image_raw_color","Unrectified right color image.");
DEFINE_int32(queue_size,10,"ROS image queue size.");

//Map 
DEFINE_int32(activateKF,200,"激活的KF的最大数量");

//vo config
DEFINE_string(vo_config,"/home/minerva/minerva/Dai_Yang/catkin_ws/src/my_slam/conf/myslam.conf","vo config file.");

//ORB
DEFINE_string(LeftORBParamsFile,"/home/minerva/minerva/Dai_Yang/catkin_ws/src/my_slam/params/ORB_extractor_left.yaml","");
DEFINE_string(RightORBParamsFile,"/home/minerva/minerva/Dai_Yang/catkin_ws/src/my_slam/params/ORB_extractor_right.yaml","");