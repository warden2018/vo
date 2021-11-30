#include "visual_odometry.h"
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "gflags.h"


namespace my_slam{

VisualOdometry::VisualOdometry(std::string& conf_dir)
  : configFile_(conf_dir),
    vo_run_(false) {

}

//实例化Tracking,Backend,Viewer;Tracking是主线程。
bool VisualOdometry::Init() {
    //std::cout << "FLAGS_left_camera_params: " << FLAGS_left_camera_params;
    //std::cout << "FLAGS_right_camera_params: " << FLAGS_right_camera_params;

    left_camera_ = Camera::CreateCamera(FLAGS_left_camera_params); 
    right_camera_ = Camera::CreateCamera(FLAGS_right_camera_params); 
    left_camera_->Init(Camera::CameraSide::LEFT);
    right_camera_->Init(Camera::CameraSide::RIGHT);

    //创建地图，前端跟踪，后端优化实例
    tracking_ = Tracking::Ptr(new Tracking);
    map_ = Map::Ptr(new Map);
    backend_ = Backend::Ptr(new Backend);
    viewer_ = Viewer::Ptr(new Viewer);
    //设置相互持有的指针
    //跟踪
    tracking_->SetBackend(backend_);
    tracking_->SetCameras(left_camera_, right_camera_);
    tracking_->SetMap(map_);
    tracking_->SetViewer(viewer_);
    
    //后端
    backend_->SetCameras(left_camera_, right_camera_);
    backend_->SetMap(map_);
    //可视化
    viewer_->SetMap(map_);
    
    return true;
}

void VisualOdometry::Run() {
    vo_run_ = true;
}


void VisualOdometry::AddRosData(const sensor_msgs::ImageConstPtr msg1, const sensor_msgs::ImageConstPtr msg2) {
    //LOG(INFO) << "Image callback recv. images. msg1: " << msg1->header.frame_id << " msg2: " << msg2->header.frame_id;
    LOG(INFO) << "VO is on or not: " << vo_run_;
    if(!vo_run_) {
        return;
    }
    //vo_run_==true,开始处理图像
    cv_bridge::CvImagePtr cv_ptr1;
    cv_bridge::CvImagePtr cv_ptr2;
    try{
        cv_ptr1 = cv_bridge::toCvCopy(msg1,sensor_msgs::image_encodings::BGR8);
        cv_ptr2 = cv_bridge::toCvCopy(msg2,sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        LOG(ERROR) << "cv_bridge exception: " << e.what();
    }
    //ROS 数据转换为OpenCV的Mat
    cv::Mat image_left, image_right;
    image_left = cv_ptr1->image;
    image_right = cv_ptr2->image;
    double time_stamp = msg2->header.stamp.sec + msg2->header.stamp.nsec / 1e9;
    tracking_->AddImage(time_stamp,image_left,image_right);
}

} // namespace my_slam