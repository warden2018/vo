#include "frame.h"
#include "camera.h"

namespace my_slam {

Frame::Frame(){}
Frame::Frame(long id, double ts, const Eigen::Isometry3d& pose, const Mat& leftImg,const Mat& rightImg)
    : id_(id),
      time_stamp_(ts),
      pose_(pose),
      left_image_(leftImg),
      right_image_(rightImg) {}

void Frame::SetPose(const Eigen::Isometry3d& pose) {
    std::unique_lock<std::mutex> lock(pose_mutex_);
    pose_ = pose;
}

const Eigen::Isometry3d& Frame::Pose() {
    std::unique_lock<std::mutex> lock(pose_mutex_);
    return pose_;
}

void Frame::SetKeyFrame() {
    static long keyframe_factory_id = 0;
    is_keyFrame_ = true;
    key_frame_id_ = keyframe_factory_id++;
}

Frame::Ptr Frame::CreateFrame() { 
    static long factory_id = 0;
    Frame::Ptr new_frame = std::make_unique<Frame>();
    new_frame->id_ = factory_id++;
    new_frame->SetPose(Eigen::Isometry3d::Identity());
    return new_frame;
}

const std::vector<std::shared_ptr<Feature>>& Frame::GetLeftFeatures() const {
    //std::unique_lock<std::mutex> lock(pose_mutex_);
    return features_left_;
}

const std::vector<std::shared_ptr<Feature>>& Frame::GetRightFeatures() const {
    //std::unique_lock<std::mutex> lock(pose_mutex_);
    return features_right_;
}

const cv::Mat& Frame::GetUndistortLeftImg() const {
    return left_image_; //去畸变过后的图像
}

const cv::Mat& Frame::GetUndistortRightImg() const {
    return right_image_; //去畸变过后的图像
}

bool Frame::SetLeftImg(const cv::Mat& img) {
    left_image_ = img;
}

//因为设置完right image之后，两张图像就被Frame持有了，马上去畸变
bool Frame::SetRightImg(const cv::Mat& img) {
    right_image_ = img;

    UndistortBothImg();
}

void Frame::AddLeftFeature(const std::shared_ptr<Feature>& feature) {
    //std::shared_ptr<Feature> feature_copy = std::make_shared<Feature>(feature->GetFrame(),feature->GetKeyPoint());
    features_left_.push_back(feature);
}

void Frame::AddRightFeature(const std::shared_ptr<Feature>& feature) {
    features_right_.push_back(feature);
}

const int Frame::GetLeftFeatureNum() const {
    return features_left_.size();
}

const int Frame::GetRightFeatureNum() const {
    return features_right_.size();
}

unsigned long Frame::GetId() {
    return id_;
}

unsigned long Frame::GetKeyFrameId() {
    return key_frame_id_;
}

const std::shared_ptr<Feature> Frame::GetLeftFeatureByindex(const int& index) const {
    if(index >= GetLeftFeatureNum()) {
        LOG(ERROR) << "The index is above the max.";
        return features_left_[GetLeftFeatureNum() - 1];
    }
    return features_left_[index];
}

const std::shared_ptr<Feature> Frame::GetRightFeatureByindex(const int& index) const {
    if(index >= GetRightFeatureNum()) {
        LOG(ERROR) << "The index is above the max.";
        return features_right_[GetRightFeatureNum() - 1];
    }
    return features_right_[index];
}

bool Frame::UndistortBothImg() {
    //调用Camera类方法对双目采集到的图像去畸变
    if(left_camera_) {
        left_camera_->UndistortImage(left_image_);
    }
    if(right_camera_) {
        right_camera_->UndistortImage(right_image_);
    }
}

void Frame::SetLeftCamera(std::shared_ptr<Camera> camera) {
    left_camera_ = camera;
}

void Frame::SetRightCamera(std::shared_ptr<Camera> camera) {
    right_camera_ = camera;
}

const std::shared_ptr<Camera>& Frame::GetLeftCamera() {
    if(left_camera_) {
        return left_camera_;
    }
}

const std::shared_ptr<Camera>& Frame::GetRightCamera() {
    if(right_camera_) {
        return right_camera_;
    }
}

} // namespace my_slam