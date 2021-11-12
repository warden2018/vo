/*
* 作者：李阳
* 简介：双目相机采集到的两帧图像，当前的位姿，根据识别到的特征点的数量来判断是否是关键帧。
* Tracking类需要在每次ROS node收到图像消息之后，转换为Frame数据结构，然后调用Tracking
* 的AddFrame方法实现对current frame 和last frame的更新迭代。
*/

#ifndef MYSLAM_FRAME_H_
#define MYSLAM_FRAME_H_

#include "types.h"
#include <atomic>
#include <list>
#include <mutex>
#include <vector>
#include <Eigen/Core>
#include "sensor_msgs/Image.h"

namespace my_slam{

class Feature;
class MapPoint;
class Camera;

class Frame {
public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 typedef std::unique_ptr<Frame> Ptr;

 Frame();
 Frame(long id, double ts, const Eigen::Isometry3d& pose, const Mat& leftImg,const Mat& rightImg);

 const Eigen::Isometry3d& Pose();

 void SetPose(const Eigen::Isometry3d& pose);

 void SetKeyFrame();

 const std::vector<std::shared_ptr<Feature>>& GetLeftFeatures() const;

 const std::vector<std::shared_ptr<Feature>>& GetRightFeatures() const;
 
 const std::shared_ptr<Feature> GetLeftFeatureByindex(const int& index) const;

 const std::shared_ptr<Feature> GetRightFeatureByindex(const int& index) const;

 void AddLeftFeature(const std::shared_ptr<Feature>& feature);

 void AddRightFeature(const std::shared_ptr<Feature>& feature);

 const int GetLeftFeatureNum() const;

 const int GetRightFeatureNum() const;
 
 const cv::Mat& GetUndistortLeftImg() const;

 const cv::Mat& GetUndistortRightImg() const;

 bool SetLeftImg(const cv::Mat& img);

 bool SetRightImg(const cv::Mat& img);

 bool UndistortBothImg();

 unsigned long GetId();

 unsigned long GetKeyFrameId();
 
 void SetLeftCamera(std::shared_ptr<Camera> camera);

 void SetRightCamera(std::shared_ptr<Camera> camera);
 
 const std::shared_ptr<Camera>& GetLeftCamera();

 const std::shared_ptr<Camera>& GetRightCamera();

 static Frame::Ptr CreateFrame();

private:
 unsigned long id_;
 unsigned long key_frame_id_;
 bool is_keyFrame_;
 double time_stamp_;
 Eigen::Isometry3d pose_;
 std::mutex pose_mutex_;
 cv::Mat left_image_, right_image_;
 
 std::shared_ptr<Camera> left_camera_;
 std::shared_ptr<Camera> right_camera_;

 std::vector<std::shared_ptr<Feature>> features_left_;
 std::vector<std::shared_ptr<Feature>> features_right_;


};
}//namespace my_slam

#endif //MYSLAM_FRAME_H_