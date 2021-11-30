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
class ORBextractor;

class Frame {
public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 typedef std::unique_ptr<Frame> Ptr;

 Frame(double ts, std::shared_ptr<Camera> left, std::shared_ptr<Camera> right,
                                std::shared_ptr<ORBextractor> leftExtrator,
                                std::shared_ptr<ORBextractor> rightExtrator, 
                                const Eigen::Isometry3d& pose, 
                                const Mat& leftImg,const Mat& rightImg);

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
 
 const cv::Mat GetUndistortLeftImg();

 const cv::Mat GetUndistortRightImg();

 bool SetLeftImg(const cv::Mat& img);

 bool SetRightImg(const cv::Mat& img);

 bool UndistortBothImg();

 unsigned long GetId();

 unsigned long GetKeyFrameId();
 
 void SetLeftCamera(std::shared_ptr<Camera> camera);

 void SetRightCamera(std::shared_ptr<Camera> camera);
 
 const std::shared_ptr<Camera>& GetLeftCamera();

 const std::shared_ptr<Camera>& GetRightCamera();
 //flag:0,左相机，1,右相机
 void ExtractORB(int flag, const cv::Mat& img, const int x0, const int x1);
 
 //ORB描述子匹配左右图像上的特征点，如果能够匹配到，视差计算深度，得到地图点。这个地图点，
 //是后面做相机姿态估计的基础。
 void ComputeStereoMatches();
 
 //返回的是世界坐标系的坐标
 Vec3f UnprojectStereo(const int &i);
 
 void UndistortKeyPoints();

 static Frame::Ptr CreateFrame(double ts, 
                                std::shared_ptr<Camera> left, std::shared_ptr<Camera> right,
                                std::shared_ptr<ORBextractor> leftExtrator,
                                std::shared_ptr<ORBextractor> rightExtrator, 
                                const Eigen::Isometry3d& pose, 
                                const Mat& leftImg,const Mat& rightImg);

 int N; //ORB detector检测到的特征点数量
 int Nleft_, Nright_;
 std::vector<float> vDepth_;
 //地图点容器
 std::vector<std::shared_ptr<MapPoint>> vpMapPoints_;

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
 
 

 //双目的基线长度
 float baseline_; //单位是米
 float baselineFx_; //单位是像素

 //ORB extractor related
 std::shared_ptr<ORBextractor> ORBextractorLeft_;
 std::shared_ptr<ORBextractor> ORBextractorRight_;
 std::vector<cv::KeyPoint> vKeys_, vKeysRight_;
 // ORB descriptor, each row associated to a keypoint.
 cv::Mat Descriptors_, DescriptorsRight_;
 int monoLeft_, monoRight_;
 std::vector<float> vuRight_;
 std::vector<cv::KeyPoint> vKeysUn_;
 

 // Scale pyramid info.
 int nScaleLevels_;
 float fScaleFactor_;
 float fLogScaleFactor_;
 std::vector<float> vScaleFactors_;
 std::vector<float> vInvScaleFactors_;
 std::vector<float> vLevelSigma2_;
 std::vector<float> vInvLevelSigma2_;


};
}//namespace my_slam

#endif //MYSLAM_FRAME_H_