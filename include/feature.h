/*
* 作者：李阳
* 简介：特征点类，opencv识别到的2D像素特征
*/


#ifndef MYSLAM_FEATURE_H_
#define MYSLAM_FEATURE_H_

#include <memory>
#include <Eigen/Core>
#include <opencv2/features2d.hpp>

//#include ""

namespace my_slam {

class Frame;
class MapPoint;

class Feature {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::unique_ptr<Feature> Ptr;

  Feature(){}

  Feature(std::shared_ptr<Frame> frame,const cv::KeyPoint& kp);
  
  std::shared_ptr<MapPoint> GetMapPoint();

  cv::KeyPoint GetKeyPoint();
  
  void SetMapPoint(const std::shared_ptr<MapPoint> mp);

  bool IsOutlier();

  void SetOutlier(const bool outlier);
  
  bool IsOnLeft();

  void SetOnLeft(const bool onleft);
  
  std::weak_ptr<Frame> GetFrame();
  
  static Feature::Ptr CreateNewFeature(std::shared_ptr<Frame> frame,const cv::KeyPoint& kp);
  
 private:
  std::weak_ptr<Frame> frame_; // 持有该feature的frame

  cv::KeyPoint position_; // 2D提取位置，像素坐标系下

  //std::weak_ptr<MapPoint> map_point_; // 关联的地图点

  bool is_outlier_;

  bool is_onLeft_image_; //标识是否在左相机采集图上，false表示为在右相机

  std::weak_ptr<MapPoint> map_point_; // 关联的地图点
};

}


#endif // MYSLAM_FEATURE_H_