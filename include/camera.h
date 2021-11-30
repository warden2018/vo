/*
* 作者：李阳
* 简介：相机类，负责读取文件中的针孔模型内参数，提供三种坐标系的转换方法
*/

#ifndef MYSLAM_CAMERA_H_
#define MYSLAM_CAMERA_H_

#include <memory>
#include "types.h"

namespace my_slam{
class Camera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Camera> Ptr;
  enum class CameraSide {LEFT,RIGHT};
  Camera();

  Camera(const std::string& params_file);
  bool Init(CameraSide side);
  bool LoadIntrinsics(const std::string& fileStr);
  bool LoadExtrinsics(const std::string& fileStr);
  Eigen::Isometry3d Pose();

  cv::Mat K_cv();

  Mat33 K_eigen();

  void SetLeft();

  void SetRight();

  const float GetBaselineMeter() const;

  const float GetBaselineFx() const;

  cv::Mat UndistortImage(cv::Mat img);
  //坐标系转换：世界坐标系，相机坐标系和图像坐标系
  Vec3 World2camera(const Vec3& p_w, const Eigen::Isometry3d& T_b_w); // T_c_w:T^c_w in latex
  Vec3 Camera2world(const Vec3& p_c, const Eigen::Isometry3d& T_b_w); // T_c_w:T^c_w in latex
  Vec2 Camera2pixel(const Vec3& p_c); 
  Vec3 Pixel2camera(const Vec2& p_p, const double depth);
  cv::Point2f Pixel2camera(const cv::KeyPoint& p_p);
  Vec2 World2pixel(const Vec3& p_w, const Eigen::Isometry3d& T_b_w);
  Vec3 Pixel2world(const Vec2& p_p, const Eigen::Isometry3d& T_b_w,const double depth);
  Eigen::Isometry3d GetT_c_b();
  static Camera::Ptr CreateCamera(const std::string& paramsfile);
  cv::Mat GetDistCoeff();
 private:
  std::string intrinsincs_file_;
  PinpoleIntrinsics pinpole_intrinsics_;
  double baseline_;
  int width_;
  int height_;
  int fps_;
  Eigen::Isometry3d T_c_b_ = Eigen::Isometry3d::Identity(); //base_link到相机坐标系的转换T_b_c
               //ceres的参数更方便一些。
  CameraSide camera_side_; //标记了这个相机是不是左相机，如果是右侧相机，右侧到左侧差一个相机的外参数

};
}//namespace my_slam

#endif // MYSLAM_CAMERA_H_