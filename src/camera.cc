#include "camera.h"
#include <yaml-cpp/yaml.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace my_slam {

Camera::Camera(const std::string& params_file) 
    : camera_side_(CameraSide::LEFT),
      intrinsincs_file_(params_file) {
    LOG(INFO) << "Pass " << intrinsincs_file_ << "to camera.";
}

Eigen::Isometry3d Camera::Pose() {
    return T_c_b_;
}

Camera::Ptr Camera::CreateCamera(const std::string& paramsfile) {
    Camera::Ptr new_camera(new Camera(paramsfile));
    return new_camera;
}

bool Camera::Init(CameraSide side) {
    LOG(INFO) << "file dir and name is: " << intrinsincs_file_;
LoadIntrinsics(intrinsincs_file_);
LoadExtrinsics(intrinsincs_file_);
camera_side_ = side;
if(camera_side_ == CameraSide::LEFT) {
    LOG(INFO) << "Left camera from base_link: ";
    LOG(INFO) << T_c_b_.matrix();
} else {
    LOG(INFO) << "Right camera from base_link: ";
    LOG(INFO) << T_c_b_.matrix();
}
}

cv::Mat Camera::K_cv() {
    cv::Mat K = (cv::Mat_<double>(3,3)<< 
         pinpole_intrinsics_.fx_,
         0,
         pinpole_intrinsics_.cx_,
         0,
         pinpole_intrinsics_.fy_,
         pinpole_intrinsics_.cy_,
         0,
         0,
         1);
    return K;
}

Mat33 Camera::K_eigen() {
    Mat33 K;
    K << pinpole_intrinsics_.fx_,
         0,
         pinpole_intrinsics_.cx_,
         0,
         pinpole_intrinsics_.fy_,
         pinpole_intrinsics_.cy_,
         0,
         0,
         1;

    return K;
}

Vec3 Camera::World2camera(const Vec3& p_w, const Eigen::Isometry3d& T_b_w) {
    Vec3 p_c = T_c_b_ * T_b_w * p_w;
    return p_c;
}

Vec3 Camera::Camera2world(const Vec3& p_c, const Eigen::Isometry3d& T_b_w) {
    Vec3  p_w = T_b_w.inverse() * T_c_b_.inverse() * p_c;
    return p_w;
}

Vec2 Camera::Camera2pixel(const Vec3& p_c) {
    Vec3 normalized = p_c;
    if(normalized(2) <= 0) {
        LOG(ERROR) << "The camera z coordinate is below zero: " << normalized(3);
        return Vec2(0,0);
    }
    normalized(0) /= normalized(2);
    normalized(1) /= normalized(2);
    normalized(2) = 1;

    Vec3 pixel = K_eigen() * normalized;
    return Vec2(pixel(0),pixel(1));
}

Vec3 Camera::Pixel2camera(const Vec2& p_p, const double depth) {
    Vec3 normalized(p_p(0),p_p(1),1);
    normalized = depth * K_eigen().inverse() * normalized;
    return normalized;
}

Vec2 Camera::World2pixel(const Vec3& p_w, const Eigen::Isometry3d& T_b_w) {
    Vec2 pixel = Camera2pixel(World2camera(p_w,T_b_w));
    return pixel;
}

Vec3 Camera::Pixel2world(const Vec2& p_p, const Eigen::Isometry3d& T_b_w,const double depth) {
    Vec3 world = Camera2world(Pixel2camera(p_p,depth),T_b_w);
    return world;
}

Eigen::Isometry3d Camera::GetT_c_b() {
    return T_c_b_;
}

cv::Mat Camera::UndistortImage(cv::Mat img) {
    cv::Mat cameraMatrix = K_cv();
    const cv::Mat D = ( cv::Mat_<double> (4,1) << pinpole_intrinsics_.k1_, 
                                                pinpole_intrinsics_.k2_, 
                                                pinpole_intrinsics_.p1_,
                                                pinpole_intrinsics_.p2_);

    cv::Mat undistortedImg;

    cv::undistort(img, undistortedImg, cameraMatrix, D);

    return undistortedImg;
}

void Camera::SetLeft() {
    camera_side_ = CameraSide::LEFT;
}

void Camera::SetRight() {
    camera_side_ = CameraSide::RIGHT;
}

bool Camera::LoadIntrinsics(const std::string& fileStr) {
    YAML::Node config = YAML::LoadFile(fileStr);
    if (config["Camera.fx"]) {
        pinpole_intrinsics_.fx_ = config["Camera.fx"].as<double>();
        pinpole_intrinsics_.fy_ = config["Camera.fy"].as<double>();
        pinpole_intrinsics_.cx_ = config["Camera.cx"].as<double>();
        pinpole_intrinsics_.cy_ = config["Camera.cy"].as<double>();

        pinpole_intrinsics_.k1_ = config["Camera.k1"].as<double>();
        pinpole_intrinsics_.k2_ = config["Camera.k2"].as<double>();
        pinpole_intrinsics_.p1_ = config["Camera.p1"].as<double>();
        pinpole_intrinsics_.p2_ = config["Camera.p2"].as<double>();
    }
}

bool Camera::LoadExtrinsics(const std::string& fileStr) {
    //TODO(李阳)：根据左目还是右目相机，完成对该相机到Frame的外参数
    //初始化。 所以pose返回的就是初始化完成之后的外参数。
    Eigen::Matrix4d transformation_matrix;
    try {
    YAML::Node config = YAML::LoadFile(fileStr);
    if (!config) {
      LOG(ERROR) << "Open TransformationMatrix File: " << fileStr << "failed.";
      return false;
    }
    if (!config["translation.x"]) {
      LOG(ERROR) << "Open TransformationMatrix File: " << fileStr << "has no transform.";
      return false;
    }

    //fill translation
    if (config["translation.x"]) {
      (transformation_matrix)(0, 3) =
          config["translation.x"].as<double>();
      (transformation_matrix)(1, 3) =
          config["translation.y"].as<double>();
      (transformation_matrix)(2, 3) =
          config["translation.z"].as<double>();
    } else {
      LOG(ERROR) << "TransformationMatrix File: " << fileStr << "has no transform:translation.";
      return false;
    }
    // fill rotation
    if (config["rotation.x"]) {
      double qx = config["rotation.x"].as<double>();
      double qy = config["rotation.y"].as<double>();
      double qz = config["rotation.z"].as<double>();
      double qw = config["rotation.w"].as<double>();
      Eigen::Quaternion<double> rotation(qw, qx, qy, qz);
      (transformation_matrix).block<3, 3>(0, 0) = rotation.toRotationMatrix();
    } else {
      LOG(ERROR) << "TransformationMatrix File: " << fileStr << " has no transform:rotation.";
      return false;
    }
  } catch (const YAML::Exception &e) {
    LOG(ERROR) <<"[Error]" << fileStr << "load failed. error: " << e.what();
    LOG(ERROR) << "Please ensure param file is exist or format is correct";
    return false;
  }

  T_c_b_.matrix() = transformation_matrix;
  if(camera_side_==CameraSide::LEFT) {
      LOG(INFO) << "Left camera(" << ") to base_link translform is: " << T_c_b_.matrix(); 
  }else {
      LOG(INFO) << "Right camera(" << ") to base_link translform is: " << T_c_b_.matrix();
  }
  
}
} // namespace my_slam



