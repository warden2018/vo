/*
* 作者：李阳
* 简介：Eigen 矩阵、李群、向量的type defs.方便在其他类当中使用
*/

#ifndef MYSLAM_TYPES_H_
#define MYSLAM_TYPES_H_


/*
* Eigen的主要头文件，包含了稠密矩阵（dense matrix）,向量（vector）.
*/
#include <Eigen/Core>
/*
* 几何模块提供了如下的功能：
* 1. 固定维度下的单应变化（homogeneous transformation）
* 2. 平移，旋转
* 3. 四元数
* 4 . 叉积
* 5. 正交向量产生
* 6. 两个点云之间的最小二乘变换
*/
#include <Eigen/Geometry>

/*
*
*
*/
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
//针对Eigen的类型定义,double
//向量
typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<float, 3, 1> Vec3f;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;

//矩阵
typedef Eigen::Matrix<double, 2, 2> Mat22;
typedef Eigen::Matrix<double, 3, 3> Mat33;
typedef Eigen::Matrix<double, 4, 4> Mat44;
typedef Eigen::Matrix<double, 3, 4> Mat34;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;

//李群：考虑是否使用。vslambook2代码中是使用Sophus库的，但是
typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;


//图像
#include <opencv2/core/core.hpp>
//#include <opencv2/opencv.hpp>
using cv::Mat;

//相机针孔模型参数

struct PinpoleIntrinsics {
    //pinpole camera intrinsices
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double k1_;
  double k2_;
  double p1_;
  double p2_; 
};


//google glog
#include <glog/logging.h>

#endif // MYSLAM_TYPES_H_