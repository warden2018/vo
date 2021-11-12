
#ifndef CERESTYPES_H_
#define CERESTYPES_H_

#include "ceres/ceres.h"
#include "types.h"


namespace my_slam{
/* Template class for BA
/* 测量值：在前一个相机坐标系下的特征点坐标，在下一帧图像上测量到的这些特征点的像素坐标。相机内参数K是固定的。
/* operator()中，待优化的参数包含了平移旋转，ceres_rot是旋转，形式是轴角，
/* ceres_trans是平移
*/
struct ReprojectionError {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ReprojectionError(Eigen::Vector3d point,Eigen::Vector2d pixel,Eigen::Matrix3d K)
      : point_(point), pixel_(pixel),K_(K) {}

  template <typename T>
  bool operator()(const T* const ceres_rot, const T* const ceres_trans, T* residuals) const {
    //step1: 读取平移旋转的数据，转换成为Eigen的格式
    T p1[3];
    T p2[3];
    p1[0] = T(point_(0));
    p1[1] = T(point_(1));
    p1[2] = T(point_(2));
    //step2: 将世界坐标系下的点坐标转换为相机坐标系的坐标
    //旋转
    ceres::AngleAxisRotatePoint(ceres_rot, p1, p2);
    //平移
    p2[0] += ceres_trans[0];
    p2[1] += ceres_trans[1];
    p2[2] += ceres_trans[2];
    // cout << "After rotation and translation point: (" << p2[0] << ", "
    //                                         << p2[1] << ", "
    //                                         << p2[2] << ") " << endl;
    // cout << "K_ is: " << endl << K_ << endl; 
    //Step3: 计算投影到图像空间下的坐标
    Eigen::Vector<T,3> pixel_point;
    pixel_point(0) = K_(0,0) * p2[0] + K_(0,2) * p2[2];
    pixel_point(1) = K_(1,1) * p2[1] + K_(1,2) * p2[2];
    pixel_point(2) = K_(2,2) * p2[2];
    if(pixel_point(2)==0.0) {
        LOG(ERROR) << "Depth of map point should not be 0";
        return false;
    }

    pixel_point(0) /= pixel_point(2); //x 
    pixel_point(1) /= pixel_point(2); //y
    //cout << "predicted pixel: (" << pixel_point(0) << "," << pixel_point(1) << ")" << endl;
    // The error is the difference between the predicted and observed position.
    residuals[0] = pixel_point(0) - pixel_[0];
    residuals[1] = pixel_point(1) - pixel_[1];
    //cout << "pixel residual: (" << residuals[0] << ", " << residuals[1] << endl;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const Eigen::Vector3d point,
                                        const Eigen::Vector2d pixel,
                                        const Eigen::Matrix3d K) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 3>(
                new ReprojectionError(point,pixel,K)));
  }


  Eigen::Vector3d point_;
  Eigen::Vector2d pixel_;
  Eigen::Matrix3d K_;
};

} // namespace my_slam

#endif // CERESTYPES_H_