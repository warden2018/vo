#ifndef MYSLAM_CONVERTER_H_
#define MYSLAM_CONVERTER_H_

#include <opencv2/core/core.hpp>

#include <Eigen/Dense>
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sba/g2o_types_sba_api.h"

namespace my_slam
{

class Converter
{
public:
    // static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static g2o::SE3Quat toSE3Quat(const cv::Mat& cvT);
    static g2o::SE3Quat toSE3Quat(const Eigen::Isometry3d& isoT);
    // static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    // static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    // static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCVMatf34(const Eigen::Isometry3d& isoT);
    // static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    // static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    // static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

    // static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double,2,1> toVector2d(const cv::Point2f &cvPoint);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
    static cv::Mat toCvMat34(const Eigen::Isometry3d& pose);
    // static std::vector<float> toQuaternion(const cv::Mat &M);

    static cv::Point2f Pixel2normal(const cv::KeyPoint& keyPoint, const cv::Mat K);
};

}// namespace my_slam

#endif // MYSLAM_CONVERTER_H_
