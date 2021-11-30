#include <vector>
#include <opencv2/opencv.hpp>
#include "types.h"
#include "converter.h"

namespace my_slam {

/**
 * 通过给定相机两个位置和姿态3行4列，前三列是旋转，第四列是平移，可以是同一个相机在不同位置，
 * 也可以是双目，每个相机采集到的图像上特征点坐标2D，通过DLT计算得到在世界坐标系下的3D坐标
 * @param[in] keypoint_1 -- 在camera1采集到的特征点坐标
 * @param[in] keypoint_2 -- 在camera2采集到的特征点坐标
 * @param[in] T1 -- camera1的姿态
 * @param[in] T2 -- camera2的姿态
 * @param[out] points 三角化之后的3D点集合,坐标系原点是base_link
 */
bool Triangulation(
  const std::vector<cv::Point2f>& pts_1,
  const std::vector<cv::Point2f>& pts_2,
  const cv::Mat& T1, const cv::Mat& T2,
  std::vector<cv::Point3d>& points) {

   if(pts_1.size() != pts_2.size()) {
    LOG(ERROR) << "KeyPoint size is not equal.";
    return false;
  }
  
  cv::Mat pts_3d_h;
  std::vector<cv::Point3f> pt_3d;
  //转换出来的是Homogeneous的坐标，需要接着转换为欧几里德坐标
  cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_3d_h);
  
  if(pts_3d_h.cols == 0) {
    LOG(ERROR) << "Triangulation failed.";
    return false;
  }

  //转换
  for(int i = 0; i < pts_3d_h.cols; i++) {
      Mat x = pts_3d_h.col(i);
      if(x.at<float>(3,0) == 0.0) {
        continue;
      }
      x /= x.at<float>(3,0);

      cv::Point3d p (
          x.at<float>(0,0),
          x.at<float>(1,0),
          x.at<float>(2,0)
      );

      points.push_back(p);
  }
  // convertPointsHomogeneous(pts_3d_h.reshape(4, 1), pt_3d);

  // for(int i = 0; i < pts_3d_h.cols; i++) {
  //     cv::Mat x = pts_3d_h.col(i);
  //     LOG(INFO) << "triangulatePoints output: " << i << ": [" << x.at<float>(0,0)
  //               << "," << x.at<float>(1,0)
  //               << "," << x.at<float>(2,0)
  //               << "," << x.at<float>(3,0)
  //               <<"'"
  //               <<"encludian coord: [" << pt_3d.at(i).x << ","
  //               << pt_3d.at(i).y << ","
  //               << pt_3d.at(i).z << "]";

  //     //保存结果
  //     points.push_back(cv::Point3d(pt_3d.at(i).x,pt_3d.at(i).y,pt_3d.at(i).z));
  // }


  return true;
}
    
} //namespace my_slam