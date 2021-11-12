/*
* 作者：李阳
* 简介：
*/

#ifndef MYSLAM_VISUALODOMETRY_H_
#define MYSLAM_VISUALODOMETRY_H_

#include <memory>
#include <string>
#include "types.h"
#include "frame.h"
#include "sensor_msgs/Image.h"
#include "camera.h"
#include "map.h"
#include "tracking.h"
#include "backend.h"
#include "viewer.h"


namespace my_slam{

class VisualOdometry {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<VisualOdometry> Ptr;
  VisualOdometry() {}
  VisualOdometry(std::string& conf_dir);

  bool Init();

  void Run();
  
  //Frame::Ptr NextFrame();

  void AddRosData(const sensor_msgs::ImageConstPtr msg1, const sensor_msgs::ImageConstPtr msg2);
  

 private:
  std::string   configFile_;
  Tracking::Ptr tracking_ = nullptr;
  Map::Ptr      map_ = nullptr;
  Backend::Ptr  backend_ = nullptr;
  Viewer::Ptr   viewer_ = nullptr;
  Camera::Ptr   left_camera_;
  Camera::Ptr   right_camera_;
  bool          vo_run_;
};
}//namespace my_slam

#endif // MYSLAM_VISUALODOMETRY_H_