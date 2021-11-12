/*
* 作者：李阳
* 简介：
*/

#ifndef MYSLAM_BACKEND_H_
#define MYSLAM_BACKEND_H_

#include <memory>
#include <condition_variable>
#include <thread>
#include "types.h"
#include "map.h"
#include "camera.h"

namespace my_slam{

class Backend{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Backend> Ptr;

  Backend();

  void UpdateMap();

  void SetMap(std::shared_ptr<Map> map);

  void SetCameras(Camera::Ptr left, Camera::Ptr right);

 private:
  void BakcendLoop();

  bool Optimize_g2o(Map::MapPointsType& mps,Map::KeyFramesType& kfs);

  bool Optimize_ceres(Map::MapPointsType& mps,Map::KeyFramesType& kfs);

 private:
  Camera::Ptr left_cam_ = nullptr;
  Camera::Ptr right_cam_ = nullptr;
  std::shared_ptr<Map> map_;
  std::condition_variable map_updated_;
  std::mutex mutex_;
  std::thread backend_thread_;
  std::atomic<bool> backend_running_;
};
}//namespace my_slam

#endif // MYSLAM_BACKEND_H_