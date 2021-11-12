/*
* 作者：李阳
* 简介：路标点类，包含了在世界坐标下的3D坐标值，图像上观察到这个点的特征，也就是观测信息等。
*/


#ifndef MYSLAM_MAPPOINT_H_
#define MYSLAM_MAPPOINT_H_

#include "types.h"
#include <atomic>
#include <list>
#include <mutex>
#include <memory>

namespace my_slam{

class Frame;
class Feature;

class MapPoint {
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::unique_ptr<MapPoint> Ptr;

  MapPoint();

  MapPoint(const long& id, const Vec3& position);
  MapPoint(const Vec3& position);
  const Vec3& Pos();

  void SetPos(const Vec3& pos);

  void AddObservation(std::shared_ptr<Feature> feature);

  void RemoveObservation(std::shared_ptr<Feature> feature);

  const std::list<std::weak_ptr<Feature>>& GetObservations();

  bool IsOutLier();

  void SetOutlier(const bool outlier);

  unsigned long GetId();
  
  static MapPoint::Ptr CreateNewMapPoint(const Vec3& pos);
 
 private:
  unsigned long id_; 
  bool is_outlier_;
  Vec3 pos_;
  std::mutex data_mutex_;
  int observed_times_;
  std::list<std::weak_ptr<Feature>> observations_;


};

} //namespace my_slam

#endif // MYSLAM_MAPPOINT_H_