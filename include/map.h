/*
* 作者：李阳
* 简介：地图类，实际持有Frame和MapPoint对象。注意，Feature类当中对Frame和MapPoint对象的引用使用的
* 是std::weak_ptr,所以对这两个对象的生命周期的维护是在Map。
*/

#ifndef MYSLAM_MAP_H_
#define MYSLAM_MAP_H_

#include <unordered_map>
#include <memory>
#include "frame.h"
#include "map_point.h"


namespace my_slam{

class Map {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Map> Ptr;
  typedef std::unordered_map<unsigned long, std::shared_ptr<MapPoint>> MapPointsType;
  typedef std::unordered_map<unsigned long, std::shared_ptr<Frame>> KeyFramesType;
  
  Map() {}

  void InsertKeyFrame(std::shared_ptr<Frame> frame);

  void InsertMapPoint(std::shared_ptr<MapPoint> mapPoint);

  void CleanMap();

  MapPointsType  GetAllMapPoints();

  MapPointsType GetAllActiveMapPoints();

  KeyFramesType GetAllKeyFrames();

  KeyFramesType GetAllActiveKeyFrames();

 private:
  void RemoveOldKeyFrames();

 private:
  std::mutex data_mutex_;

  MapPointsType mapPoints_;
  MapPointsType activeMapPoints_;

  KeyFramesType keyFrames_;
  KeyFramesType activeKeyFrames_;


};
}//namespace my_slam

#endif // MYSLAM_MAP_H_