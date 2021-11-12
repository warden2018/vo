
#include "map_point.h"
#include "feature.h"

namespace my_slam{

MapPoint::MapPoint(const long& id, const Vec3& position)
    : id_(id),
      pos_(position),
      observed_times_(0),
      is_outlier_(true) { }

MapPoint::MapPoint()
    : observed_times_(0),
      is_outlier_(true) { }


MapPoint::MapPoint(const Vec3& position)
    : pos_(position),
      observed_times_(0),
      is_outlier_(true) { 
    
}
const Vec3& MapPoint::Pos() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    return pos_;
}

void MapPoint::SetPos(const Vec3& pos) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    pos_ = pos;
}

void MapPoint::AddObservation(std::shared_ptr<Feature> feature) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    observations_.push_back(feature);
    observed_times_++;
}

bool MapPoint::IsOutLier() {
    return is_outlier_;
}

void MapPoint::SetOutlier(const bool outlier) {
    is_outlier_ = outlier;
}

void MapPoint::RemoveObservation(std::shared_ptr<Feature> feature) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    for (auto iter = observations_.begin(); iter != observations_.end(); ++iter) {
        if(iter->lock() == feature) {
            observations_.erase(iter);
            feature->GetMapPoint().reset();
            observed_times_--;
            break;
        }
    }
}

const std::list<std::weak_ptr<Feature>>& MapPoint::GetObservations() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    return observations_;
}

MapPoint::Ptr MapPoint::CreateNewMapPoint(const Vec3& pos) {
    static long factory_id = 0;
    //std::unique_ptr<MapPoint> new_mapPoint = std::make_unique<MapPoint>(pos);
    std::unique_ptr<MapPoint> new_mapPoint(new MapPoint(pos));
    new_mapPoint->id_ = factory_id++;
    new_mapPoint->SetOutlier(false);
    return new_mapPoint;
}

unsigned long MapPoint::GetId() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    return id_;
}

} // namespace my_slam