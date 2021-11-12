#include "map.h"
#include "gflags.h"

namespace my_slam {

Map::MapPointsType Map::GetAllMapPoints() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    return mapPoints_;
}

Map::MapPointsType Map::GetAllActiveMapPoints() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    return activeMapPoints_;
}

Map::KeyFramesType Map::GetAllKeyFrames() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    return keyFrames_;
}

Map::KeyFramesType Map::GetAllActiveKeyFrames() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    return activeKeyFrames_;
}

void Map::CleanMap() {

}

void Map::InsertKeyFrame(std::shared_ptr<Frame> frame) {
    if(keyFrames_.find(frame->GetId()) == keyFrames_.end()) {
        keyFrames_.insert(make_pair(frame->GetId(),frame));
        activeKeyFrames_.insert(make_pair(frame->GetId(),frame));
    } else {
        keyFrames_[frame->GetId()] = frame;
        activeKeyFrames_[frame->GetId()] = frame;
    }

    if(activeKeyFrames_.size() > FLAGS_activateKF) {
        RemoveOldKeyFrames();
    }
}

void Map::InsertMapPoint(std::shared_ptr<MapPoint> mapPoint) {
    if(mapPoints_.find(mapPoint->GetId()) == mapPoints_.end()) {
        mapPoints_.insert(make_pair(mapPoint->GetId(),mapPoint));
        activeMapPoints_.insert(make_pair(mapPoint->GetId(),mapPoint));
    } else {
        mapPoints_[mapPoint->GetId()] = mapPoint;
        activeMapPoints_[mapPoint->GetId()] = mapPoint;
    }
}



void Map::RemoveOldKeyFrames() {

}


} //namespace my_slam