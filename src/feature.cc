/* 
* 作者：李阳
*
*/

#include "feature.h"
#include "frame.h"

namespace my_slam {

Feature::Feature(std::shared_ptr<Frame> frame,const cv::KeyPoint& kp)
    : frame_(frame),
      position_(kp) {}


std::shared_ptr<MapPoint> Feature::GetMapPoint() {
    return map_point_.lock();
}

cv::KeyPoint Feature::GetKeyPoint() {
    return position_;
}

void Feature::SetMapPoint(const std::shared_ptr<MapPoint> mp) {
    //int scount = mp.use_count();
    map_point_ = mp;
    //scount = mp.use_count();
}

bool Feature::IsOutlier() {
    return is_outlier_;
}

void Feature::SetOutlier(const bool outlier) {
    is_outlier_ = outlier;
}

std::weak_ptr<Frame> Feature::GetFrame() {
    return frame_;
}

bool Feature::IsOnLeft() {
 return is_onLeft_image_;
}

void Feature::SetOnLeft(const bool onleft) {
    is_onLeft_image_ = onleft;
}

Feature::Ptr Feature::CreateNewFeature(std::shared_ptr<Frame> frame,const cv::KeyPoint& kp) {
    std::unique_ptr<Feature> new_feature = std::make_unique<Feature>(frame,kp);
    new_feature->is_outlier_ = false;
    new_feature->is_onLeft_image_ = true;
    return new_feature;
}

} // namespace my_slam