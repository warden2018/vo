#ifndef MYSLAM_KEYFRAME__H_
#define MYSLAM_KEYFRAME__H_


#include <memory>

namespace my_slam{

class Frame;
class MapPoint;
class Map;
class KeyFrameDataBase;

class KeyFrame {

typedef std::unique_ptr<KeyFrame> Ptr;
 public:
  KeyFrame(std::shared_ptr<Frame> frame, std::shared_ptr<Map> map, std::shared_ptr<KeyFrameDataBase> pKFDB);


  static KeyFrame::Ptr CreateNewKeyFrame(std::shared_ptr<Frame> frame, std::shared_ptr<Map> map, std::shared_ptr<KeyFrameDataBase> pKFDB);

 private:
  std::shared_ptr<Map> map_;
  std::shared_ptr<KeyFrameDataBase> pKFDB_;
  
};





} //namespace my_slam
#endif // MYSLAM_KEYFRAME__H_