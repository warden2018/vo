#include "key_frame.h"


namespace my_slam {

KeyFrame::KeyFrame(std::shared_ptr<Frame> frame, std::shared_ptr<Map> map, std::shared_ptr<KeyFrameDataBase> pKFDB)
 : map_(map),
   pKFDB_(pKFDB_) {

}

KeyFrame::Ptr KeyFrame::CreateNewKeyFrame(std::shared_ptr<Frame> frame, std::shared_ptr<Map> map, std::shared_ptr<KeyFrameDataBase> pKFDB) {
    std::unique_ptr<KeyFrame> new_keyFrame = std::make_unique<KeyFrame>(frame,map,pKFDB);
    return new_keyFrame;
}

} //namespace my_slam 