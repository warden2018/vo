#ifndef MYSLAM_KEYFRAMEDATABASE__H_
#define MYSLAM_KEYFRAMEDATABASE__H_

#include "ORBVocabulary.h"
#include <vector>

namespace my_slam{

class Frame;
class MapPoint;
class Map;
class KeyFrame;

class KeyFrameDataBase {
 public:
  KeyFrameDataBase(const ORBVocabulary& voc);

 private:
  // Associated vocabulary
  const ORBVocabulary* mpVoc;

  // Inverted file
  std::vector<std::list<KeyFrame*> > mvInvertedFile;
};





}

#endif // MYSLAM_KEYFRAMEDATABASE__H_