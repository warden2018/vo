

#ifndef MYSLAM_ORBVOCABULARY_H
#define MYSLAM_ORBVOCABULARY_H

#include "DBoW2/DBoW2/FORB.h"
#include "DBoW2/DBoW2/TemplatedVocabulary.h"

namespace my_slam
{

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
  ORBVocabulary;

} //namespace my_slam

#endif // MYSLAM_ORBVOCABULARY_H
