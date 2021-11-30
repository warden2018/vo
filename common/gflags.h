//
// Created by yang on 2021/3/29.
//

#include <gflags/gflags.h>

DECLARE_string(left_img_file);
DECLARE_string(disparity_file);
DECLARE_int32(nPoints);
DECLARE_int32(nBorder);
DECLARE_int32(halfPatchSize);
DECLARE_int32(interations);
DECLARE_int32(img_number);
DECLARE_int32(pyramids);
DECLARE_double(pyr_scale);
//log
DECLARE_string(myslamlog_dir);
DECLARE_string(left_camera_params);
DECLARE_string(right_camera_params);
//Tracking params
DECLARE_int32(nFeatures);
DECLARE_int32(nFeatureIinit);
DECLARE_int32(nFeaturesTracking);
DECLARE_int32(nFeaturesTrackingBad);
DECLARE_int32(nFeaturesKeyframe);
DECLARE_double(gftt_qualityLevel);
DECLARE_double(gftt_minDistance);
DECLARE_double(maskNeigbourPixels);

//backend optimization params
DECLARE_bool(useDense);
DECLARE_double(2Dhuber95);
DECLARE_double(3Dhuber95);

//ROS interface for image topics
DECLARE_string(leftImgTopic);
DECLARE_string(rightImgTopic);
DECLARE_int32(queue_size);

//Map 
DECLARE_int32(activateKF);
//vo config file
DECLARE_string(vo_config);

//ORB Extractor config file
DECLARE_string(LeftORBParamsFile);
DECLARE_string(RightORBParamsFile);
