/*
* 作者：李阳
* 简介：
* 三角测量：通过不同位置对同一路标点进行测量，从测量到的图像像素来推测路标点的深度信息。
*/

#ifndef MYSLAM_TRACKING_H_
#define MYSLAM_TRACKING_H_

#include <memory>
#include <opencv2/features2d.hpp>
#include "types.h"
#include "frame.h"
#include "camera.h"
#include "viewer.h"

namespace my_slam{

class Map;
class Backend;

enum class TrackingStatus { INITIALIZING, TRACKING_GOOD, TRACKING_BAD, TRACKING_LOST };
class Tracking {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Tracking> Ptr;

  Tracking();

  void AddFrame(std::shared_ptr<Frame> frame);

  TrackingStatus GetStatus();

  bool SetMap(std::shared_ptr<Map> map);

  bool SetBackend(std::shared_ptr<Backend> backend);

  bool SetCameras(Camera::Ptr left, Camera::Ptr right);

  bool SetViewer(std::shared_ptr<Viewer> viewer);
 private:

  /**
  * 双目相机的初始化，通过三角化找到初始化的地图点
  * @return true if success
  */
  bool stereoInit(); 
  
  /**
  * 光流跟踪，新相机位姿的估计和内点统计，返回了跟踪的效果是否是OK还是BAD
  * @return true if success
  */
  bool Track();

  /**
  * 重置Tracking
  * @return true if success
  */
  bool Reset();

  /**
  * 跟踪新图像上的特征点
  * @return 
  */
  int TrackCurrentFrame();

  /**
  * 根据光流的前后帧匹配结果，利用图优化做非线性优化，BA
  * @return 
  */
  int EstimateCurrentPose();

   /**
  * 根据光流的前后帧匹配结果，利用图优化做非线性优化，BA
  * @return 
  */
  int EstimateCurrentPose_g2o();

  /**
  * 如果光流匹配到的特征点数量不够多，那么认为遇到了较新的场景，那么，就把该Frame添加到
  * @return true 如果添加成功
  */
  bool InsertKeyframe();

   /**
   * 对判断为新关键帧内的左图像特征点的地图点（MapPoint）添加当前帧观察到的Feature
   * @return 
   */
  void SetObservationsForKeyFrame();
  /**
   * 对判断为新关键帧内的左图像特征点的地图点（MapPoint）添加当前帧观察到的Feature
   * @return 
   */
  int DetectNewLeftFeatures();
  /**
   * 对比当前帧右图和左图，找到左图中的特征点在右图当中的位置
   * @return 
   */
  int DetectFeaturesOfRight();
  /**
   * 基于左右目图像的2D特征点和左、右相机在Frame下的位姿，得到特征点在Frame下的3D坐标。
   * 然后依据当前frame在世界坐标系的位姿，可以得到世界坐标系的地图点。在三角化过程中，
   * 不是所有左右图的特征点都参与，只有左图里面的未生成3D地图点会参与三角化。
   * @return 
   */
  void TriagulateNewPoints();
  /**
   * 记录跟踪的当前状态
   * @return 
   */
  void LogTrackingStatus();
  /**
   * 双目初始化之后，三角化得到了地图点信息，构建初始的地图
   * @return 
   */
  bool BuildInitMap();

 private:
  TrackingStatus status_;
  std::shared_ptr<Frame> current_frame_;
  std::shared_ptr<Frame> last_frame_;
  Camera::Ptr left_camera_;
  Camera::Ptr right_camera_;

  //其他类的指针
  std::shared_ptr<Map> map_ = nullptr;
  std::shared_ptr<Backend> backend_ = nullptr;
  std::shared_ptr<Viewer> viewer_ = nullptr;
  Eigen::Isometry3d relative_motion_;
  int tracking_inliers_;
  cv::Ptr<cv::GFTTDetector> gftt_detector_ = nullptr;
};
}//namespace my_slam

#endif // MYSLAM_TRACKING_H_