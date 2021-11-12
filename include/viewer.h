#ifndef MYSLAM_VIEWER_H_
#define MYSLAM_VIEWER_H_

#include <thread>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include "types.h"
#include "frame.h"
#include "map.h"

namespace my_slam {

class Viewer {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    void SetMap(Map::Ptr map) { map_ = map; }

    void Close();

    // 增加一个当前帧
    void AddCurrentFrame(std::shared_ptr<Frame> current_frame);

    // 更新地图
    void UpdateMap();

   private:
    void ThreadLoop();

    void DrawFrame(std::shared_ptr<Frame> frame, const float* color);

    void DrawMapPoints();

    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    /// plot the features in current frame into an image
    cv::Mat PlotFrameImage();

    std::shared_ptr<Frame> current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;

    std::thread viewer_thread_;
    bool viewer_running_ = true;

    std::unordered_map<unsigned long, std::shared_ptr<Frame>> active_keyframes_;
    std::unordered_map<unsigned long, std::shared_ptr<MapPoint>> active_landmarks_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;
};
}  // namespace my_slam

#endif  // MYSLAM_VIEWER_H_
