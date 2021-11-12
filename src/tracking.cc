#include "tracking.h"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include "map.h"
#include "feature.h"
#include "backend.h"
#include "gflags.h"
#include "EigenTypes.h"
#include "ceres_types.h"
#include "converter.h"
#include "algorithms.h"
#include "g2o_types.h"

namespace my_slam{

Tracking::Tracking() {
    gftt_detector_ = cv::GFTTDetector::create(FLAGS_nFeatures,FLAGS_gftt_qualityLevel,FLAGS_gftt_minDistance);
    LOG(INFO) << "Tracking created opencv GFTTDetector with params: (" << FLAGS_nFeatures << 
                ", " << FLAGS_gftt_qualityLevel << ", "  << FLAGS_gftt_minDistance;

    status_ = TrackingStatus::INITIALIZING;

    relative_motion_ = Eigen::Isometry3d::Identity();

}

void Tracking::AddFrame(std::shared_ptr<Frame> frame) {
    LOG(INFO) << "Step into AddFrame.";
    LogTrackingStatus();
     current_frame_ = frame;
    switch (status_) {
        case TrackingStatus::INITIALIZING: //准备初始化
             stereoInit();
            break;
        case TrackingStatus::TRACKING_GOOD: //不做事情，因为新的frame和上一帧frame相似，足够多的特征点，新相机的位姿也估计得很好
                                            //所以在这种状态下，地图点是不添加的
            break;
        case TrackingStatus::TRACKING_BAD:
            Track();
            break;
        
        case TrackingStatus::TRACKING_LOST:
            Reset();
            break;

        default:
            break;
    }

    last_frame_ = current_frame_;
    LogTrackingStatus();
}

bool Tracking::Track() {
    // 给current_frame_设置初始化的Pose
    if(last_frame_) {
         current_frame_->SetPose(relative_motion_* last_frame_->Pose());
    }
    
    TrackCurrentFrame(); //返回光流匹配到的特征点数量
    tracking_inliers_ = EstimateCurrentPose_g2o(); //估计出当前帧的位姿

    //根据测量值也就是前后帧图像的匹配情况，利用ceres框架得到内点的数量做比较
     if(tracking_inliers_ > FLAGS_nFeaturesTracking) {
        status_ = TrackingStatus::TRACKING_GOOD;
    } else if(tracking_inliers_ > FLAGS_nFeaturesTrackingBad) {
        status_ = TrackingStatus::TRACKING_BAD;
    } else {
        status_ = TrackingStatus::TRACKING_LOST;
    }
    //尝试向地图添加关键帧，如果新帧跟踪效果好，那么，整个Track的流程到这里就结束了，更新当前帧
    InsertKeyframe();

    //保存经过优化的当前帧和上一帧的相对关系
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    //TODO(李阳)：添加Viewer对关键帧的数据可视化
    
    return true;
}

/**
 * 发现是关键帧之后，将该关键帧添加到地图当中，后端发现地图添加了关键帧，开始做处理
 */
bool Tracking::InsertKeyframe() {
    if(tracking_inliers_ > FLAGS_nFeaturesKeyframe) {
        //仍然有足够多的特征内点，所以不需要添加
        return false;
    }
    //因为当前帧跟踪到的地图点数量不够，所以，认为当前帧看到了比较多新的特征点，
    //设置当前帧为关键帧。需要将这些特征点还原成为地图点并且存储在地图当中。
    current_frame_->SetKeyFrame(); 
    //调用地图指针插入关键帧
    map_->InsertKeyFrame(current_frame_);

    LOG(INFO) << "Set frame " << current_frame_->GetId() << "as key frame " << current_frame_->GetKeyFrameId();

    SetObservationsForKeyFrame();

    DetectNewLeftFeatures();

    DetectFeaturesOfRight();
    
    TriagulateNewPoints();

    //update backend map
    backend_->UpdateMap();
    
    if(viewer_) {
        viewer_->UpdateMap();
    }
    return true;

}

int Tracking::DetectFeaturesOfRight() {
    //使用LK光流估计右图特征点
    //特征点（Key Points）在当前帧左相机和右相机的列表
    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto &feature: current_frame_->GetLeftFeatures()) {
        kps_left.push_back(feature->GetKeyPoint().pt);
        //auto mapPoint = feature->GetMapPoint().lock();
        auto mapPoint = feature->GetMapPoint();
        if(mapPoint) {
            auto pixelPoint = right_camera_->World2pixel(mapPoint->Pos(),current_frame_->Pose()); //实际上还是投影到了左相机像素上
            kps_right.push_back(cv::Point2f(pixelPoint(0),pixelPoint(1)));
        } else {
            kps_right.push_back(feature->GetKeyPoint().pt);
        }
    }

    //调用Camera的接口实现对原始图像的去畸变
    cv::Mat undistorted_left = current_frame_->GetUndistortLeftImg();
    cv::Mat undistorted_right = current_frame_->GetUndistortRightImg();

    std::vector<uchar> status;
    std::vector<float> error;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,30,0.01);
    cv::Size winSize(31,31);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(undistorted_left, 
                            undistorted_right, 
                            kps_left, 
                            kps_right, 
                            status, 
                            error,
                            winSize,
                            3,
                            termcrit,
                            cv::OPTFLOW_USE_INITIAL_FLOW);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    auto time_used_opencv = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "OpticalFlow tracking left and right image time cost by opencv: " <<  time_used_opencv.count();

    //统计计算结果，并且赋值

    int nGoodFeatures = 0;

    for(size_t i = 0; i < status.size(); i++) {
        if(status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            //Feature::Ptr feature(new Feature(current_frame_,kp)); //像素点
            auto feature = Feature::CreateNewFeature(current_frame_,kp);
            std::shared_ptr<Feature> sfeature = std::move(feature);
            current_frame_->AddRightFeature(sfeature);
            nGoodFeatures++;
        }
        else {
            current_frame_->AddRightFeature(nullptr);
        }
    }

    LOG(INFO) << "Found " << nGoodFeatures << "2D feature points in the current frame.";
       return nGoodFeatures;
}

void Tracking::TriagulateNewPoints() {
    //过滤current_frame_当中左目相机特征点，找到那些没有3D地图点的特征点
    std::vector<bool> unT; //存储feature是否被三角化
    std::vector<cv::KeyPoint> UKs_left,UKs_right; // UK意思是Untriagulated KeyPoint
    for(int i=0; i < current_frame_->GetLeftFeatureNum();i++) {
        if(current_frame_->GetLeftFeatureByindex(i)->GetMapPoint() == nullptr && 
            current_frame_->GetRightFeatureByindex(i)!=nullptr) {
                UKs_left.push_back(current_frame_->GetLeftFeatureByindex(i)->GetKeyPoint());
                UKs_right.push_back(current_frame_->GetRightFeatureByindex(i)->GetKeyPoint());
            }
    }

    std::vector<cv::Point3d>  triagulated_points;

    Eigen::Isometry3d left_T_c_b = current_frame_->GetLeftCamera()->GetT_c_b();
    Eigen::Isometry3d right_T_c_b = current_frame_->GetRightCamera()->GetT_c_b();
    cv::Mat left_T_c_b_cv = Converter::toCVMatf34(left_T_c_b);
    cv::Mat right_T_c_b_cv = Converter::toCVMatf34(right_T_c_b);
    
    if(!Triangulation(UKs_left,UKs_right,left_T_c_b_cv,right_T_c_b_cv,triagulated_points)) {
        LOG(ERROR) << "Triagulate new points failed.";
    }

    Eigen::Isometry3d Twc = current_frame_->Pose().inverse();
    for(int i=0;i<triagulated_points.size();i++) {
        if(triagulated_points[i].z > 0) {
            Vec3 pw(triagulated_points[i].x,triagulated_points[i].y,triagulated_points[i].z);
            pw = Twc * pw; //将Frame坐标系下的坐标转换为世界坐标系
            auto newMapPoint = MapPoint::CreateNewMapPoint(pw);
            newMapPoint->AddObservation(current_frame_->GetLeftFeatureByindex(i));
            newMapPoint->AddObservation(current_frame_->GetRightFeatureByindex(i));
            std::shared_ptr<MapPoint> smp = std::move(newMapPoint);
            if(smp) {
                current_frame_->GetLeftFeatureByindex(i)->SetMapPoint(smp);
                current_frame_->GetRightFeatureByindex(i)->SetMapPoint(smp);
                map_->InsertMapPoint(smp);
            }
            
        }
    }
    

}

/**
 * 先将已检测出来的Feature像素点周围像素抹去，剩下的像素通过opencv检测并且
 * 保存到当前帧的features_left_当中。实际上，在下一次新帧到来时，current_frame_
 * 变为last_frame_, 里面已经包含了这部分的特征点。
 */
int Tracking::DetectNewLeftFeatures() {
    cv::Mat maskedImg(current_frame_->GetUndistortLeftImg().size(),CV_8UC1,255); //这是一张纯黑色的图像
    for(auto& feature : current_frame_->GetLeftFeatures()) {
        double half_rect = FLAGS_maskNeigbourPixels;
        cv::rectangle(maskedImg,
                      feature->GetKeyPoint().pt - cv::Point2f(half_rect,half_rect),
                      feature->GetKeyPoint().pt + cv::Point2f(half_rect,half_rect),
                      0,
                      cv::FILLED);
    }

    std::vector<cv::KeyPoint> newDetected;
    gftt_detector_->detect(current_frame_->GetUndistortLeftImg(),newDetected,maskedImg);


    for(size_t i = 0; i < newDetected.size(); i++) {
        auto feature = Feature::CreateNewFeature(current_frame_,newDetected[i]);
        std::shared_ptr<Feature> sfeature = std::move(feature);
        //std::shared_ptr<Feature> feature = std::make_shared<Feature>(current_frame_,newDetected[i]);
        current_frame_->AddLeftFeature(sfeature);
    }

    LOG(INFO) << "New detected " << newDetected.size() << " feature points.";
    return newDetected.size();
}

void Tracking::LogTrackingStatus() {
    switch (status_)
    {
    case TrackingStatus::INITIALIZING:
        LOG(INFO) << "Tracking INITIALIZING";
        break;
    case TrackingStatus::TRACKING_GOOD:
        LOG(INFO) << "Tracking TRACKING_GOOD";
        break;
    case TrackingStatus::TRACKING_BAD:
        LOG(INFO) << "Tracking TRACKING_BAD";
        break;
    case TrackingStatus::TRACKING_LOST:
        LOG(INFO) << "Tracking TRACKING_LOST";
        break;

    default:
        break;
    }
}

/**
 * 在判断为新关键帧之后，需要将关键帧里面的地图点对应的观测（Featrure）更新一下
 * 这样在后端拿到
 * 
 */
void Tracking::SetObservationsForKeyFrame() {
    for(auto& feature : current_frame_->GetLeftFeatures()) {
        //auto map = feature->GetMapPoint().lock();
        auto map = feature->GetMapPoint();
        if(map) {
            map->AddObservation(feature);
        }
    }
}

TrackingStatus Tracking::GetStatus() {
 return status_;
}

bool Tracking::stereoInit() {
    //先检测出左图特征点
    int numLeft = DetectNewLeftFeatures();
    //再从右图检测出和左图对应的特征点
    int numRight = DetectFeaturesOfRight();
    if(numRight > numLeft) {
        LOG(ERROR) << "In stereo init, right features should not be larger than left." 
                    << "Left: " << numLeft << " right: " << numRight;
        return false;
    }
    
     bool success = BuildInitMap();
    if(success) {
        status_ = TrackingStatus::TRACKING_BAD;
        // if(viewer_) {
        //     viewer_->AddCurrentFrame(current_frame_);
        //     viewer_->UpdateMap();
        // }
    }

    return true;
}

bool Tracking::SetMap(std::shared_ptr<Map> map) {
    if(map) {
        map_ = map;
        return true;
    }else {
        return false;
    }
}

bool Tracking::SetBackend(std::shared_ptr<Backend> backend) {
    if(backend) {
        backend_ = backend;
        return true;
    } else {
        return false;
    }
}

//TODO(李阳)：需要在实例化Camera对象时候调用该方法设置对象是左相机还是右相机
bool Tracking::SetCameras(Camera::Ptr left, Camera::Ptr right) {
    if(!left || !right) {
        return false;
    }
    left_camera_ = left;
    right_camera_ = right;
    return true;
}

bool Tracking::Reset() {
    return true;
}

bool Tracking::SetViewer(std::shared_ptr<Viewer> viewer) {
    viewer_ = viewer;
}

/*
* 使用左侧相机的图像序列，尝试匹配当前帧和上一帧的特征点，将匹配到的特征点对应的
* 三维地图点保存到当前帧，用于下一步估计当前帧的位姿。这里不是所有的Feature对应的
* 地图点都有效，很多都是无效的地图点。
*/
int Tracking::TrackCurrentFrame() {
    //特征点（Key Points）在当前帧和上一帧的列表
    std::vector<cv::Point2f> kpsLast, kpsCurrent;
    auto left_features = last_frame_->GetLeftFeatures();
    for (const auto& feature: left_features) {
        if(feature->GetMapPoint()) { // 关联的地图点 {
            //auto mapPoint = feature->GetMapPoint().lock(); // Last Frame的地图点
            auto mapPoint = feature->GetMapPoint(); // Last Frame的地图点
            auto pixelPoint = left_camera_->World2pixel(mapPoint->Pos(),current_frame_->Pose());
            kpsLast.push_back(feature->GetKeyPoint().pt);
            kpsCurrent.push_back(cv::Point2f(pixelPoint(0),pixelPoint(1)));
            //LOG(INFO) << "Tracking next frame, map point project to pixel: [" << pixelPoint(0) << "," << pixelPoint(1) << "]";
        } else {
            kpsLast.push_back(feature->GetKeyPoint().pt);
            kpsCurrent.push_back(feature->GetKeyPoint().pt);
        }
    }

    //调用Camera的接口实现对原始图像的去畸变
    cv::Mat undistorted_last = last_frame_->GetUndistortLeftImg();
    cv::Mat undistorted_current = current_frame_->GetUndistortLeftImg();

    std::vector<uchar> status;
    std::vector<float> error;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,30,0.01);
    cv::Size winSize(31,31);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(undistorted_last, 
                            undistorted_current, 
                            kpsLast, 
                            kpsCurrent, 
                            status, 
                            error,
                            winSize,
                            3,
                            termcrit,
                            cv::OPTFLOW_USE_INITIAL_FLOW);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    auto time_used_opencv = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "OpticalFlow tracking time cost by opencv: " <<  time_used_opencv.count();

    //统计计算结果，并且赋值

    int nGoodFeatures = 0;

    for(size_t i = 0; i < status.size(); i++) {
        if(status[i]) {
            cv::KeyPoint kp(kpsCurrent[i], 7);
            //Feature::Ptr feature(new Feature(current_frame_,kp)); //像素点
            auto curfeature = Feature::CreateNewFeature(current_frame_,kp);
            std::shared_ptr<Feature> sCurfeature = std::move(curfeature);
            auto sLastFeature = last_frame_->GetLeftFeatureByindex(i);
            std::shared_ptr<MapPoint> new_mp = sLastFeature->GetMapPoint();
            if(new_mp) {
                sCurfeature->SetMapPoint(new_mp); //地图点在前后帧之间没有变化，在feature中只有KeyPoint
            }
            current_frame_->AddLeftFeature(sCurfeature);
            nGoodFeatures++;
        }
    }

    LOG(INFO) << "Found " << nGoodFeatures << "2D feature points in the current frame.";
    return nGoodFeatures;
    // return 1;
}


int Tracking::EstimateCurrentPose_g2o() {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // 相机参数顶点
    std::shared_ptr<g2o::CameraParameters> cam_params_left
        = std::make_shared<g2o::CameraParameters>(current_frame_->GetLeftCamera()->K_eigen()(0,0), principal_point, 0.);

    cam_params->setId(0);


    std::shared_ptr<g2o::VertexSE3Expmap> vertex_pose 
                                    = std::make_shared<g2o::VertexSE3Expmap>();

    vertex_pose->setId(0);
    vertex_pose->setEstimate(Converter::toSE3Quat(current_frame_->Pose()));
    optimizer.addVertex(vertex_pose.get());

    // edges
    int index = 1;
    std::vector<std::shared_ptr<g2o::EdgeProjectXYZ2UV>> edges;
    std::vector<std::shared_ptr<Feature>> features;
    for (size_t i = 0; i < current_frame_->GetLeftFeatureNum(); ++i) {
        auto mp = current_frame_->GetLeftFeatureByindex(i)->GetMapPoint();
        if (mp) {
            features.push_back(current_frame_->GetLeftFeatureByindex(i));
            std::shared_ptr<g2o::EdgeProjectXYZ2UV> edge = std::make_shared<g2o::EdgeProjectXYZ2UV>();
            edge->fx = current_frame_->GetLeftCamera()->K_eigen()(0,0);
            edge->fy = current_frame_->GetLeftCamera()->K_eigen()(1,1);
            edge->cx = current_frame_->GetLeftCamera()->K_eigen()(0,2);
            edge->cy = current_frame_->GetLeftCamera()->K_eigen()(1,2);
            edge->setId(index);
            //添加相机顶点
            edge->setVertex(1, vertex_pose.get());
            //添加地图顶点
            std::shared_ptr<g2o::VertexSBAPointXYZ> vMapPoint = std::make_shared<g2o::VertexSBAPointXYZ>();
            vMapPoint->setId(mp->GetId());
            vMapPoint->setEstimate(mp->Pos());
            edge->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(vMapPoint.get()));
            edge->setMeasurement(
                Converter::toVector2d(current_frame_->GetLeftFeatureByindex(i)->GetKeyPoint().pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge.get());
            index++;
        }
    }

    // estimate the Pose the determine the outliers
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(Converter::toSE3Quat(current_frame_->Pose()));
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->IsOutlier()) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                features[i]->SetOutlier(true);
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->SetOutlier(false);
                e->setLevel(0);
            };

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    // Set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

    for (auto &feat : features) {
        if (feat->IsOutlier()) {
            feat->GetMapPoint().reset();
            feat->SetOutlier(false);  // maybe we can still use it in future
        }
    }
    return features.size() - cnt_outlier;
}

/*
* 在提取左相机的前后帧对应的特征点之后，对相机的位姿进行估计。使用BA的方法，调用ceres库完成。
* 统计内点的数量，后面会根据内点的数量来决定跟踪效果的好坏，gflag里面配置了好坏的判断标准。
*/
int Tracking::EstimateCurrentPose() {
    //create problem
    ceres::Problem problem;
    //loss function for filter out outliers
    ceres::LossFunction* loss_function = NULL;

    //提取Isometry3d中的平移和旋转
    //平移
    // double trans_x = current_frame_->Pose().translation().x();
    // double trans_y = current_frame_->Pose().translation().y();
    // double trans_z = current_frame_->Pose().translation().z();

    Eigen::Vector<double, 6> vec = isometry2Params(current_frame_->Pose());
    double ceres_rot[3] = {vec[0],vec[1],vec[2]};
    double ceres_trans[3] = {vec[3],vec[4],vec[5]};
    //获取用于重投影的相机内参数
    Mat33 K_eigen = left_camera_->K_eigen();
    
    int length = current_frame_->GetLeftFeatureNum();
     for(int i = 0; i < current_frame_->GetLeftFeatureNum(); ++i) {
        auto& feature = current_frame_->GetLeftFeatureByindex(i);
        //获取地图点
        //auto mp = feature->GetMapPoint().lock();
        auto mp = feature->GetMapPoint();
        if(!mp) {
            continue;
        }
        Vec3 mp_eigen = mp->Pos();
        //获取对应的feature
        cv::KeyPoint kp = feature->GetKeyPoint();
        Vec2 kp_eigen(kp.pt.x,kp.pt.y);
        //创建一个CostFucntion块
        ceres::CostFunction* cost_function =
        ReprojectionError::Create(mp_eigen,kp_eigen,K_eigen);//传递测量值到costFunction
        //根据CostFunction 产生Residual Block 
        problem.AddResidualBlock(cost_function,loss_function,ceres_rot,ceres_trans); //传递外参数
    }

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    

    //extract result from ceres solver
    double ceres_results[6];
    ceres_results[0] = ceres_rot[0];
    ceres_results[1] = ceres_rot[1];
    ceres_results[2] = ceres_rot[2];

    ceres_results[3] = ceres_trans[3];
    ceres_results[4] = ceres_trans[4];
    ceres_results[5] = ceres_trans[5];


    Eigen::Isometry3d result = Eigen::params2Isometry(ceres_results);

    //将优化结果赋值到current frame
    current_frame_->SetPose(result);    
}

//注意，在这个阶段，因为我们不知道base_link在世界坐标系下的姿态，所以，我们把当前三角化成功
//的点的参考原点就是当前base_link的原点。
bool Tracking::BuildInitMap() {
    //获取base_link到左右相机的转换关系
    Eigen::Isometry3d left_T_c_b = current_frame_->GetLeftCamera()->GetT_c_b();
    Eigen::Isometry3d right_T_c_b = current_frame_->GetRightCamera()->GetT_c_b();
    cv::Mat left_T_c_b_cv = Converter::toCVMatf34(left_T_c_b);
    cv::Mat right_T_c_b_cv = Converter::toCVMatf34(right_T_c_b);
    LOG(INFO) << "left_T_c_b_cv: \n " << left_T_c_b_cv;
    LOG(INFO) << "right_T_c_b_cv: \n" << right_T_c_b_cv;
    std::vector<cv::KeyPoint> keypoint_left, keypoint_right;
    std::vector<cv::Point3d> triagulated; //base_link 坐标系下的三角化点
    std::vector<int> validPointIndex;
    for(int i = 0; i < current_frame_->GetLeftFeatureNum();i++) {
        if(current_frame_->GetLeftFeatureByindex(i) == nullptr
            || current_frame_->GetRightFeatureByindex(i) == nullptr) {
            continue;
        }
        auto kp_left = current_frame_->GetLeftFeatureByindex(i)->GetKeyPoint();
        auto kp_right = current_frame_->GetRightFeatureByindex(i)->GetKeyPoint();
        keypoint_left.push_back(kp_left);
        keypoint_right.push_back(kp_right);
        validPointIndex.push_back(i);
    }
      int validCnt = 0; //成功三角化的地图点数量
    if(Triangulation(keypoint_left,keypoint_right,left_T_c_b_cv,right_T_c_b_cv,triagulated)) {
        //产生了地图点，添加到地图中
        for(int i = 0; i < triagulated.size();i++) {
            if(triagulated[i].z < 0) continue;
            
            auto new_mp = MapPoint::CreateNewMapPoint(Vec3(triagulated[i].x,triagulated[i].y,triagulated[i].z));
            new_mp->AddObservation(current_frame_->GetLeftFeatureByindex(validPointIndex[i]));
            new_mp->AddObservation(current_frame_->GetRightFeatureByindex(validPointIndex[i]));
            std::shared_ptr<MapPoint> smp = std::move(new_mp);
            map_->InsertMapPoint(smp);
            auto cur_leftFea = current_frame_->GetLeftFeatureByindex(validPointIndex[i]);
            auto cur_rightFea = current_frame_->GetRightFeatureByindex(validPointIndex[i]);
            cur_leftFea->SetMapPoint(smp);
            cur_rightFea->SetMapPoint(smp);
            validCnt++;
        }
    }

    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    backend_->UpdateMap();

    LOG(INFO) << "Building Init map with valid " << validCnt << " map points.";
    return true;
}

} // namespace my_slam