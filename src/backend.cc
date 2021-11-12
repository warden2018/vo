#include "backend.h"
#include <map>
#include "g2o_types.h"
#include "gflags.h"
#include "converter.h"
#include "feature.h"
#include "g2o_types.h"

namespace my_slam {

Backend::Backend() {
    backend_running_.store(true);
    backend_thread_ = std::thread(std::bind(&Backend::BakcendLoop,this));
}

void Backend::UpdateMap() {
    std::lock_guard<std::mutex> lock(mutex_);
    map_updated_.notify_one();
}

void Backend::SetMap(std::shared_ptr<Map> map) {
    map_ = map;
}

void Backend::BakcendLoop() {
    while(backend_running_.load()) {
        std::unique_lock<std::mutex> lock(mutex_);
        map_updated_.wait(lock);

        Map::KeyFramesType active_kfs = map_->GetAllActiveKeyFrames();
        Map::MapPointsType active_mps = map_->GetAllActiveMapPoints();
        //Optimize_g2o(active_mps,active_kfs);
        // Optimize_ceres(active_mps,active_kfs);
    }
}

bool Backend::Optimize_g2o(Map::MapPointsType& mps,Map::KeyFramesType& kfs) {
    //初始化和设定g2o优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver; //这里的6,3代表了相机姿态是6自由度，3是路标点的姿态
    if (FLAGS_useDense) { //如果是稠密，那么使用LinearSolverDense进行线性方程的求解
        linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    } else {
        using BaLinearSolver = g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>; //否则，使用LinearSolverCholmod
        linearSolver = g2o::make_unique<BaLinearSolver>();
    }

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );//传入BlockSolver创建一个OptimizationAlgorithmLevenberg类型的OptimizationAlgorithm
    optimizer.setAlgorithm(solver);

    //开始添加顶点（包括了Frame的顶点和地图点顶点）
    //为了保证kfs和地图点的id不一样，需要做一个id的计数
    
    unsigned long max_kfs_id = 0;
    std::map<unsigned long, g2o::VertexSE3Expmap* > vertices_kfs; //用于保存加入到图优化当中的关键帧节点id和对应的关键帧
    //十四讲里面，有一个局部的map：vertices和vertices_landmarks存储结果，其实也可以直接去optimizer里面获取,ORBSLAM2里面是这么做
    for(auto& kf : kfs) {
        auto keyframe = kf.second;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(keyframe->Pose()));
        vSE3->setId(2 + keyframe->GetId());
        vertices_kfs.insert({keyframe->GetId(),vSE3}); //通过KF的id记录这个顶点
        optimizer.addVertex(vSE3);

        if((keyframe->GetId() + 2) > max_kfs_id) {
            max_kfs_id = keyframe->GetId() + 2;
        }
    } // end Key Frame

    //鲁棒核函数的参数
    const double huber2D = sqrt(FLAGS_2Dhuber95);
    const double huber3D = sqrt(FLAGS_3Dhuber95);

    //开始添加地图点到顶点
    bool currentMpIsInGraph = false;
    std::map<unsigned long,g2o::VertexSBAPointXYZ* > vertices_mps;
    for(auto& mp : mps) {
        auto mappoint = mp.second;
        if(mappoint->IsOutLier()) continue;
        currentMpIsInGraph = false;
        unsigned long mp_id = mappoint->GetId();
        auto observations = mappoint->GetObservations();
        //因为这里需要遍历某一个地图点关联的所有观测值，所以在添加地图顶点时候，需要注意避免重复添加
        for(auto& obser : observations) {
            if(obser.lock()==nullptr) continue;
            auto feature = obser.lock();
            if(feature->IsOutlier() || feature->GetFrame().lock() == nullptr) continue;
            //添加地图点
            if(!currentMpIsInGraph) {
                g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
                const int id = max_kfs_id + mappoint->GetId() + 1; //关键帧节点的id在前面，地图点在后面
                vPoint->setId(id);
                vPoint->setEstimate(mappoint->Pos());
                optimizer.addVertex(vPoint);
                vertices_mps.insert({mp_id,vPoint}); //通过MapPoint的id记录这个顶点
                currentMpIsInGraph = true;
            }
            //添加观测边
            auto frame = feature->GetFrame().lock(); //每一个Feature一定对应一个KF
            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ(); //3D地图点投影到像素上，和实际测量值之间的误差边
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(vertices_mps.at(mp_id))); //当前的地图点
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(vertices_kfs.at(frame->GetId()))); //当前的关键帧
            e->setMeasurement(Converter::toVector2d(feature->GetKeyPoint().pt)); //setMeasurement是Eigen的类型
            e->setInformation(Mat22::Identity()); //信息矩阵是单位阵，如果前端使用的是金字塔，那么，对于高层的金字塔，需要降低这个数值
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(huber2D);
            //判断特征点在左图还是右图
            if(feature->IsOnLeft()) { //在左图
                e->fx = frame->GetLeftCamera()->K_eigen()(0,0);
                e->fy = frame->GetLeftCamera()->K_eigen()(1,1);
                e->cx = frame->GetLeftCamera()->K_eigen()(0,2);
                e->cy = frame->GetLeftCamera()->K_eigen()(1,2);
            } else { //在右图
                e->fx = frame->GetRightCamera()->K_eigen()(0,0);
                e->fy = frame->GetRightCamera()->K_eigen()(1,1);
                e->cx = frame->GetRightCamera()->K_eigen()(0,2);
                e->cy = frame->GetRightCamera()->K_eigen()(1,2);
            }

            optimizer.addEdge(e); //添加误差边，这里只是最小化重投影的误差，所以是2D的向量
        } //end observation

    } // end map point


    //开始做优化
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    //获取优化结果，更新地图点坐标和关键帧位姿


}

bool Backend::Optimize_ceres(Map::MapPointsType& mps,Map::KeyFramesType& kfs) {
    
}



void Backend::SetCameras(Camera::Ptr left, Camera::Ptr right) {
    left_cam_ = left;
    right_cam_ = right;
}

} //namespace my_slam