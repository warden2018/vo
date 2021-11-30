#include <iostream>
#include <opencv2/opencv.hpp>
#include "catch.cc"
#include "feature.h"
#include "types.h"
#include <Eigen/Core>
#include "frame.h"
#include "map_point.h"

using namespace my_slam;
//该case是测试shared_ptr和weak_ptr的引用计数、内存块是否存在
TEST_CASE("Feature","") {
    //模拟先对2D特征检测，保存好Feature
    //auto new_frame = Frame::CreateFrame();
    //std::shared_ptr<Frame> sfr = std::move(new_frame);
    // cv::Point2f pc(100,100);
    // cv::KeyPoint kp(pc,7);
    // auto feature = Feature::CreateNewFeature(sfr,kp);
    // std::shared_ptr<Feature> sfeature = std::move(feature);
    // //Feature::Ptr feature(new Feature(sfr,kp)); //像素点
    // sfr->AddLeftFeature(sfeature);
    // //再添加一个Feature
    // cv::Point2f pc2(200,200);
    // cv::KeyPoint kp2(pc2,7);
    // auto feature2 = Feature::CreateNewFeature(sfr,kp2);
    // std::shared_ptr<Feature> sfeature2 = std::move(feature2);
    // sfr->AddLeftFeature(sfeature2);
    // //再根据三角化之后的3D点对这个Feature设置MapPoint
    // long countNum = -1;
    // auto new_mp = MapPoint::CreateNewMapPoint();
    // new_mp->SetPos(Vec3(0,0,0));
    // new_mp->AddObservation(sfr->GetLeftFeatureByindex(0));
    // std::shared_ptr<MapPoint> smp = std::move(new_mp);
    // //产生另外一个MapPoint
    //  auto new_mp2 = MapPoint::CreateNewMapPoint();
    // new_mp2->SetPos(Vec3(1,0,0));
    // new_mp2->AddObservation(sfr->GetLeftFeatureByindex(1));
    // std::shared_ptr<MapPoint> smp2 = std::move(new_mp2);
    // //new_mp->AddObservation(new_frame->GetRightFeatureByindex(0));
    // sfr->GetLeftFeatureByindex(0)->SetMapPoint(smp);
    // sfr->GetLeftFeatureByindex(1)->SetMapPoint(smp2);

    //for 循环先创建五个left feature
    // for(int i = 0; i < 5; i++) {
    //     cv::Point2f pc(150 + i,100);
    //     cv::KeyPoint kp(pc,7);
    //     auto feature = Feature::CreateNewFeature(sfr,kp);
    //     std::shared_ptr<Feature> sfeature = std::move(feature);
    //     //Feature::Ptr feature(new Feature(sfr,kp)); //像素点
    //     sfr->AddLeftFeature(sfeature);
    // }

    // auto new_mp1 = MapPoint::CreateNewMapPoint(Vec3(10,20,1));
    // auto new_mp2 = MapPoint::CreateNewMapPoint(Vec3(10,20,2));
    // auto new_mp3 = MapPoint::CreateNewMapPoint(Vec3(10,20,3));
    // auto new_mp4 = MapPoint::CreateNewMapPoint(Vec3(10,20,4));
    // auto new_mp5 = MapPoint::CreateNewMapPoint(Vec3(10,20,5));
    // std::vector<std::shared_ptr<MapPoint>> new_mps;
    // new_mps.push_back(std::move(new_mp1));
    // new_mps.push_back(std::move(new_mp2));
    // new_mps.push_back(std::move(new_mp3));
    // new_mps.push_back(std::move(new_mp4));
    // new_mps.push_back(std::move(new_mp5));
    // //创建五个MapPoint
    // for(int i = 0; i < 5; i++) {
    //     //std::unique_ptr<MapPoint> new_mp = MapPoint::CreateNewMapPoint(Vec3(10,20,1 + i));
    //     //auto new_smp = new_mps.at(i);
    //     std::unique_ptr<MapPoint> new_mp(new MapPoint(Vec3(10,20,1 + i)));
    //     std::shared_ptr<MapPoint> smp = std::move(new_mp);
    //     new_mps.push_back(smp);
    //     smp->AddObservation(sfr->GetLeftFeatureByindex(i));
    //     auto cur_leftFea = sfr->GetLeftFeatureByindex(i);
    //     cur_leftFea->SetMapPoint(smp);
    // }

    // //读取五个Feature当中的MapPoint
    // auto left_features = sfr->GetLeftFeatures();
    // for (const auto& feature: left_features) {
    //     auto wmp = feature->GetMapPoint();
    //     std::shared_ptr<MapPoint> smp = wmp; 
    //     if(smp != nullptr) {
    //         std::cout << "MapPoint z: " << smp->Pos().z() << std::endl;
    //     }
    //     REQUIRE(smp != nullptr);
    // }

    // auto mp2 = smp;
    // countNum = mp2.use_count();
    // auto mp3 = smp;
    // countNum = mp3.use_count();
    // //获取Frame中的Feature
    // std::weak_ptr<MapPoint> mppoint;
    // mppoint = sfr->GetLeftFeatureByindex(0)->GetMapPoint(); //GetMapPoint返回的是std::weak_ptr
    // countNum = mppoint.use_count();
    // if(mppoint.lock()) {
    //     std::cout << "map point(weak_ptr) in feature can be found.";
    // }

    // //第二个
    // std::weak_ptr<MapPoint> mppoint2;
    // mppoint2 = sfr->GetLeftFeatureByindex(1)->GetMapPoint(); //GetMapPoint返回的是std::weak_ptr
    // countNum = mppoint2.use_count();
    // if(mppoint2.lock()) {
    //     std::cout << "map point(weak_ptr) in feature can be found.";
    // }

}