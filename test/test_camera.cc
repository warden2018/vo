#include <iostream>
#include "catch.cc"
#include "camera.h"
#include "types.h"
#include <Eigen/Core>

TEST_CASE("CameraTest.Transform","[Eigen]") {
    //初始化Camera对象
    // PinpoleIntrinsics intrinsics;
    // intrinsics.cx_ = 300;
    // intrinsics.cy_ = 200;
    // intrinsics.fx_ = 150;
    // intrinsics.fy_ = 100;
    // intrinsics.k1_ = 0.1;
    // intrinsics.k2_ = 0;
    // intrinsics.p1_ = 0;
    // intrinsics.p2_ = 0;
    // double baseline = 0.15;
    //Eigen::Isometry3d trans = Eigen::Isometry3d::Identity();
    my_slam::Camera camera(std::string("/home/minerva/minerva/Dai_Yang/catkin_ws/src/my_slam/params/zed2i_left.yaml"));
    camera.Init(my_slam::Camera::CameraSide::LEFT);
    //初始化测试数据
    Eigen::AngleAxisd rotation_vector(M_PI/2, Eigen::Vector3d (0,0,1));

    Eigen::Isometry3d T_c_w = Eigen::Isometry3d::Identity();
    T_c_w.rotate(rotation_vector);
    T_c_w.pretranslate(Vec3(0,0,1));

    Vec3 before(1,0,0);
    Vec3 after = camera.World2camera(before,T_c_w);
    std::cout << "Before world2camera world pose is: " << before.transpose() << std::endl;
    std::cout << "After world2camera world pose is: " << after.transpose() << std::endl;
    REQUIRE((after - Vec3(0,1,1)).norm() < 0.001);



    //开始测试


}
