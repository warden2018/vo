cmake_minimum_required(VERSION 2.8.3)
project(my_slam)

set(CMAKE_CXX_FLAGS "-std=c++14")
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
#SET(D_GLIBCXX_USE_CXX11_ABI,"0")
#add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
        roscpp 
        rospy 
        std_msgs 
        genmsg 
        message_filters 
        cv_bridge 
        image_transport)


## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

############### dependencies ######################
#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV version: " ${OpenCV_VERSION})
#sophus
find_package(Sophus REQUIRED)

#GFlag
SET(gflags_DIR "/usr/local/lib/cmake/gflags/")
find_package(gflags REQUIRED)

# Pangolin
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

# g2o
set(G2O_DIR "/home/minerva/Downloads/3rd/g2o-20201223_git/")
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

#Eigen
include_directories("/usr/local/include/eigen3")

#ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

#csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

#glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})


# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})


#yaml-cpp
#find_package(yaml-cpp REQUIRED)
#include_directories(${YAML_CPP_INCLUDE_DIRS})
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
message(STATUS "Yaml: ${yaml-cpp_FOUND}")


set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        #${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        ${CERES_LIBRARIES}
        )

## Build my_slam
include_directories(include 
                    ${catkin_INCLUDE_DIRS}
                    common
                    3rd/DBoW2/DBoW2
                    test)

# libs including src/ and gflag
add_library(myslamLib SHARED
        src/map_point.cc
        src/feature.cc
        src/frame.cc
        src/camera.cc
        src/tracking.cc
        src/visual_odometry.cc
        src/converter.cc
        src/map.cc
        src/viewer.cc
        src/backend.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/key_frame.cc
        src/key_frameDatabase.cc
        common/gflags.h
        common/gflags.cc
        
        )

message(STATUS "Yaml lib: ${YAML_CPP_LIBRARIES}")
add_library(yaml_cpp_lib STATIC IMPORTED)
set_target_properties(yaml_cpp_lib PROPERTIES IMPORTED_LOCATION /home/minerva/Downloads/3rd/yaml-cpp-yaml-cpp-0.7.0/build/libyaml-cpp.a)
target_link_libraries(myslamLib yaml_cpp_lib ${THIRD_PARTY_LIBS} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} DBoW2)

# binary
add_executable(my_slam apps/my_slam.cc)
target_link_libraries(my_slam ${catkin_LIBRARIES} myslamLib)

#unit test
add_executable(test_camera test/test_camera.cc src/camera.cc common/catch.hpp)
target_link_libraries(test_camera myslamLib ${THIRD_PARTY_LIBS})

add_executable(test_feature test/test_feature.cc src/feature.cc common/catch.hpp)
target_link_libraries(test_feature myslamLib ${THIRD_PARTY_LIBS})