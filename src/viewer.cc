#include "viewer.h"
#include "feature.h"
#include "frame.h"
#include "camera.h"
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace my_slam {

Viewer::Viewer() {
    viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
}

void Viewer::Close() {
    viewer_running_ = false;
    viewer_thread_.join();
}

void Viewer::AddCurrentFrame(std::shared_ptr<Frame> current_frame) {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    current_frame_ = current_frame;
}

void Viewer::UpdateMap() {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    assert(map_ != nullptr);
    active_keyframes_ = map_->GetAllActiveKeyFrames();
    active_landmarks_ = map_->GetAllActiveMapPoints();
    map_updated_ = true;
}

void Viewer::ThreadLoop() {
    pangolin::CreateWindowAndBind("MySLAM", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState vis_camera(
        pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& vis_display =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(vis_camera));

    const float blue[3] = {0, 0, 1};
    const float green[3] = {0, 1, 0};

    while (!pangolin::ShouldQuit() && viewer_running_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);

        std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        if (current_frame_) {
            DrawFrame(current_frame_, green);
            FollowCurrentFrame(vis_camera);

            cv::Mat img_left = PlotFrameLeftImage();
            cv::Mat img_right = PlotFrameRightImage();
            cv::imshow("Left image", img_left);
            cv::imshow("Right image", img_right);
            cv::waitKey(1);
        }

        if (map_) {
            DrawMapPoints();
        }

        pangolin::FinishFrame();
        usleep(5000);
    }

    LOG(INFO) << "Stop viewer";
}

void Viewer::ValidateTriangulation() {
    cv::Mat img_left = PlotFrameLeftReproj();
    cv::Mat img_right = PlotFrameRightReproj();
    cv::imshow("Validation Reprojection Left", img_left);
    cv::imshow("Validation Reprojection Right", img_right);
    cv::waitKey(1);
}

void Viewer::Put3DInfo2Img(const Vec3& pos, const cv::Point2f& location, const int& id, const cv::Mat& img) {
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 0.4;
    int thickness = 3;

    std::ostringstream strs;
    strs << std::setprecision(3)
        << id << "(" << pos.x()
        << "," << pos.y()
        << "," << pos.z() << ")";
    std::string str = strs.str();

    cv::putText(img,str,location,font_face,font_scale,cv::Scalar(255,0,0));
}

cv::Mat Viewer::PlotFrameRightImage() {
    cv::Mat img_out = current_frame_->GetUndistortRightImg().clone();
    //cv::cvtColor(current_frame_->GetUndistortLeftImg(), img_out, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < current_frame_->GetRightFeatures().size(); ++i) {
        if (current_frame_->GetRightFeatures()[i]->GetMapPoint()) {
            auto feat = current_frame_->GetRightFeatures()[i];
            Put3DInfo2Img(feat->GetMapPoint()->Pos(),feat->GetKeyPoint().pt,i,img_out);
            cv::circle(img_out, feat->GetKeyPoint().pt, 2, cv::Scalar(0, 250, 0),
                       2);
            int font_face = cv::FONT_HERSHEY_COMPLEX;
            double font_scale = 0.4;
            int thickness = 3;

            std::ostringstream strs;
            strs << i;
            std::string str = strs.str();

            cv::putText(img_out,str,feat->GetKeyPoint().pt,font_face,font_scale,cv::Scalar(0,250,0));
        }
    }
    return img_out;
}

cv::Mat Viewer::PlotFrameLeftImage() {
    cv::Mat img_out = current_frame_->GetUndistortLeftImg().clone();
    //cv::cvtColor(current_frame_->GetUndistortLeftImg(), img_out, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < current_frame_->GetLeftFeatures().size(); ++i) {
        if (current_frame_->GetLeftFeatures()[i]->GetMapPoint()) {
            auto feat = current_frame_->GetLeftFeatures()[i];
            Put3DInfo2Img(feat->GetMapPoint()->Pos(),feat->GetKeyPoint().pt,i,img_out);
            cv::circle(img_out, feat->GetKeyPoint().pt, 2, cv::Scalar(0, 250, 0),
                       2);
            
            int font_face = cv::FONT_HERSHEY_COMPLEX;
            double font_scale = 0.4;
            int thickness = 3;

            std::ostringstream strs;
            strs << i;
            std::string str = strs.str();

            cv::putText(img_out,str,feat->GetKeyPoint().pt,font_face,font_scale,cv::Scalar(0,250,0));
        }
    }
    return img_out;
}

cv::Mat Viewer::PlotFrameLeftReproj() {
    cv::Mat img_out = current_frame_->GetUndistortLeftImg().clone();
    LOG(INFO) << "Left Img size. rows: " << img_out.size[0] << ", cols: " << img_out.size[1];
    //cv::cvtColor(current_frame_->GetUndistortLeftImg(), img_out, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < current_frame_->GetLeftFeatures().size(); ++i) {
        if (current_frame_->GetLeftFeatures()[i]->GetMapPoint()) {
            auto feat = current_frame_->GetLeftFeatures()[i];
            //Put3DInfo2Img(feat->GetMapPoint()->Pos(),feat->GetKeyPoint().pt,i,img_out);
            auto reproj_left = current_frame_->GetLeftCamera()->World2pixel(feat->GetMapPoint()->Pos(),current_frame_->Pose());
            LOG(INFO) << "Left camera reproj. pixel coordinates: " << reproj_left(0) << ", " << reproj_left(1);
            if(reproj_left(0)<=img_out.size[1] && reproj_left(1)<=img_out.size[0]) {
                cv::circle(img_out, cv::Point2f(reproj_left(0),reproj_left(1)), 2, cv::Scalar(0, 0, 255),
                       2);
                int font_face = cv::FONT_HERSHEY_COMPLEX;
                double font_scale = 0.4;
                int thickness = 3;

                std::ostringstream strs;
                strs << i;
                std::string str = strs.str();

                cv::putText(img_out,str,feat->GetKeyPoint().pt,font_face,font_scale,cv::Scalar(0,0,255));
            }
        }
        cv::circle(img_out, current_frame_->GetLeftFeatures()[i]->GetKeyPoint().pt, 2, cv::Scalar(0, 255, 0),
                       2);
    }

    cv::putText(img_out,"reprojected point",cv::Point2f(5,10),cv::FONT_HERSHEY_COMPLEX,0.4,cv::Scalar(0,0,255));
    cv::putText(img_out,"detected feature point",cv::Point2f(5,20),cv::FONT_HERSHEY_COMPLEX,0.4,cv::Scalar(0,255,0));
    return img_out;
}

cv::Mat Viewer::PlotFrameRightReproj() {
    cv::Mat img_out = current_frame_->GetUndistortLeftImg().clone();
    LOG(INFO) << "Right Img size. rows: " << img_out.size[0] << ", cols: " << img_out.size[1];
    //cv::cvtColor(current_frame_->GetUndistortLeftImg(), img_out, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < current_frame_->GetLeftFeatures().size(); ++i) {
        if (current_frame_->GetLeftFeatures()[i]->GetMapPoint()) {
            auto feat = current_frame_->GetLeftFeatures()[i];
            //Put3DInfo2Img(feat->GetMapPoint()->Pos(),feat->GetKeyPoint().pt,i,img_out);
            auto reproj_right = current_frame_->GetRightCamera()->World2pixel(feat->GetMapPoint()->Pos(),current_frame_->Pose());
            if(reproj_right(0)<=img_out.size[1] && reproj_right(1)<=img_out.size[0]) {
                cv::circle(img_out, cv::Point2f(reproj_right(0),reproj_right(1)), 2, cv::Scalar(0, 0, 255),
                       2);
                int font_face = cv::FONT_HERSHEY_COMPLEX;
                double font_scale = 0.4;
                int thickness = 3;

                std::ostringstream strs;
                strs << i;
                std::string str = strs.str();

                cv::putText(img_out,str,cv::Point2f(reproj_right(0),reproj_right(1)),font_face,font_scale,cv::Scalar(0,0,255));
            } 
            LOG(INFO) << "Right camera reproj. pixel coordinates: " << reproj_right(0) << ", " << reproj_right(1);
        }
        cv::circle(img_out, current_frame_->GetRightFeatures()[i]->GetKeyPoint().pt, 2, cv::Scalar(0, 255, 0),
                       2);
    }
    cv::putText(img_out,"reprojected point",cv::Point2f(5,10),cv::FONT_HERSHEY_COMPLEX,0.4,cv::Scalar(0,0,255));
    cv::putText(img_out,"detected feature point",cv::Point2f(5,20),cv::FONT_HERSHEY_COMPLEX,0.4,cv::Scalar(0,255,0));
    return img_out;
}

void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
    Eigen::Isometry3d Twc = current_frame_->Pose().inverse();
    pangolin::OpenGlMatrix m(Twc.matrix());
    vis_camera.Follow(m, true);
}

void Viewer::DrawFrame(std::shared_ptr<Frame> frame, const float* color) {
    Eigen::Isometry3d Twc = frame->Pose().inverse();
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 400;
    const float fy = 400;
    const float cx = 512;
    const float cy = 384;
    const float width = 1080;
    const float height = 768;

    glPushMatrix();

    Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    glMultMatrixf((GLfloat*)m.data());

    if (color == nullptr) {
        glColor3f(1, 0, 0);
    } else
        glColor3f(color[0], color[1], color[2]);

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

void Viewer::DrawMapPoints() {
    const float red[3] = {1.0, 0, 0};
    for (auto& kf : active_keyframes_) {
        DrawFrame(kf.second, red);
    }

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto& landmark : active_landmarks_) {
        auto pos = landmark.second->Pos();
        glColor3f(red[0], red[1], red[2]);
        glVertex3d(pos[0], pos[1], pos[2]);
    }
    glEnd();
}

}  // namespace myslam
