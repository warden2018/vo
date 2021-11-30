#include "frame.h"
#include "camera.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include <thread>

namespace my_slam {

Frame::Frame(double ts, std::shared_ptr<Camera> left, std::shared_ptr<Camera> right,
                                std::shared_ptr<ORBextractor> leftExtrator,
                                std::shared_ptr<ORBextractor> rightExtrator, 
                                const Eigen::Isometry3d& pose, 
                                const Mat& leftImg,const Mat& rightImg)
    : time_stamp_(ts),
      left_camera_(left),
      right_camera_(right),
      pose_(pose),
      left_image_(leftImg),
      right_image_(rightImg),
      ORBextractorLeft_(leftExtrator),
      ORBextractorRight_(rightExtrator) {
    //初始化双目的基线长度
    baseline_ = left_camera_->GetBaselineMeter();
    baselineFx_ = left_camera_->GetBaselineFx();

    // Scale Level Info
    nScaleLevels_ = ORBextractorLeft_->GetLevels();
    fScaleFactor_ = ORBextractorLeft_->GetScaleFactor();
    fLogScaleFactor_ = log(fScaleFactor_);
    vScaleFactors_ = ORBextractorLeft_->GetScaleFactors();
    vInvScaleFactors_ = ORBextractorLeft_->GetInverseScaleFactors();
    vLevelSigma2_ = ORBextractorLeft_->GetScaleSigmaSquares();
    vInvLevelSigma2_ = ORBextractorLeft_->GetInverseScaleSigmaSquares();


    std::thread threadLeft(&Frame::ExtractORB,this,0,leftImg,0,0);
    std::thread threadRight(&Frame::ExtractORB,this,1,rightImg,0,0);

    threadLeft.join();
    threadRight.join();

    N = vKeys_.size();
    vpMapPoints_ = std::vector<std::shared_ptr<MapPoint>>(N,nullptr);
    if(vKeys_.empty()){ //当没有特征点，返回。
        return;
    }

    UndistortKeyPoints();

    ComputeStereoMatches();
}

void Frame::SetPose(const Eigen::Isometry3d& pose) {
    std::unique_lock<std::mutex> lock(pose_mutex_);
    pose_ = pose;
}

const Eigen::Isometry3d& Frame::Pose() {
    std::unique_lock<std::mutex> lock(pose_mutex_);
    return pose_;
}

void Frame::SetKeyFrame() {
    static long keyframe_factory_id = 0;
    is_keyFrame_ = true;
    key_frame_id_ = keyframe_factory_id++;
}

Frame::Ptr Frame::CreateFrame(double ts, std::shared_ptr<Camera> left, std::shared_ptr<Camera> right,
                                std::shared_ptr<ORBextractor> leftExtrator,
                                std::shared_ptr<ORBextractor> rightExtrator, 
                                const Eigen::Isometry3d& pose, 
                                const Mat& leftImg,const Mat& rightImg) { 
    static long factory_id = 0;
    Frame::Ptr new_frame = std::make_unique<Frame>(ts,left,right,leftExtrator,rightExtrator, pose,leftImg,rightImg);
    new_frame->id_ = factory_id++;
    //new_frame->SetPose(Eigen::Isometry3d::Identity());
    return new_frame;
}

const std::vector<std::shared_ptr<Feature>>& Frame::GetLeftFeatures() const {
    //std::unique_lock<std::mutex> lock(pose_mutex_);
    return features_left_;
}

const std::vector<std::shared_ptr<Feature>>& Frame::GetRightFeatures() const {
    //std::unique_lock<std::mutex> lock(pose_mutex_);
    return features_right_;
}

const cv::Mat Frame::GetUndistortLeftImg() {
    cv::Mat grayOut;
    cv::cvtColor(left_image_,grayOut,cv::COLOR_BGR2GRAY);
    return grayOut; //去畸变过后的图像
}

const cv::Mat Frame::GetUndistortRightImg() {
    //先把彩色图像转化为灰度图
    cv::Mat grayOut;
    cv::cvtColor(right_image_,grayOut,cv::COLOR_BGR2GRAY);
    return grayOut; //去畸变过后的图像
}

bool Frame::SetLeftImg(const cv::Mat& img) {
    left_image_ = img;
}

//因为设置完right image之后，两张图像就被Frame持有了，马上去畸变
bool Frame::SetRightImg(const cv::Mat& img) {
    right_image_ = img;

    UndistortBothImg();
}

void Frame::AddLeftFeature(const std::shared_ptr<Feature>& feature) {
    //std::shared_ptr<Feature> feature_copy = std::make_shared<Feature>(feature->GetFrame(),feature->GetKeyPoint());
    features_left_.push_back(feature);
}

void Frame::AddRightFeature(const std::shared_ptr<Feature>& feature) {
    features_right_.push_back(feature);
}

const int Frame::GetLeftFeatureNum() const {
    return features_left_.size();
}

const int Frame::GetRightFeatureNum() const {
    return features_right_.size();
}

unsigned long Frame::GetId() {
    return id_;
}

unsigned long Frame::GetKeyFrameId() {
    return key_frame_id_;
}

const std::shared_ptr<Feature> Frame::GetLeftFeatureByindex(const int& index) const {
    if(index >= GetLeftFeatureNum()) {
        LOG(ERROR) << "The index is above the max.";
        return features_left_[GetLeftFeatureNum() - 1];
    }
    return features_left_[index];
}

const std::shared_ptr<Feature> Frame::GetRightFeatureByindex(const int& index) const {
    if(index >= GetRightFeatureNum()) {
        LOG(ERROR) << "The index is above the max.";
        return features_right_[GetRightFeatureNum() - 1];
    }
    return features_right_[index];
}

bool Frame::UndistortBothImg() {
    //调用Camera类方法对双目采集到的图像去畸变
    if(left_camera_) {
        left_image_ = left_camera_->UndistortImage(left_image_);
    }
    if(right_camera_) {
        right_image_ = right_camera_->UndistortImage(right_image_);
    }
}

void Frame::SetLeftCamera(std::shared_ptr<Camera> camera) {
    left_camera_ = camera;
}

void Frame::SetRightCamera(std::shared_ptr<Camera> camera) {
    right_camera_ = camera;
}

const std::shared_ptr<Camera>& Frame::GetLeftCamera() {
    if(left_camera_) {
        return left_camera_;
    }
}

const std::shared_ptr<Camera>& Frame::GetRightCamera() {
    if(right_camera_) {
        return right_camera_;
    }
}

void Frame::ExtractORB(int flag, const cv::Mat& img, const int x0, const int x1) {
    std::vector<int> vLapping = {x0,x1};
    if(flag==0)
        monoLeft_ = ORBextractorLeft_->Extract(img,cv::Mat(),vKeys_,Descriptors_,vLapping);
    else
        monoRight_ = ORBextractorRight_->Extract(img,cv::Mat(),vKeysRight_,DescriptorsRight_,vLapping);
}

void Frame::ComputeStereoMatches() {
    vuRight_ = std::vector<float>(N,-1.0f);
    vDepth_ = std::vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = ORBextractorLeft_->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    std::vector<std::vector<size_t> > vRowIndices(nRows,std::vector<size_t>());

    for(int i=0; i<nRows; i++) {
        vRowIndices[i].reserve(200);
    }
        
    const int Nr = vKeysRight_.size();

    for(int iR=0; iR<Nr; iR++) {
        const cv::KeyPoint &kp = vKeysRight_[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*vScaleFactors_[vKeysRight_[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = baseline_;
    const float minD = 0;
    const float maxD = baselineFx_/minZ;

    // For each left keypoint search a match in the right image
    std::vector<std::pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++) {
        const cv::KeyPoint &kpL = vKeys_[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const std::vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = Descriptors_.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = vKeysRight_[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = DescriptorsRight_.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist) {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = vKeysRight_[bestIdxR].pt.x;
            const float scaleFactor = vInvScaleFactors_[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = ORBextractorLeft_->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            std::vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= ORBextractorRight_->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++) {
                cv::Mat IR = ORBextractorRight_->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = vScaleFactors_[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD) {
                if(disparity<=0) {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                vDepth_[iL]=baselineFx_/disparity;
                vuRight_[iL] = bestuR;
                vDistIdx.push_back(std::pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--) {
        if(vDistIdx[i].first<thDist)
            break;
        else {
            vuRight_[vDistIdx[i].second]=-1;
            vDepth_[vDistIdx[i].second]=-1;
        }
    }
}


void Frame::UndistortKeyPoints() {
    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=vKeys_[i].pt.x;
        mat.at<float>(i,1)=vKeys_[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::Mat cv_k = left_camera_->K_cv();
    cv::Mat cv_distCoeff = left_camera_->GetDistCoeff();
    cv::undistortPoints(mat,mat,cv_k,cv_distCoeff,cv::Mat(),cv_k);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    vKeysUn_.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = vKeys_[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        vKeysUn_[i]=kp;
    }
}

Vec3f Frame::UnprojectStereo(const int &i) {
    const float z = vDepth_[i];
    if(z>0)
    {
        const float u = vKeysUn_[i].pt.x;
        const float v = vKeysUn_[i].pt.y;
        const float x = (u-left_camera_->K_eigen()(0,2)) * z / left_camera_->K_eigen()(0,0);
        const float y = (v-left_camera_->K_eigen()(1,2))*z / left_camera_->K_eigen()(1,1);
        Vec3 posC(x,y,z);
        Vec3 posW = left_camera_->Camera2world(posC,pose_);
        Vec3f posWf(posW(0),posW(1),posW(2));
        return posWf;
    }
    else
        return Vec3f(0,0,0);
}
} // namespace my_slam