#include "converter.h"
#include "types.h"

namespace my_slam
{

// std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
// {
//     std::vector<cv::Mat> vDesc;
//     vDesc.reserve(Descriptors.rows);
//     for (int j=0;j<Descriptors.rows;j++)
//         vDesc.push_back(Descriptors.row(j));

//     return vDesc;
// }

// g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
// {
//     Eigen::Matrix<double,3,3> R;
//     R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
//          cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
//          cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

//     Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

//     return g2o::SE3Quat(R,t);
// }

g2o::SE3Quat Converter::toSE3Quat(const Eigen::Isometry3d& isoT) {
    Mat33 rotation = isoT.rotation();
    Vec3 translation = isoT.translation();
    return g2o::SE3Quat(rotation,translation);
}

// cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
// {
//     Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
//     return toCvMat(eigMat);
// }

// cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
// {
//     Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
//     Eigen::Vector3d eigt = Sim3.translation();
//     double s = Sim3.scale();
//     return toCvSE3(s*eigR,eigt);
// }

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCVMatf34(const Eigen::Isometry3d& isoT) {
    cv::Mat cvMat(3,4,CV_32F);
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 4; j++) {
            cvMat.at<float>(i,j) = isoT.matrix()(i,j);
        }
    }

    return cvMat.clone();
}

// cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
// {
//     cv::Mat cvMat(3,3,CV_32F);
//     for(int i=0;i<3;i++)
//         for(int j=0; j<3; j++)
//             cvMat.at<float>(i,j)=m(i,j);

//     return cvMat.clone();
// }

// cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
// {
//     cv::Mat cvMat(3,1,CV_32F);
//     for(int i=0;i<3;i++)
//             cvMat.at<float>(i)=m(i);

//     return cvMat.clone();
// }

// cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
// {
//     cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
//     for(int i=0;i<3;i++)
//     {
//         for(int j=0;j<3;j++)
//         {
//             cvMat.at<float>(i,j)=R(i,j);
//         }
//     }
//     for(int i=0;i<3;i++)
//     {
//         cvMat.at<float>(i,3)=t(i);
//     }

//     return cvMat.clone();
// }

// Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
// {
//     Eigen::Matrix<double,3,1> v;
//     v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

//     return v;
// }

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint) {
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;
    return v;
}

 Eigen::Matrix<double,2,1> Converter::toVector2d(const cv::Point2f &cvPoint) {
     Eigen::Matrix<double,2,1> v;
     v << cvPoint.x, cvPoint.y;
     return v;
 }

Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3) {
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

cv::Mat Converter::toCvMat34(const Eigen::Isometry3d& pose) {
     cv::Mat cvMat = cv::Mat::eye(3,4,CV_32F);
     Eigen::Matrix4d pose_matrix;
     pose_matrix = pose.matrix();
    for(int i=0;i<3;i++) {
        for(int j=0;j<3;j++) {
            cvMat.at<float>(i,j)=pose_matrix(i,j);
        }
    }

    for(int i=0;i<3;i++) {
        cvMat.at<float>(i,3)=pose_matrix(i,3);
    }

    return cvMat.clone();
}

// std::vector<float> Converter::toQuaternion(const cv::Mat &M)
// {
//     Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
//     Eigen::Quaterniond q(eigMat);

//     std::vector<float> v(4);
//     v[0] = q.x();
//     v[1] = q.y();
//     v[2] = q.z();
//     v[3] = q.w();

//     return v;
// }

/**
 * 像素坐标转换为相机的归一化平面坐标
 * @param p_p -- 像素坐标
 * @param K -- 相机的内参数矩阵 3×3
 * @return 归一化坐标,格式是cv::Point2f
 */
cv::Point2f Converter::Pixel2normal(const cv::KeyPoint& keyPoint, const cv::Mat K) {
    Vec3 normalized(keyPoint.pt.x,keyPoint.pt.y,1);
    Mat33 K_eigen = toMatrix3d(K);
    normalized = K_eigen.inverse() * normalized;
    return cv::Point2f(normalized(0),normalized(1));
}

} //namespace my_slam
