#ifndef G2OTYPES_H_
#define G2OTYPES_H_

#include <iostream>
#include <g2o/core/base_vertex.h>
#include "types.h"
#include "converter.h"

#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include <g2o/core/optimization_algorithm_gauss_newton.h>


#include "g2o/solvers/structure_only/structure_only_solver.h"


namespace my_slam{


// Vec2 project(const Vec3& v){
//     if(v(2) <= 0) {
//         return Vec2(0,0);
//     }
//     Vec2 res;
//     res(0) = v(0)/v(2);
//     res(1) = v(1)/v(2);
//     return res;
// }



//定义g2o下的节点和边，借助g2o来优化求相机的运动
class PoseVertex : public g2o::BaseVertex<6,Sophus::SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void setToOriginImpl() override {
        _estimate = Sophus::SE3d();
    }

    virtual void oplusImpl(const double *update) override {
        Eigen::Matrix<double,6,1> update_eigen;
        update_eigen << update[0] , update[1] , update[2] , update[3] , update[4] , update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }

    virtual bool read(std::istream &in) {}

    virtual bool write(std::ostream &out) const {}
};

class ProjectionEdge : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, PoseVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ProjectionEdge(const Eigen::Vector3d &pos,const Eigen::Isometry3d& base_link2Camera,const Eigen::Matrix3d &K) 
     : _pos3d(pos),
       _K(K),
       _baselink2camera(base_link2Camera){}

    virtual void computeError() override {
        const PoseVertex* v = static_cast<const PoseVertex *> (_vertices[0]);
        const Sophus::SE3d T = v->estimate();
        Eigen::Vector3d pose_pixel = _K * _baselink2camera * (T * _pos3d); //未归一化的像素坐标系，注意，这里地图点转换为base link点，再转换为left camera的点，然后再做投影到像素
        pose_pixel /= pose_pixel[2]; // 归一化像素坐标
        _error = pose_pixel.head<2>() - _measurement;//误差项，是一个2*1的向量.要注意，误差项是谁减去谁（这里是预测值减去观测值），要和下面的雅克比矩阵对应上。
    }

    //Jacobian -- 这里需要准备的数据是计算雅克比矩阵必须的。在这个case里面，我们需要的是相机坐标系下面的X,Y,Z,相机内参数
    virtual void linearizeOplus() override {
        const PoseVertex* v = static_cast<const PoseVertex *> (_vertices[0]);
        const Sophus::SE3d T = v->estimate();
        Eigen::Vector3d pos_cam = Converter::toSophusSE3d(_baselink2camera) * T * _pos3d; //X',Y',Z'
        double fx = _K(0,0);
        double fy = _K(1,1);
        //double cx = _K(0,2);
        //double cy = _K(1,2);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Z_2 = Z * Z;
        _jacobianOplusXi <<
            fx / Z, 0, -fx * X / Z_2, -fx * X * Y / Z_2, fx + fx * X * X / Z_2, -fx * Y / Z,
            0, fy / Z, -fy * Y / Z_2, -fy - fy * Y * Y / Z_2, fy * X * Y / Z_2, fy * X / Z;
    }

    virtual bool read(std::istream &in) {}

    virtual bool write(std::ostream &out) const {}

public:
    Eigen::Vector3d _pos3d;
    Eigen::Isometry3d _baselink2camera;
    Eigen::Matrix3d _K;
};


class  EdgeSE3ProjectXYZOnlyPoseOneinStereo: public  g2o::BaseUnaryEdge<2, Vec2, g2o::VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZOnlyPoseOneinStereo(){}

  bool read(std::istream& is) {
      for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
  }

  bool write(std::ostream& os) const {
       for (int i=0; i<2; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
            os << " " <<  information()(i,j);
            }
        return os.good();
  }

  void computeError()  {
    const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    Vec2 obs(_measurement);
    _error = obs-cam_project(base2camera_.map(v1->estimate().map(Xw))); //这里是先将地图点转换为base link坐标值，然后再转换为相机坐标系的值，再做投影
  }

  bool isDepthPositive() {
    const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    return (base2camera_ * v1->estimate().map(Xw))(2)>0.0;
  }


  virtual void linearizeOplus() {
    g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
    Vec3 xyz_trans = base2camera_.map(vi->estimate().map(Xw));

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double invz = 1.0/xyz_trans[2];
    double invz_2 = invz*invz;

    _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
    _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
    _jacobianOplusXi(0,2) = y*invz *fx;
    _jacobianOplusXi(0,3) = -invz *fx;
    _jacobianOplusXi(0,4) = 0;
    _jacobianOplusXi(0,5) = x*invz_2 *fx;

    _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
    _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
    _jacobianOplusXi(1,2) = -x*invz *fy;
    _jacobianOplusXi(1,3) = 0;
    _jacobianOplusXi(1,4) = -invz *fy;
    _jacobianOplusXi(1,5) = y*invz_2 *fy;
  }

  Vec2 cam_project(const Vec3 & trans_xyz) {
    Vec2 proj = project2d(trans_xyz);
    Vec2 res;
    res[0] = proj[0]*fx + cx;
    res[1] = proj[1]*fy + cy;
    return res;
}

  Eigen::Vector3d Xw;
  g2o::SE3Quat base2camera_;
  double fx, fy, cx, cy;

 private:
  Vec2 project2d(const Eigen::Vector3d& v)  {
    Vec2 res;
    res(0) = v(0)/v(2);
    res(1) = v(1)/v(2);
    return res;
}
};




} //namespace my_slam

#endif // G2OTYPES_H_