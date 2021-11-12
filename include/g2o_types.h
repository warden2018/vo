#ifndef G2OTYPES_H_
#define G2OTYPES_H_


#include <g2o/core/base_vertex.h>
#include "types.h"

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


//参数顶点
class CameraParameters : public g2o::Parameter
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    CameraParameters();

    CameraParameters(double fx, double fy,
        const Vec2& principle_point)
      : fx_(fx),
        fy_(fy),
        principle_point_(principle_point) {}

    Vec2 cam_map (const Vec3 & trans_xyz) const {
        Vec2 proj = project(trans_xyz);
        Vec2 res;
        res(0) = proj(0) * fx_ + principle_point_(0);
        res(1) = proj(1) * fy_ + principle_point_(1);
        return res;
    }

    //Vector3 stereocam_uvu_map (const Vector3 & trans_xyz) const;

    virtual bool read (std::istream& is){
      is >> fx_;
      is >> fy_;
      is >> principle_point_(0);
      is >> principle_point_(1);
    //   is >> baseline;
      return true;
    }

    virtual bool write (std::ostream& os) const {
      os << fx_ << " ";
      os << fy_ << " ";
      os << principle_point_(0) << " ";
      os << principle_point_(1) << " ";
      //os << baseline << " ";
      return true;
    }

    double fx_;
    double fy_;
    Vec2 principle_point_;
    //double baseline_;
};

} //namespace my_slam

#endif // G2OTYPES_H_