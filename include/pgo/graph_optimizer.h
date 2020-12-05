// Codes for pose graph optimization in LiDAR SLAM
// By Yue Pan et al.

#ifndef INCLUDE_GRAPH_OPTIMIZER_H
#define INCLUDE_GRAPH_OPTIMIZER_H

#include "utility.hpp"

#include <cmath>

#if CERES_ON
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#endif

#if G2O_ON
#include "g2o/core/sparse_optimizer.h"
#endif

using namespace std;

namespace lo
{

  template <typename T>
  inline Eigen::Matrix<T, 3, 1> quat2angleaxis(
      const Eigen::Quaternion<T> &quaternion)
  {
    Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
    // We choose the quaternion with positive 'w', i.e., the one with a smaller
    // angle that represents this orientation.
    if (normalized_quaternion.w() < 0.)
    {
      normalized_quaternion.x() *= T(-1.);
      normalized_quaternion.y() *= T(-1.);
      normalized_quaternion.z() *= T(-1.);
    }
    // We convert the normalized_quaternion into a vector along the rotation axis
    // with length of the rotation angle.
    const T angle = T(2.) * atan2(normalized_quaternion.vec().norm(),
                                  normalized_quaternion.w());
    constexpr double cutoff_angle = 1e-7; // We linearize below this angle.
    const T scale = angle < cutoff_angle ? T(2.) : angle / sin(angle / T(2.));
    return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
                                  scale * normalized_quaternion.y(),
                                  scale * normalized_quaternion.z());
  }

  // reference: https://github.com/gaoxiang12/slambook
  // Computes the error term for two poses that have a relative pose measurement
  // between them. Let the hat variables be the measurement. We have two poses x_a
  // and x_b. Through sensor measurements we can measure the transformation of
  // frame B w.r.t frame A denoted as t_ab_hat. We can compute an error metric
  // between the current estimate of the poses and the measurement.
  //
  // In this formulation, we have chosen to represent the rigid transformation as
  // a Hamiltonian quaternion, q, and position, p. The quaternion ordering is
  // [x, y, z, w].

  // The estimated measurement is:
  //      t_ab = [ p_ab ]  = [ R(q_a)^T * (p_b - p_a) ]
  //             [ q_ab ]    [ q_a^{-1] * q_b         ]
  //
  // where ^{-1} denotes the inverse and R(q) is the rotation matrix for the
  // quaternion. Now we can compute an error metric between the estimated and
  // measurement transformation. For the orientation error, we will use the
  // standard multiplicative error resulting in:
  //
  //   error = [ p_ab - \hat{p}_ab                 ]
  //           [ 2.0 * Vec(q_ab * \hat{q}_ab^{-1}) ]
  //
  // where Vec(*) returns the vector (imaginary) part of the quaternion. Since
  // the measurement has an uncertainty associated with how accurate it is, we
  // will weight the errors by the square root of the measurement information
  // matrix:
  //
  //   residuals = I^{1/2) * error
  // where I is the information matrix which is the inverse of the covariance.

  class PoseGraph3dErrorTermQUAT
  {
  public:
    PoseGraph3dErrorTermQUAT(const pose_qua_t &transform_ab_measured,
                             const Eigen::Matrix<double, 6, 6> &sqrt_information)
        : transform_ab_measured_(transform_ab_measured), sqrt_information_(sqrt_information) {}

    template <typename T>
    bool operator()(const T *const t_a_ptr, const T *const q_a_ptr,
                    const T *const t_b_ptr, const T *const q_b_ptr,
                    T *residuals_ptr) const
    {
      Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_a(t_a_ptr);
      Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

      Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_b(t_b_ptr);
      Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

      // Compute the relative transformation between the two frames.
      Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
      // Eigen::Quaternion<T> q_a_inverse = q_a.inverse();
      Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

      // Represent the displacement between the two frames in the A frame.
      Eigen::Matrix<T, 3, 1> t_ab_estimated = q_a_inverse * (t_b - t_a);

      // Compute the error between the two orientation estimates.
      Eigen::Quaternion<T> delta_q =
          transform_ab_measured_.quat.template cast<T>() * q_ab_estimated.conjugate();
      //        Eigen::Quaternion<T> delta_q =
      //                t_ab_measured_.quat.template cast<T>() * q_ab_estimated.inverse();
      // Eigen::Matrix::cast<typename T> can change num_type in Matrix
      // To use Eigen::cast in template function, you need to write like a.template cast<T>(),
      // so that compiler will know '<" is not a "less than sign"

      // Compute the residuals.
      // [ position         ]   [ delta_p          ]
      // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
      // reference: https://github.com/gaoxiang12/slambook
      Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
      residuals.template block<3, 1>(0, 0) =
          t_ab_estimated - transform_ab_measured_.trans.template cast<T>();

      // use euler-angle (Failed)
      //            Eigen::Matrix<T,3,3> rot = delta_q.toRotationMatrix().template cast<T>();
      //            Eigen::Matrix<T,3,1> vec = rot.eulerAngles(0, 1, 2).template cast<T>();
      //            residuals.template block<3, 1>(3, 0) = vec;
      // use quaternion real part (passed)
      residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();
      // use Angle-Axis error (Failed)
      // residuals.template block<3, 1>(3, 0) = quat2angleaxis(delta_q);

      // Scale the residuals by the measurement uncertainty.
      residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

      return true;
    }

#if CERES_ON
    static ceres::CostFunction *Create(
        const pose_qua_t &transform_ab_measured,
        const Eigen::Matrix<double, 6, 6> &sqrt_information)
    {
      return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTermQUAT, 6, 3, 4, 3, 4>( // error(dim=6) ,4 input, tran1(dim=3), quat1(dim=4), tran2(dim=3), quat2(dim=4)
          new PoseGraph3dErrorTermQUAT(transform_ab_measured, sqrt_information));
    }
#endif

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    // The measurement for the position of B relative to A in the A frame.
    const pose_qua_t transform_ab_measured_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 6, 6> sqrt_information_;
  };

  class GlobalOptimize
  {
  public:
    GlobalOptimize()
    {
      init();
    }

    GlobalOptimize(pgo_param_t &param)
    {
      param_ = param;
      init();
    }

    void set_equal_weight(bool on_or_not);
    void set_robust_function(bool robust_status);
    void set_max_iter_num(int max_iter);
    void set_rotation_limit(bool only_limit_translation_or_not);
    void set_diagonal_information_matrix(bool diagonal_information_matrix_on_or_not);
    void set_covariance_updating_ratio(float first_time_ratio, float life_long_ratio);
    void set_wrong_edge_check_threshold(float tran_thre, float rot_thre);
    void set_free_node(bool on_or_not);
    void set_problem_size(bool is_small_size_problem);

    bool optimize_pose_graph_g2o(cloudblock_Ptrs &all_blocks, constraints &all_cons, bool update_edge_or_not = true);

    bool optimize_pose_graph_ceres(cloudblock_Ptrs &all_blocks, constraints &all_cons,
                                   double moving_threshold_tran = 5.0, double moving_threshold_rot = 0.5, bool update_edge_or_not = true);

    bool optimize_pose_graph_gtsam(cloudblock_Ptrs &all_blocks, constraints &all_cons);

    //common
    bool update_optimized_nodes(cloudblock_Ptrs &all_blocks, bool update_init_pose = false, bool update_bbx = false);

    //registration blunder elimination
#if 0
    bool cal_con_number(constraints &all_cons, strip &all_blocks);
    bool eliminate_blunders(constraints &all_cons, strip &all_blocks);
    float cal_f_norm(Eigen::Matrix4d &Trans1, Eigen::Matrix4d &Trans2); //f-norm is used for evaluating the similarity of two matrices
#endif

  protected:
  private:
    void init()
    {
#if CERES_ON
      ceres_problem_ = new ceres::Problem();
      ceres_options_ = new ceres::Solver::Options();
      ceres_summary_ = new ceres::Solver::Summary();
#endif

#if G2O_ON
      g2o_optimizer_ = new g2o::SparseOptimizer();
#endif
    }

    //g2o
    bool set_pgo_options_g2o();

    bool set_pgo_problem_g2o(cloudblock_Ptrs &all_blocks, constraints &all_cons);

    bool solve_pgo_problem_g2o();

    bool assign_pose_optimized_g2o(cloudblock_Ptrs &all_blocks);

    bool robust_verify_chi2_g2o();

    //ceres
    bool set_pgo_options_ceres();

    bool set_pgo_problem_ceres(cloudblock_Ptrs &all_blocks, constraints &all_cons,
                               double moving_threshold_tran, double moving_threshold_rot);

    bool solve_pgo_problem_ceres();

    bool assign_pose_optimized_ceres(cloudblock_Ptrs &all_blocks);

    bool fix_node_ceres(int block_id, float translate_thre, float quat_thre, bool only_translation = true);

    bool update_edge_covariance_ceres(constraints &all_cons);

    double *mutable_pose_tran(int i) // Use the block unique id to get the right parameters
    {
      return ceres_parameters_ + (7 * i);
    }

    double *mutable_pose_quat(int i) // Use the block unique id to get the right parameters
    {
      return ceres_parameters_ + (7 * i + 3);
    }

    //gtsam
    bool set_pgo_options_gtsam();

    bool set_pgo_problem_gtsam(cloudblock_Ptrs &all_blocks, constraints &all_cons);

    bool solve_pgo_problem_gtsam();

    bool assign_pose_optimized_gtsam(cloudblock_Ptrs &all_blocks);

    //commom
    bool update_optimized_edges(constraints &all_cons, bool fixed_reg_edge = true);

    void set_adjacent_edge_information_matrix(Matrix6d &vc_matrix);

    bool check_wrong_edge(Eigen::Matrix4d delta_tran_mat, float translation_thre, float rotation_thre_deg);

    pgo_param_t param_;

    double *ceres_observations_;
    double *ceres_parameters_; //The structure of unknown parameters: [pose (*7, first tran and then quat)], for ceres
    int num_ceres_observations_;
    int num_ceres_parameters_;

#if CERES_ON
    //ceres
    ceres::Problem *ceres_problem_;
    ceres::Solver::Options *ceres_options_;
    ceres::Solver::Summary *ceres_summary_;
    ceres::LossFunction *ceres_robust_kernel_function_;
#endif

#if G2O_ON
    //g2o
    g2o::SparseOptimizer *g2o_optimizer_;
#endif

#if GTSAM_ON
    gtsam::NonlinearFactorGraph gtsam_factor_graph_;
    gtsam::Values initial_estimates_;
    gtsam::Values optimized_estimates_;
    gtsam::ISAM2 *isam_;
    gtsam::Values isam_current_estimates_;

    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr constraint_noise_;
    gtsam::noiseModel::Base::shared_ptr robust_noise_model_;
#endif
  };

} // namespace lo

#endif //INCLUDE_GRAPH_OPTIMIZER_H