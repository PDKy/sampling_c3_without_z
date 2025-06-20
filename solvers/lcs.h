#pragma once
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/sorted_pair.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace solvers {
class LCS {
 public:
  /// Constructor for time-varying LCS
  /// @param A, B, D, d Dynamics constraints x_{k+1} = A_k x_k + B_k u_k + D_k
  /// \lambda_k + d_k
  /// @param E, F, H, c Complementarity constraints  0 <= \lambda_k \perp E_k
  /// x_k + F_k \lambda_k  + H_k u_k + c_k
  LCS(const std::vector<Eigen::MatrixXd>& A,
      const std::vector<Eigen::MatrixXd>& B,
      const std::vector<Eigen::MatrixXd>& D,
      const std::vector<Eigen::VectorXd>& d,
      const std::vector<Eigen::MatrixXd>& E,
      const std::vector<Eigen::MatrixXd>& F,
      const std::vector<Eigen::MatrixXd>& H,
      const std::vector<Eigen::VectorXd>& c, double dt);

  /// Constructor for time-invariant LCS
  LCS(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
      const Eigen::MatrixXd& D, const Eigen::VectorXd& d,
      const Eigen::MatrixXd& E, const Eigen::MatrixXd& F,
      const Eigen::MatrixXd& H, const Eigen::VectorXd& c, const int& N,
      double dt);

  LCS(const LCS& other);
  LCS& operator=(const LCS&);
  LCS(LCS&&) = default;
  LCS& operator=(LCS&&) = default;

  void SetTangentGapLinearization(const Eigen::MatrixXd& W_x,
                                  const Eigen::MatrixXd& W_l,
                                  const Eigen::MatrixXd& W_u,
                                  const Eigen::VectorXd& w) {
    W_x_ = W_x;
    W_l_ = W_l;
    W_u_ = W_u;
    w_ = w;
    has_tangent_linearization_ = true;
  }

  /// Simulate the system for one-step
  /// @param x_init Initial x value
  /// @param input Input value
  const Eigen::VectorXd Simulate(const Eigen::VectorXd& x_init, const  Eigen::VectorXd& input, bool verbose = false);

 public:
  std::vector<Eigen::MatrixXd> A_;
  std::vector<Eigen::MatrixXd> B_;
  std::vector<Eigen::MatrixXd> D_;
  std::vector<Eigen::VectorXd> d_;
  std::vector<Eigen::MatrixXd> E_;
  std::vector<Eigen::MatrixXd> F_;
  std::vector<Eigen::MatrixXd> H_;
  std::vector<Eigen::VectorXd> c_;
  Eigen::MatrixXd W_x_;
  Eigen::MatrixXd W_l_;
  Eigen::MatrixXd W_u_;
  Eigen::VectorXd w_;
  bool has_tangent_linearization_ = false;
  Eigen::MatrixXd J_c_;
  int N_;
  double dt_;

  int n_;
  int m_;
  int k_;
};

}  // namespace solvers
}  // namespace dairlib
