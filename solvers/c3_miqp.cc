#include "solvers/c3_miqp.h"

namespace dairlib {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

C3MIQP::C3MIQP(const LCS& LCS, const CostMatrices& costs,
               const vector<VectorXd>& xdesired, const C3Options& options)
    : C3(LCS, costs, xdesired, options), env_(true) {
  // Create an environment
  //  env_.set("LogToConsole", "0");
  env_.set("OutputFlag", "0");
  env_.set("Threads", "5");
  env_.start();
}

VectorXd C3MIQP::SolveSingleProjection(const MatrixXd& U,
                                       const VectorXd& delta_c,
                                       const MatrixXd& E, const MatrixXd& F,
                                       const MatrixXd& H, const VectorXd& c,
                                       const int admm_iteration,
                                       const int& warm_start_index) {
  // set up linear term in cost
  VectorXd cost_lin = -2 * delta_c.transpose() * U;

  // set up for constraints (Ex + F \lambda + Hu + c >= 0)
  MatrixXd Mcons1(m_, n_ + m_ + k_);
  Mcons1 << E, F, H;

  // set up for constraints (\lambda >= 0)
  MatrixXd MM1 = MatrixXd::Zero(m_, n_);
  MatrixXd MM2 = MatrixXd::Identity(m_, m_);
  MatrixXd MM3 = MatrixXd::Zero(m_, k_);
  MatrixXd Mcons2(m_, n_ + m_ + k_);
  Mcons2 << MM1, MM2, MM3;

  GRBModel model = GRBModel(env_);
  //  model.set(GRB_IntParam_LogToConsole, 1);
  //  model.set(GRB_StringParam_LogFile, "grb_debug");
  //  model.set("Cutoff", "0.001");
  //  model.set("FeasibilityTol", "0.00001");
  //  model.set("FeasibilityTol", "0.01");
  //  model.set("IterationLimit", "40");

  GRBVar delta_k[n_ + m_ + k_];
  GRBVar binary[m_];

  for (int i = 0; i < m_; i++) {
    binary[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    if (warm_start_index != -1) {
      binary[i].set(GRB_DoubleAttr_Start,
                    warm_start_binary_[admm_iteration][warm_start_index](i));
    }
  }

  for (int i = 0; i < n_ + m_ + k_; i++) {
    delta_k[i] = model.addVar(-100.0, 100.0, 0.0, GRB_CONTINUOUS);
    if (warm_start_index != -1) {
      delta_k[i].set(GRB_DoubleAttr_Start,
                     warm_start_delta_[admm_iteration][warm_start_index](i));
    }
  }

  GRBQuadExpr obj = 0;

  for (int i = 0; i < n_ + m_ + k_; i++) {
    obj.addTerm(cost_lin(i), delta_k[i]);
    obj.addTerm(U(i, i), delta_k[i], delta_k[i]);
  }

  model.setObjective(obj, GRB_MINIMIZE);

  int M = 100000;  // big M variable
  double coeff[n_ + m_ + k_];
  double coeff2[n_ + m_ + k_];

  for (int i = 0; i < m_; i++) {
    GRBLinExpr cexpr = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_ + m_ + k_; j++) {
      coeff[j] = Mcons2(i, j);
    }

    cexpr.addTerms(coeff, delta_k, n_ + m_ + k_);
    model.addConstr(cexpr >= 0);
    model.addConstr(cexpr <= M * (1 - binary[i]));

    GRBLinExpr cexpr2 = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_ + m_ + k_; j++) {
      coeff2[j] = Mcons1(i, j);
    }

    cexpr2.addTerms(coeff2, delta_k, n_ + m_ + k_);
    model.addConstr(cexpr2 + c(i) >= 0);
    model.addConstr(cexpr2 + c(i) <= M * binary[i]);
  }

  model.optimize();

  VectorXd delta_kc(n_ + m_ + k_);
  VectorXd binaryc(m_);
  for (int i = 0; i < n_ + m_ + k_; i++) {
    delta_kc(i) = delta_k[i].get(GRB_DoubleAttr_X);
  }
  for (int i = 0; i < m_; i++) {
    binaryc(i) = binary[i].get(GRB_DoubleAttr_X);
  }

  if (warm_start_index != -1) {
    warm_start_delta_[admm_iteration][warm_start_index] = delta_kc;
    warm_start_binary_[admm_iteration][warm_start_index] = binaryc;
  }
  return delta_kc;
}

VectorXd C3MIQP::SolveRobustSingleProjection(
    const MatrixXd& U, const VectorXd& delta_c, const MatrixXd& E,
    const MatrixXd& F, const MatrixXd& H, const VectorXd& c,
    const Eigen::MatrixXd& W_x, const Eigen::MatrixXd& W_l,
    const Eigen::MatrixXd& W_u, const Eigen::VectorXd& w,
    const int admm_iteration, const int& warm_start_index) {
  // set up linear term in cost
  VectorXd cost_lin = -2 * delta_c.transpose() * U;

  // set up for constraints (Ex + F \lambda + Hu + c >= 0)
  MatrixXd Mcons1(m_, n_ + m_ + k_);
  Mcons1 << E, F, H;

  // set up for constraints (\lambda >= 0)
  MatrixXd MM1 = MatrixXd::Zero(m_, n_);
  MatrixXd MM2 = MatrixXd::Identity(m_, m_);
  MatrixXd MM3 = MatrixXd::Zero(m_, k_);
  MatrixXd Mcons2(m_, n_ + m_ + k_);
  Mcons2 << MM1, MM2, MM3;

  GRBModel model = GRBModel(env_);

  GRBVar delta_k[n_ + m_ + k_];
  GRBVar binary[m_];
  GRBVar reduced_friction_cone_binary[m_];

  for (int i = 0; i < m_; i++) {
    binary[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    if (warm_start_index != -1) {
      binary[i].set(GRB_DoubleAttr_Start,
                    warm_start_binary_[admm_iteration][warm_start_index](i));
    }
  }

  for (int i = 0; i < n_ + m_ + k_; i++) {
    delta_k[i] = model.addVar(-100.0, 100.0, 0.0, GRB_CONTINUOUS);
    if (warm_start_index != -1) {
      delta_k[i].set(GRB_DoubleAttr_Start,
                     warm_start_delta_[admm_iteration][warm_start_index](i));
    }
  }

  GRBQuadExpr obj = 0;

  for (int i = 0; i < n_ + m_ + k_; i++) {
    obj.addTerm(cost_lin(i), delta_k[i]);
    obj.addTerm(U(i, i), delta_k[i], delta_k[i]);
  }

  // initial state constraint
  if (warm_start_index == 0){
    for (int i = 0; i < n_; ++i){
      model.addConstr(delta_k[i] == delta_c[i]);
    }
  }


  model.setObjective(obj, GRB_MINIMIZE);

  int M = 1000;  // big M variable
  double coeff[n_ + m_ + k_];
  double coeff2[n_ + m_ + k_];

  for (int i = 0; i < m_; i++) {
    GRBLinExpr lambda_expr = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_ + m_ + k_; j++) {
      coeff[j] = Mcons2(i, j);
    }

    lambda_expr.addTerms(coeff, delta_k, n_ + m_ + k_);
    model.addConstr(lambda_expr >= 0);
    model.addConstr(lambda_expr <= M * (1 - binary[i]));

    GRBLinExpr activation_expr = 0;

    /// convert VectorXd to double
    for (int j = 0; j < n_ + m_ + k_; j++) {
      coeff2[j] = Mcons1(i, j);
    }

    activation_expr.addTerms(coeff2, delta_k, n_ + m_ + k_);
    model.addConstr(activation_expr + c(i) >= 0);
    model.addConstr(activation_expr + c(i) <= M * binary[i]);
  }

  double* tangential_velocity_coeffs;
  MatrixXd P_t(m_, n_ + m_ + k_);
  int constraint_rows = m_;
  P_t << W_x, W_l, W_u;
//  std::cout << "W_x: " << W_x << std::endl;
//  std::cout << "W_l: " << W_l << std::endl;
//  std::cout << "W_u: " << W_u << std::endl;
  // stewart and trinkle
//  P_t << E, F, H;
//  MatrixXd P_t2 = P_t.bottomRows(((m_ / 6) * 4) / 7 * 3);
//  P_t = P_t2;
//  constraint_rows = P_t2.rows();

  for (int i = 0; i < constraint_rows; ++i) {
    reduced_friction_cone_binary[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    GRBLinExpr tangential_velocity_expr = 0;
    tangential_velocity_coeffs = P_t.row(i).data();
    tangential_velocity_expr.addTerms(tangential_velocity_coeffs, delta_k,
                                      n_ + m_ + k_);

    /// Constraint explanation:
    /// Adding a binary decision to either be:
    /// - inside a conservative (mu_l) friction cone (stick)
    /// or
    /// - on the boundary of the original friction cone (mu) (slip)
    /// where mu_l < mu

    /// The active constraint for the sticking condition is:
    /// lambda_1 - lambda_2 <= mu_l / mu * (lambda_1 + lambda_2)

    /// The active constraint for the friction cone boundary is:
    /// lambda_2 == 0
    /// where lambda_2 is the friction force in the sliding direction
    /// tangential velocity expr is the tangential velocity in the SAME direction as the friction force
    if (i % 2 == 1) {
      model.addConstr(delta_k[n_ + i - 1] - delta_k[n_ + i] <=
                      (0.5 / 0.6) * (delta_k[n_ + i] + delta_k[n_ + i - 1]) +
                          M * (reduced_friction_cone_binary[i]));
      model.addConstr(tangential_velocity_expr <=
                      M * (reduced_friction_cone_binary[i]));
      model.addConstr(delta_k[n_ + i] <=
          M * (1 - reduced_friction_cone_binary[i]));
    } else {
      model.addConstr(delta_k[n_ + i + 1] - delta_k[n_ + i] <=
                      (0.5 / 0.6) * (delta_k[n_ + i] + delta_k[n_ + i + 1]) +
                          M * (reduced_friction_cone_binary[i]));
      model.addConstr(tangential_velocity_expr <=
                      M * (reduced_friction_cone_binary[i]));
      model.addConstr(delta_k[n_ + i] <=
          M * (1 - reduced_friction_cone_binary[i]));
    }
    /// if i % 2 == 0:
    ///     lambda_1 = 0
    /// else:
    ///     lambda_2 = 0
//    model.addConstr(delta_k[n_ + i] <=
//                    M * (1 - reduced_friction_cone_binary[i]));
  }

  model.optimize();

  VectorXd delta_kc(n_ + m_ + k_);
  VectorXd binaryc(m_);
  for (int i = 0; i < n_ + m_ + k_; i++) {
    delta_kc(i) = delta_k[i].get(GRB_DoubleAttr_X);
  }
  for (int i = 0; i < m_; i++) {
    binaryc(i) = binary[i].get(GRB_DoubleAttr_X);
  }

  if (warm_start_index != -1) {
    warm_start_delta_[admm_iteration][warm_start_index] = delta_kc;
    warm_start_binary_[admm_iteration][warm_start_index] = binaryc;
  }

  return delta_kc;
}

std::vector<Eigen::VectorXd> C3MIQP::GetWarmStartDelta() const {
  return warm_start_delta_[0];
}

std::vector<Eigen::VectorXd> C3MIQP::GetWarmStartBinary() const {
  return warm_start_binary_[0];
}

}  // namespace solvers
}  // namespace dairlib