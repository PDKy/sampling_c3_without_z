//
// Created by brian on 1/19/22.
//

#include "alip_traj_gen.h"

#include <cmath>
#include <fstream>
#include <string>

using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
namespace systems {

ALIPTrajGenerator::ALIPTrajGenerator(
    const MultibodyPlant<double>& plant, Context<double>* context,
    double desired_com_height, const vector<int>& unordered_fsm_states,
    const vector<double>& unordered_state_durations,
    const vector<vector<std::pair<const Eigen::Vector3d,
                                  const drake::multibody::Frame<double>&>>>&
        contact_points_in_each_state,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
    : plant_(plant),
      context_(context),
      desired_com_height_(desired_com_height),
      unordered_fsm_states_(unordered_fsm_states),
      unordered_state_durations_(unordered_state_durations),
      contact_points_in_each_state_(contact_points_in_each_state),
      world_(plant_.world_frame()) {
  DRAKE_DEMAND(unordered_fsm_states.size() == unordered_state_durations.size());
  DRAKE_DEMAND(unordered_fsm_states.size() ==
               contact_points_in_each_state.size());

  this->set_name("ALIP_traj");

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        "x, u, t", OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
                    .get_index();
  fsm_port_ =
      this->DeclareVectorInputPort("fsm", BasicVector<double>(1)).get_index();
  touchdown_time_port_ =
      this->DeclareVectorInputPort("t_touchdown", BasicVector<double>(1))
          .get_index();

  // Provide an instance to allocate the memory first (for the output)
  ExponentialPlusPiecewisePolynomial<double> exp;
  drake::trajectories::Trajectory<double>& traj_inst = exp;
  output_port_com_ = this->DeclareAbstractOutputPort(
                             "alip_com_prediction", traj_inst,
                             &ALIPTrajGenerator::CalcComTrajFromCurrent)
                         .get_index();
  output_port_alip_state_ = this->DeclareAbstractOutputPort(
                                    "alip x, y, Lx, Ly prediction", traj_inst,
                                    &ALIPTrajGenerator::CalcAlipTrajFromCurrent)
                                .get_index();

  m_ = plant_.CalcTotalMass(*context);

  MatrixXd A = CalcA(desired_com_height);
  MatrixXd B = -MatrixXd::Identity(4, 2);
  MatrixXd C = MatrixXd::Identity(4, 4);
  MatrixXd G = MatrixXd::Identity(4, 4);

  S2SKalmanFilterData filter_data = {A, B, C, Q, R, G};
  S2SKalmanFilter filter = S2SKalmanFilter(filter_data);
  std::pair<S2SKalmanFilter, S2SKalmanFilterData> model_filter = {filter,
                                                                  filter_data};
  alip_filter_idx_ = this->DeclareAbstractState(
      drake::Value<std::pair<S2SKalmanFilter, S2SKalmanFilterData>>(
          model_filter));

  prev_foot_idx_ = this->DeclareDiscreteState(Vector2d::Zero());
  prev_fsm_idx_ = this->DeclareDiscreteState(-1 * VectorXd::Ones(1));
  com_z_idx_ =
      this->DeclareDiscreteState(desired_com_height * VectorXd::Ones(1));

  this->DeclarePerStepUnrestrictedUpdateEvent(
      &ALIPTrajGenerator::UnrestrictedUpdate);
}

drake::systems::EventStatus ALIPTrajGenerator::UnrestrictedUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::State<double>* state) const {
  int prev_fsm = state->get_discrete_state(prev_fsm_idx_).value()(0);
  auto& [filter, filter_data] = state->get_mutable_abstract_state<
      std::pair<S2SKalmanFilter, S2SKalmanFilterData>>(alip_filter_idx_);

  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd v = robot_output->GetVelocities();

  // Read in finite state machine
  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();
  int mode_index = GetModeIdx((int)fsm_state(0));

  // calculate current estimate of ALIP state
  Vector3d CoM, L, stance_foot_pos;
  CalcAlipState(robot_output->GetState(), mode_index, &CoM, &L,
                &stance_foot_pos);

  Vector4d x_alip;
  x_alip.head(2) = CoM.head(2) - stance_foot_pos.head(2);
  x_alip.tail(2) = L.head(2);

  // Filter the alip state
  filter_data.A = CalcA(CoM(2));
  if ((int)fsm_state(0) == prev_fsm) {
    filter.Update(filter_data, Vector2d::Zero(), x_alip,
                  robot_output->get_timestamp());
  } else {
    Vector2d prev_stance_pos =
        state->get_discrete_state(prev_foot_idx_).value();
    filter.Update(filter_data, stance_foot_pos.head<2>() - prev_stance_pos,
                  x_alip, robot_output->get_timestamp());
    state->get_mutable_discrete_state(prev_fsm_idx_).get_mutable_value()
        << fsm_state;
  }
  state->get_mutable_discrete_state(com_z_idx_).get_mutable_value()
      << CoM.tail(1) - stance_foot_pos.tail(1);
  state->get_mutable_discrete_state(prev_foot_idx_).get_mutable_value()
      << stance_foot_pos.head<2>();
  return EventStatus::Succeeded();
}

ExponentialPlusPiecewisePolynomial<double>
ALIPTrajGenerator::ConstructAlipComTraj(
    const Vector3d& CoM, const Vector3d& stance_foot_pos,
    const Vector4d& x_alip, double start_time,
    double end_time_of_this_fsm_state) const {
  double CoM_wrt_foot_z = (CoM(2) - stance_foot_pos(2));
  DRAKE_DEMAND(CoM_wrt_foot_z > 0);

  // create a 3D one-segment polynomial for ExponentialPlusPiecewisePolynomial
  Vector2d T_waypoint_com{start_time, end_time_of_this_fsm_state};
  MatrixXd Y = MatrixXd::Zero(3, 2);
  Y.col(0).head(2) = stance_foot_pos.head(2);
  Y.col(1).head(2) = stance_foot_pos.head(2);

  // We add stance_foot_pos(2) to desired COM height to account for state
  // drifting
  double max_height_diff_per_step = 0.05;
  double final_height = std::clamp(
      desired_com_height_ + stance_foot_pos(2),
      CoM(2) - max_height_diff_per_step, CoM(2) + max_height_diff_per_step);
  //  double final_height = desired_com_height_ + stance_foot_pos(2);
  Y(2, 0) = final_height;
  Y(2, 1) = final_height;

  Vector3d Y_dot_start = Vector3d::Zero();
  VectorXd Y_dot_end = Vector3d::Zero();

  PiecewisePolynomial<double> pp_part =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          T_waypoint_com, Y, Y_dot_start, Y_dot_end);

  MatrixXd K = MatrixXd::Zero(3, 4);
  K.topLeftCorner(2, 2) = MatrixXd::Identity(2, 2);
  auto A = CalcA(CoM_wrt_foot_z);

  return ExponentialPlusPiecewisePolynomial<double>(K, A, x_alip, pp_part);
}

Eigen::MatrixXd ALIPTrajGenerator::CalcA(double com_z) const {
  // Dynamics of ALIP: (eqn 6) https://arxiv.org/pdf/2109.14862.pdf
  const double g = 9.81;
  double a1x = 1.0 / (m_ * com_z);
  double a2x = -m_ * g;
  double a1y = -1.0 / (m_ * com_z);
  double a2y = m_ * g;

  // Sum of two exponential + one-segment 3D polynomial
  MatrixXd A = MatrixXd::Zero(4, 4);
  A(0, 3) = a1x;
  A(1, 2) = a1y;
  A(2, 1) = a2x;
  A(3, 0) = a2y;
  return A;
}

ExponentialPlusPiecewisePolynomial<double>
ALIPTrajGenerator::ConstructAlipStateTraj(
    const Eigen::Vector4d& x_alip, double com_z, double start_time,
    double end_time_of_this_fsm_state) const {
  Vector2d breaks = {start_time, end_time_of_this_fsm_state};
  PiecewisePolynomial<double> pp_part =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          breaks, MatrixXd::Zero(4, 2), Vector4d::Zero(), Vector4d::Zero());
  MatrixXd K = MatrixXd::Identity(4, 4);

  return ExponentialPlusPiecewisePolynomial<double>(K, CalcA(com_z), x_alip,
                                                    pp_part);
}

void ALIPTrajGenerator::CalcAlipState(
    const Eigen::VectorXd& x, int mode_index,
    const drake::EigenPtr<Eigen::Vector3d>& CoM_p,
    const drake::EigenPtr<Eigen::Vector3d>& L_p,
    const drake::EigenPtr<Eigen::Vector3d>& stance_pos_p) const {
  multibody::SetPositionsAndVelocitiesIfNew<double>(plant_, x, context_);
  Vector3d CoM = plant_.CalcCenterOfMassPositionInWorld(*context_);

  // Take average of contact points a stance position
  Vector3d stance_foot_pos = Vector3d::Zero();
  for (const auto& stance_foot : contact_points_in_each_state_[mode_index]) {
    Vector3d position;
    plant_.CalcPointsPositions(*context_, stance_foot.second, stance_foot.first,
                               world_, &position);
    stance_foot_pos += position;
  }
  stance_foot_pos /= contact_points_in_each_state_[mode_index].size();

  Vector3d L =
      plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, stance_foot_pos)
          .rotational();

  *CoM_p = CoM;
  *L_p = L;
  *stance_pos_p = stance_foot_pos;
}

void ALIPTrajGenerator::CalcComTrajFromCurrent(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  // Read in finite state machine
  int fsm_state = this->EvalVectorInput(context, fsm_port_)->value()(0);

  // Read in finite state machine switch time
  VectorXd prev_event_time =
      this->EvalVectorInput(context, touchdown_time_port_)->get_value();

  int mode_index = GetModeIdx(fsm_state);

  // Get time
  double timestamp = robot_output->get_timestamp();
  double start_time = timestamp;
  double end_time = prev_event_time(0) + unordered_state_durations_[mode_index];
  start_time = std::clamp(
      start_time, -std::numeric_limits<double>::infinity(), end_time - 0.001);

  Vector3d CoM, L, stance_foot_pos;
  CalcAlipState(robot_output->GetState(), mode_index, &CoM, &L,
                &stance_foot_pos);

  Vector4d x_alip =
      context
          .get_abstract_state<std::pair<S2SKalmanFilter, S2SKalmanFilterData>>(
              alip_filter_idx_)
          .first.x();

  // Assign traj
  auto exp_pp_traj = (ExponentialPlusPiecewisePolynomial<double>*)dynamic_cast<
      ExponentialPlusPiecewisePolynomial<double>*>(traj);
  *exp_pp_traj =
      ConstructAlipComTraj(CoM, stance_foot_pos, x_alip, start_time, end_time);
}

void ALIPTrajGenerator::CalcAlipTrajFromCurrent(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  // Read in finite state machine
  int fsm_state = this->EvalVectorInput(context, fsm_port_)->value()(0);

  // Read in finite state machine switch time
  VectorXd prev_event_time =
      this->EvalVectorInput(context, touchdown_time_port_)->get_value();

  int mode_index = GetModeIdx(fsm_state);

  // Get time
  double timestamp = robot_output->get_timestamp();
  double start_time = timestamp;
  double end_time = prev_event_time(0) + unordered_state_durations_[mode_index];
  start_time = std::clamp(
      start_time, -std::numeric_limits<double>::infinity(), end_time - 0.001);

  // Assign traj
  auto exp_pp_traj = (ExponentialPlusPiecewisePolynomial<double>*)dynamic_cast<
      ExponentialPlusPiecewisePolynomial<double>*>(traj);

  Vector4d x_alip =
      context
          .get_abstract_state<std::pair<S2SKalmanFilter, S2SKalmanFilterData>>(
              alip_filter_idx_)
          .first.x();
  double com_z = context.get_discrete_state(com_z_idx_).value()(0);

  *exp_pp_traj = ConstructAlipStateTraj(x_alip, com_z, start_time, end_time);
}

int ALIPTrajGenerator::GetModeIdx(int fsm_state) const {
  auto it = find(unordered_fsm_states_.begin(), unordered_fsm_states_.end(),
                 fsm_state);
  int mode_index = std::distance(unordered_fsm_states_.begin(), it);
  DRAKE_DEMAND(it != unordered_fsm_states_.end());
  return mode_index;
}

}  // namespace systems
}  // namespace dairlib
