#pragma once

#include <memory>
#include <utility>

#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "examples/Cassie/osc/osc_walking_gains.h"
#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "systems/controllers/osc/com_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/osc_gains.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"

namespace dairlib {
namespace examples {
namespace controllers {

using drake::multibody::Frame;
using Eigen::Vector3d;
using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

class OSCWalkingControllerDiagram final
    : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OSCWalkingControllerDiagram)

  /// @param[in] osc_gains_filename filepath containing the osc_running_gains.
  /// @param[in] osqp_settings filepath containing the osqp settings.
  OSCWalkingControllerDiagram(drake::multibody::MultibodyPlant<double>& plant,
                              bool has_double_stance,
                              const std::string& osc_gains_filename,
                              const std::string& osqp_settings_filename);

  /// @return the input port for the plant state.
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_input_port_index_);
  }

  /// @return the input port for the cassie_out struct (containing radio
  /// commands).
  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_input_port_index_);
  }

  /// @return the output port for the controller torques.
  const drake::systems::OutputPort<double>& get_output_port_robot_input() const {
    return this->get_output_port(control_output_port_index_);
  }

  /// @return the output port for the controller torques.
  const drake::systems::OutputPort<double>& get_output_port_torque() const {
    return this->get_output_port(torque_output_port_index_);
  }

  /// @return the output port for the failure status of the controller.
  const drake::systems::OutputPort<double>& get_output_port_controller_failure()
      const {
    return this->get_output_port(controller_failure_port_index_);
  }

  drake::multibody::MultibodyPlant<double>& get_plant() {
    return *plant_;
  }

 private:
  drake::multibody::MultibodyPlant<double>* plant_;
  std::map<std::string, int> pos_map;
  std::map<std::string, int> vel_map;
  std::map<std::string, int> act_map;
  std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
      left_toe;
  std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
      left_heel;
  std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
      right_toe;
  std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
      right_heel;
  std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
      left_toe_mid;
  std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
      right_toe_mid;
  std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
      left_toe_origin;
  std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
      right_toe_origin;
  std::vector<std::pair<const Vector3d, const Frame<double>&>> left_right_foot;
  std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>
      left_foot_points;
  std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>
      right_foot_points;
  std::shared_ptr<multibody::WorldYawViewFrame<double>> view_frame_;
  multibody::WorldPointEvaluator<double> left_toe_evaluator;
  multibody::WorldPointEvaluator<double> left_heel_evaluator;
  multibody::WorldPointEvaluator<double> right_toe_evaluator;
  multibody::WorldPointEvaluator<double> right_heel_evaluator;
  multibody::DistanceEvaluator<double> left_loop;
  multibody::DistanceEvaluator<double> right_loop;
  std::unordered_map<
      int, std::vector<std::pair<const Eigen::Vector3d,
                                 const drake::multibody::Frame<double>&>>>
      feet_contact_points;
  std::vector<int> fsm_states;
  std::vector<double> state_durations;
  std::vector<int> single_support_states;
  std::vector<int> double_support_states;
  std::vector<int> unordered_fsm_states;
  std::vector<double> unordered_state_durations;
  std::vector<std::vector<std::pair<const Vector3d, const Frame<double>&>>>
      contact_points_in_each_state;
  std::unique_ptr<drake::systems::Context<double>> plant_context;
  std::vector<int> left_right_support_fsm_states;
  std::vector<double> left_right_support_state_durations;
  std::vector<double> swing_ft_gain_multiplier_breaks;
  std::vector<drake::MatrixX<double>> swing_ft_gain_multiplier_samples;
//  drake::trajectories::PiecewisePolynomial<double>
//      swing_ft_gain_multiplier_gain_multiplier;
  std::vector<double> swing_ft_accel_gain_multiplier_breaks;
  std::vector<drake::MatrixX<double>> swing_ft_accel_gain_multiplier_samples;
//  std::shared_ptr<drake::trajectories::Trajectory<double>>
//      swing_ft_accel_gain_multiplier_gain_multiplier;

  multibody::KinematicEvaluatorSet<double> evaluators;

  multibody::FixedJointEvaluator<double> left_fixed_knee_spring;
  multibody::FixedJointEvaluator<double> right_fixed_knee_spring;
  multibody::FixedJointEvaluator<double> left_fixed_ankle_spring;
  multibody::FixedJointEvaluator<double> right_fixed_ankle_spring;

  std::unique_ptr<TransTaskSpaceTrackingData> swing_foot_data;
  std::unique_ptr<ComTrackingData> com_data;
  std::unique_ptr<RelativeTranslationTrackingData> swing_ft_traj_local_;
  std::unique_ptr<TransTaskSpaceTrackingData> swing_ft_traj_global_;

  std::unique_ptr<TransTaskSpaceTrackingData> pelvis_traj_;

  std::unique_ptr<RotTaskSpaceTrackingData> pelvis_balance_traj_;
  std::unique_ptr<RotTaskSpaceTrackingData> pelvis_heading_traj_;

  std::unique_ptr<JointSpaceTrackingData> swing_toe_traj_left_;
  std::unique_ptr<JointSpaceTrackingData> swing_toe_traj_right_;

  std::unique_ptr<JointSpaceTrackingData> swing_hip_yaw_traj_;

  const int state_input_port_index_ = 0;
  const int radio_input_port_index_ = 1;
  const int control_output_port_index_ = 0;
  const int torque_output_port_index_ = 1;
  const int controller_failure_port_index_ = 2;

  const std::string control_channel_name_ = "OSC_WALKING";
};

}  // namespace controllers
}  // namespace examples
}  // namespace dairlib
