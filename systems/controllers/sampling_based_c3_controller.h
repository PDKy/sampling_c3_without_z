#pragma once

#include <string>
#include <vector>

#include <drake/common/yaml/yaml_io.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "dairlib/lcmt_sampling_c3_debug.hpp"
#include "lcm/lcm_trajectory.h"
#include "solvers/c3.h"

#include "solvers/c3_options.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "sampling_params.h"
#include "solvers/c3_output.h"
#include "solvers/lcs.h"
#include "solvers/lcs_factory.h"
#include "solvers/solver_options_io.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"
#include <queue>




namespace dairlib {
using systems::TimestampedVector;
using drake::systems::BasicVector;
using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::SortedPair;
using drake::VectorX;
using drake::geometry::GeometryId;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace systems {

/// Outputs a lcmt_timestamped_saved_traj
class SamplingC3Controller : public drake::systems::LeafSystem<double> {
 public:
  explicit SamplingC3Controller(
      drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      drake::systems::Context<drake::AutoDiffXd>* context_ad,
      const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_geoms,
      C3Options c3_options,
      SamplingC3Options sampling_c3_options,
      SamplingC3SamplingParams sampling_params,
      drake::multibody::ModelInstanceIndex franka_lcs_index,
      drake::multibody::ModelInstanceIndex object_lcs_index,

      bool verbose = false);

  const drake::systems::InputPort<double>& get_input_port_target() const {
    return this->get_input_port(target_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_final_target() const {
    return this->get_input_port(final_target_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_lcs_state() const {
    return this->get_input_port(lcs_state_input_port_);
  }

  // add separate port
  const drake::systems::InputPort<double>& get_input_port_franka_lcs_state() const {
    return this->get_input_port(franka_lcs_state_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_franka_target() const {
    return this->get_input_port(franka_target_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_franka_final_target() const {
    return this->get_input_port(franka_final_target_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_object_lcs_state() const {
    return this->get_input_port(object_lcs_state_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_object_target() const {
    return this->get_input_port(object_target_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_object_final_target() const {
    return this->get_input_port(object_final_target_input_port_);
  }


  // Current location plan output ports
  const drake::systems::OutputPort<double>& get_output_port_c3_solution_curr_plan()
      const {
    return this->get_output_port(c3_solution_curr_plan_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_solution_curr_plan_actor()
      const {
    return this->get_output_port(c3_solution_curr_plan_actor_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_solution_curr_plan_object()
      const {
    return this->get_output_port(c3_solution_curr_plan_object_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_intermediates_curr_plan()
      const {
    return this->get_output_port(c3_intermediates_curr_plan_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_lcs_contact_jacobian_curr_plan() 
      const {
    return this->get_output_port(lcs_contact_jacobian_curr_plan_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_dynamically_feasible_curr_plan_actor() 
      const {
    return this->get_output_port(dynamically_feasible_curr_plan_actor_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_dynamically_feasible_curr_plan_object() 
      const {
    return this->get_output_port(dynamically_feasible_curr_plan_object_port_);
  }

  // Best sample plan output ports
  const drake::systems::OutputPort<double>& get_output_port_c3_solution_best_plan()
      const {
    return this->get_output_port(c3_solution_best_plan_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_solution_best_plan_actor()
      const {
    return this->get_output_port(c3_solution_best_plan_actor_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_solution_best_plan_object()
      const {
    return this->get_output_port(c3_solution_best_plan_object_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_intermediates_best_plan()
      const {
    return this->get_output_port(c3_intermediates_best_plan_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_lcs_contact_jacobian_best_plan() 
      const {
    return this->get_output_port(lcs_contact_jacobian_best_plan_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_dynamically_feasible_best_plan_actor() 
      const {
    return this->get_output_port(dynamically_feasible_best_plan_actor_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_dynamically_feasible_best_plan_object() 
      const {
    return this->get_output_port(dynamically_feasible_best_plan_object_port_);
  }


  // Execution trajectory output ports
  const drake::systems::OutputPort<double>& get_output_port_c3_traj_execute_actor() 
      const {
    return this->get_output_port(c3_traj_execute_actor_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_repos_traj_execute_actor() 
      const {
    return this->get_output_port(repos_traj_execute_actor_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_traj_execute_actor() 
      const {
    return this->get_output_port(traj_execute_actor_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_c3_traj_execute_object() 
      const {
    return this->get_output_port(c3_traj_execute_object_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_repos_traj_execute_object() 
      const {
    return this->get_output_port(repos_traj_execute_object_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_traj_execute_object() 
      const {
    return this->get_output_port(traj_execute_object_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_is_c3_mode() 
      const {
    return this->get_output_port(is_c3_mode_port_);
  }

  // Sample related output ports
  const drake::systems::OutputPort<double>& get_output_port_all_sample_locations() 
      const {
    return this->get_output_port(all_sample_locations_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_all_sample_costs() 
      const {
    return this->get_output_port(all_sample_costs_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_curr_and_best_sample_costs() 
      const {
    return this->get_output_port(curr_and_best_sample_costs_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_debug()
      const {
    return this->get_output_port(debug_lcmt_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_sample_buffer_configurations()
      const {
    return this->get_output_port(sample_buffer_configurations_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_sample_buffer_costs()
      const {
    return this->get_output_port(sample_buffer_costs_port_);
  }


  // The solver options need not be done twice i.e. one for each c3 solution 
  // object.
  void SetOsqpSolverOptions(const drake::solvers::SolverOptions& options) {
    solver_options_ = options;
    c3_curr_plan_->SetOsqpSolverOptions(solver_options_);
  }

 private:
  solvers::LCS CreatePlaceholderLCS() const;

  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;
  
  void ClampEndEffectorAcceleration(drake::VectorX<double>& x_lcs_curr) const;

  void UpdateContext(Eigen::VectorXd lcs_state) const;

  void UpdateC3ExecutionTrajectory(
    const Eigen::VectorXd& x_lcs, const double& t_context) const;

  void UpdateRepositioningExecutionTrajectory(
    const Eigen::VectorXd& x_lcs, const double& t_context) const;

  void MaintainSampleBuffer(const Eigen::VectorXd& x_lcs, const int& franka_n_q) const;

  void OutputC3SolutionCurrPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const;

  void OutputC3SolutionCurrPlanActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputC3SolutionCurrPlanObject(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const;

  void OutputC3IntermediatesCurrPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const;

  void OutputLCSContactJacobianCurrPlan(
    const drake::systems::Context<double>& context,
    std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>>* lcs_contact_jacobian) const;

  void OutputC3SolutionBestPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Solution* c3_solution) const;
    
  void OutputC3SolutionBestPlanActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const;
  void OutputC3SolutionBestPlanObject(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output) const;

  void OutputC3IntermediatesBestPlan(
    const drake::systems::Context<double>& context,
    C3Output::C3Intermediates* c3_intermediates) const;

  void OutputLCSContactJacobianBestPlan(
    const drake::systems::Context<double>& context,
    std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>>* lcs_contact_jacobian) const;

  void OutputDynamicallyFeasibleCurrPlanActor(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* dynamically_feasible_curr_plan_actor) const;

  void OutputDynamicallyFeasibleCurrPlanObject(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* dynamically_feasible_curr_plan_object) const;

  void OutputDynamicallyFeasibleBestPlanActor(
    const drake::systems::Context<double>& context,
    lcmt_timestamped_saved_traj* dynamically_feasible_best_plan_actor) const;

  void OutputDynamicallyFeasibleBestPlanObject(
    const drake::systems::Context<double>& context,
    lcmt_timestamped_saved_traj* dynamically_feasible_best_plan_object) const;

  void OutputAllSampleLocations(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* all_sample_locations) const;

  void OutputAllSampleCosts(
    const drake::systems::Context<double>& context,
    lcmt_timestamped_saved_traj* output_all_sample_costs) const;

  void OutputC3TrajExecuteActor(
    const drake::systems::Context<double>& context,
    lcmt_timestamped_saved_traj* c3_execution_lcm_traj) const;
  void OutputC3TrajExecuteObject(
    const drake::systems::Context<double>& context,
    lcmt_timestamped_saved_traj* c3_execution_lcm_traj) const;

  void OutputReposTrajExecuteActor(
    const drake::systems::Context<double>& context,
    lcmt_timestamped_saved_traj* repos_execution_lcm_traj) const;
  void OutputReposTrajExecuteObject(
    const drake::systems::Context<double>& context,
    lcmt_timestamped_saved_traj* repos_execution_lcm_traj) const;

  void OutputTrajExecuteActor(
    const drake::systems::Context<double>& context,
    lcmt_timestamped_saved_traj* execution_lcm_traj) const;
  void OutputTrajExecuteObject(
    const drake::systems::Context<double>& context,
    lcmt_timestamped_saved_traj* execution_lcm_traj) const;

  void OutputIsC3Mode(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* is_c3_mode) const;

  void OutputCurrAndBestSampleCost(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_curr_and_best_sample_cost) const;

  void OutputDebug(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_sampling_c3_debug* debug_msg) const;

  void OutputSampleBufferConfigurations(
    const drake::systems::Context<double>& context,
    Eigen::MatrixXd* sample_buffer_configurations) const;

  void OutputSampleBufferCosts(
    const drake::systems::Context<double>& context,
    Eigen::VectorXd* sample_buffer_costs) const;

  drake::systems::InputPortIndex radio_port_;
  drake::systems::InputPortIndex final_target_input_port_;
  drake::systems::InputPortIndex target_input_port_;
  drake::systems::InputPortIndex lcs_state_input_port_;


  // add separate port
  drake::systems::InputPortIndex franka_lcs_state_input_port_;
  drake::systems::InputPortIndex franka_target_input_port_;
  drake::systems::InputPortIndex franka_final_target_input_port_;

  drake::systems::InputPortIndex object_lcs_state_input_port_;
  drake::systems::InputPortIndex object_target_input_port_;
  drake::systems::InputPortIndex object_final_target_input_port_;



  // Current sample output port indices
  drake::systems::OutputPortIndex c3_solution_curr_plan_port_;
  drake::systems::OutputPortIndex c3_solution_curr_plan_actor_port_;
  drake::systems::OutputPortIndex c3_solution_curr_plan_object_port_;
  drake::systems::OutputPortIndex c3_intermediates_curr_plan_port_;
  drake::systems::OutputPortIndex lcs_contact_jacobian_curr_plan_port_;
  // Best sample output port indices
  drake::systems::OutputPortIndex c3_solution_best_plan_port_;
  drake::systems::OutputPortIndex c3_solution_best_plan_actor_port_;
  drake::systems::OutputPortIndex c3_solution_best_plan_object_port_;
  drake::systems::OutputPortIndex c3_intermediates_best_plan_port_;
  drake::systems::OutputPortIndex lcs_contact_jacobian_best_plan_port_;
  // Execution trajectory output port indices
  drake::systems::OutputPortIndex c3_traj_execute_actor_port_;
  drake::systems::OutputPortIndex repos_traj_execute_actor_port_;
  drake::systems::OutputPortIndex traj_execute_actor_port_;
  drake::systems::OutputPortIndex c3_traj_execute_object_port_;
  drake::systems::OutputPortIndex repos_traj_execute_object_port_;
  drake::systems::OutputPortIndex traj_execute_object_port_;
  drake::systems::OutputPortIndex is_c3_mode_port_;
  // Dynamically feasible plan output port indices
  drake::systems::OutputPortIndex dynamically_feasible_curr_plan_actor_port_;
  drake::systems::OutputPortIndex dynamically_feasible_curr_plan_object_port_;
  drake::systems::OutputPortIndex dynamically_feasible_best_plan_actor_port_;
  drake::systems::OutputPortIndex dynamically_feasible_best_plan_object_port_;
  // Sample related output port indices
  drake::systems::OutputPortIndex all_sample_locations_port_;
  drake::systems::OutputPortIndex all_sample_costs_port_;
  drake::systems::OutputPortIndex curr_and_best_sample_costs_port_;
  drake::systems::OutputPortIndex debug_lcmt_port_;
  drake::systems::OutputPortIndex sample_buffer_configurations_port_;
  drake::systems::OutputPortIndex sample_buffer_costs_port_;

  // This plant_ has been made 'not const' so that the context can be updated.
  drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  drake::systems::Context<drake::AutoDiffXd>* context_ad_;
  const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
      contact_pairs_;
  solvers::ContactModel contact_model_;
  C3Options c3_options_;
  SamplingC3Options sampling_c3_options_;
  SamplingC3SamplingParams sampling_params_;
  drake::solvers::SolverOptions solver_options_ =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          "solvers/osqp_options_default.yaml")
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  // convenience for variable sizes
  const bool verbose_;
  int n_q_;
  int n_v_;
  int n_x_;
  int n_lambda_;
  int n_u_;
  int max_num_samples_;
  mutable double dt_ = 0.1;

  int franka_n_q_;
  int franka_n_v_;
  int franka_n_x_;

  int object_n_q_;
  int object_n_v_;
  int object_n_x_;

  //To give the C3 state back to the controller, we need to add z-axis back, so need to hard code those numbers.
  //full_n_q: ee: xyz(3) | Quaternion object(4) | object xyz(3)|
  int full_n_q_ = 3+4+3;
  int full_n_v_ = 3+3+3;
  int full_n_x_ = full_n_q_ + full_n_v_;

  //OutPutPort
  const BasicVector<double> x_lcs_des;
  const BasicVector<double> x_lcs_final_des;

  double solve_time_filter_constant_;
  drake::systems::DiscreteStateIndex plan_start_time_index_;
  mutable std::vector<Eigen::MatrixXd> Q_;
  std::vector<Eigen::MatrixXd> R_;
  mutable std::vector<Eigen::MatrixXd> G_;
  mutable std::vector<Eigen::MatrixXd> G_for_curr_location_;
  mutable std::vector<Eigen::MatrixXd> U_;
  mutable std::vector<Eigen::MatrixXd> U_for_curr_location_;
  int N_;

  // Keep track of current C3 execution's best seen cost.
  mutable int best_progress_steps_ago_;
  mutable double lowest_cost_;
  mutable double lowest_pos_and_rot_current_cost_;
  mutable double lowest_position_error_;
  mutable double lowest_orientation_error_;
  mutable double current_position_error_;
  mutable double current_orientation_error_;
  // Keep track of costs seen so far in C3.
  mutable std::queue<double> progress_cost_buffer_;

  mutable double filtered_solve_time_ = 0;

  // Predictions for the end effector location.
  mutable Eigen::VectorXd x_pred_curr_plan_;
  mutable Eigen::VectorXd x_from_last_control_loop_;
  mutable Eigen::VectorXd x_pred_from_last_control_loop_;

  // C3 solution for current location.
  mutable std::shared_ptr<solvers::C3> c3_curr_plan_;
  // TODO: these are currently unused but may be useful if implementing warm start.
  mutable std::vector<Eigen::VectorXd> z_sol_curr_plan_;
  mutable std::vector<Eigen::VectorXd> delta_curr_plan_;
  mutable std::vector<Eigen::VectorXd> w_curr_plan_;
  
  // C3 solution for best sample location.
  mutable std::shared_ptr<solvers::C3> c3_best_plan_;
  // TODO: these are currently unused but may be useful if implementing warm start.
  mutable std::vector<Eigen::VectorXd> z_sol_best_plan_;
  mutable std::vector<Eigen::VectorXd> delta_best_plan_;
  mutable std::vector<Eigen::VectorXd> w_best_plan_;

  // C3 solution for best sample in buffer.
  mutable std::shared_ptr<solvers::C3> c3_buffer_plan_;
  mutable std::vector<Eigen::VectorXd> dynamically_feasible_buffer_plan_;


  // LCS trajectories for C3 or repositioning modes.
  // mutable std::vector<TimestampedVector<double>> c3_traj_execute_;
  // mutable std::vector<TimestampedVector<double>> repos_traj_execute_;
  mutable LcmTrajectory c3_execution_lcm_traj_;
  mutable LcmTrajectory repos_execution_lcm_traj_;
 

  // Samples and associated costs computed in current control loop.
  mutable std::vector<Eigen::Vector3d> all_sample_locations_;
  mutable std::vector<std::vector<Eigen::VectorXd>> all_sample_dynamically_feasible_plans_;
  mutable Eigen::Vector3d prev_repositioning_target_ = Eigen::Vector3d::Zero();
  mutable std::vector<double> all_sample_costs_;
  mutable std::vector<double> curr_and_best_sample_cost_;

  // To detect if the final goal has been updated.
  mutable Eigen::VectorXd x_final_target_;
  mutable int detected_goal_changes_ = -1;

  // For more intelligent sampling.
  mutable int num_in_buffer_ = 0;
  mutable Eigen::MatrixXd sample_buffer_;       // (N_sample_buffer x n_q)
  mutable Eigen::VectorXd sample_costs_buffer_;

  // Miscellaneous sample related variables.
  mutable bool is_doing_c3_ = true;
  mutable bool finished_reposition_flag_ = false;
  // This flag is meant to indicate the first time the object reaches within 
  // some fixed radius of the target. When this is true, the controller will
  // change the cost to care about the object orientation as well.
  mutable bool crossed_cost_switching_threshold_ = false;
  mutable int num_threads_to_use_;

  enum SampleIndex { CURRENT_LOCATION_INDEX,
                     SAMPLE_INDEX_1, SAMPLE_INDEX_2, SAMPLE_INDEX_3,
                     SAMPLE_INDEX_4, SAMPLE_INDEX_5, SAMPLE_INDEX_6,
                     SAMPLE_INDEX_7, SAMPLE_INDEX_8, SAMPLE_INDEX_9,
                     SAMPLE_INDEX_10, SAMPLE_INDEX_11, SAMPLE_INDEX_12 };
  const SampleIndex CURRENT_REPOSITION_INDEX = SAMPLE_INDEX_1;
  mutable SampleIndex best_sample_index_ = CURRENT_LOCATION_INDEX;

  enum ModeSwitchReason {MODE_SWITCH_REASON_NONE,
                         MODE_SWITCH_TO_C3_COST,
                         MODE_SWITCH_TO_C3_REACHED_REPOS_GOAL,
                         MODE_SWITCH_TO_REPOS_COST,
                         MODE_SWITCH_TO_REPOS_UNPRODUCTIVE,
                         MODE_SWITCH_TO_C3_XBOX};
  mutable ModeSwitchReason mode_switch_reason_ = MODE_SWITCH_REASON_NONE;

  enum PursuedTargetSource {TARGET_SOURCE_NONE,
                            TARGET_SOURCE_PREVIOUS,
                            TARGET_SOURCE_NEW_SAMPLE,
                            TARGET_SOURCE_FROM_BUFFER};
  mutable PursuedTargetSource pursued_target_source_ = TARGET_SOURCE_NONE;

  //index of the models

  drake::multibody::ModelInstanceIndex franka_lcs_index_;
  drake::multibody::ModelInstanceIndex object_lcs_index_;

};

}  // namespace systems
}  // namespace dairlib
