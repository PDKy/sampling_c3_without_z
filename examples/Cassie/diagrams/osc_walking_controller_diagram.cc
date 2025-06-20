#include "osc_walking_controller_diagram.h"

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/heading_traj_generator.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/osc/osc_walking_gains.h"
#include "examples/Cassie/osc/osc_walking_gains_alip.h"
#include "examples/Cassie/osc/swing_toe_traj_generator.h"
#include "examples/Cassie/osc/walking_speed_control.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/alip_swing_ft_traj_gen.h"
#include "systems/controllers/alip_traj_gen.h"
#include "systems/controllers/fsm_event_time.h"
#include "systems/controllers/lipm_traj_gen.h"
#include "systems/controllers/osc/com_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/options_tracking_data.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/controllers/swing_ft_traj_gen.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/radio_parser.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {

using std::map;
using std::pair;
using std::string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using cassie::osc::SwingToeTrajGenerator;
using drake::geometry::SceneGraph;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
// using drake::systems::lcm::LcmPublisherSystem;
// using drake::systems::lcm::LcmSubscriberSystem;
using drake::trajectories::PiecewisePolynomial;
using multibody::FixedJointEvaluator;
using multibody::WorldYawViewFrame;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

namespace examples {
namespace controllers {

OSCWalkingControllerDiagram::OSCWalkingControllerDiagram(
    drake::multibody::MultibodyPlant<double>& plant, bool has_double_stance,
    const string& osc_walking_gains_filename,
    const string& osqp_settings_filename)
    : plant_(&plant),
      pos_map(multibody::MakeNameToPositionsMap(plant)),
      vel_map(multibody::MakeNameToVelocitiesMap(plant)),
      act_map(multibody::MakeNameToActuatorsMap(plant)),
      left_toe(LeftToeFront(plant)),
      left_heel(LeftToeRear(plant)),
      right_toe(RightToeFront(plant)),
      right_heel(RightToeRear(plant)),
      left_toe_mid(std::pair<const Vector3d, const Frame<double>&>(
          (left_toe.first + left_heel.first) / 2,
          plant.GetFrameByName("toe_left"))),
      right_toe_mid(std::pair<const Vector3d, const Frame<double>&>(
          (left_toe.first + left_heel.first) / 2,
          plant.GetFrameByName("toe_right"))),
      left_toe_origin(std::pair<const Vector3d, const Frame<double>&>(
          Vector3d::Zero(), plant.GetFrameByName("toe_left"))),
      right_toe_origin(std::pair<const Vector3d, const Frame<double>&>(
          Vector3d::Zero(), plant.GetFrameByName("toe_right"))),
      left_right_foot({left_toe_origin, right_toe_origin}),
      left_foot_points({left_heel, left_toe}),
      right_foot_points({right_heel, right_toe}),
      view_frame_(
          std::make_shared<multibody::WorldYawViewFrame<double>>(plant.GetBodyByName("pelvis"))),
      left_toe_evaluator(multibody::WorldPointEvaluator(
          plant, left_toe.first, left_toe.second, *view_frame_,
          Matrix3d::Identity(), Vector3d::Zero(), {1, 2})),
      left_heel_evaluator(multibody::WorldPointEvaluator(
          plant, left_heel.first, left_heel.second, *view_frame_,
          Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2})),
      right_toe_evaluator(multibody::WorldPointEvaluator(
          plant, right_toe.first, right_toe.second, *view_frame_,
          Matrix3d::Identity(), Vector3d::Zero(), {1, 2})),
      right_heel_evaluator(multibody::WorldPointEvaluator(
          plant, right_heel.first, right_heel.second, *view_frame_,
          Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2})),
      left_loop(LeftLoopClosureEvaluator(plant)),
      right_loop(RightLoopClosureEvaluator(plant)),
      evaluators(multibody::KinematicEvaluatorSet<double>(plant)),
      left_fixed_knee_spring(
          FixedJointEvaluator(plant, pos_map.at("knee_joint_left"),
                              vel_map.at("knee_joint_leftdot"), 0)),
      right_fixed_knee_spring(
          FixedJointEvaluator(plant, pos_map.at("knee_joint_right"),
                              vel_map.at("knee_joint_rightdot"), 0)),
      left_fixed_ankle_spring(
          FixedJointEvaluator(plant, pos_map.at("ankle_spring_joint_left"),
                              vel_map.at("ankle_spring_joint_leftdot"), 0)),
      right_fixed_ankle_spring(
          FixedJointEvaluator(plant, pos_map.at("ankle_spring_joint_right"),
                              vel_map.at("ankle_spring_joint_rightdot"), 0)) {
  // Build the controller diagram
  DiagramBuilder<double> builder;
  plant_context = plant.CreateDefaultContext();
  feet_contact_points[0] = std::vector<
      std::pair<const Vector3d, const drake::multibody::Frame<double>&>>(
      {left_toe, left_heel});
  feet_contact_points[1] = std::vector<
      std::pair<const Vector3d, const drake::multibody::Frame<double>&>>(
      {right_toe, right_heel});

  // Read-in the parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  OSCGains osc_gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(osc_walking_gains_filename), {}, {}, yaml_options);
  OSCWalkingGainsALIP osc_walking_gains =
      drake::yaml::LoadYamlFile<OSCWalkingGainsALIP>(
          osc_walking_gains_filename);

  /**** FSM and contact mode configuration ****/
  int left_stance_state = 0;
  int right_stance_state = 1;
  int post_left_double_support_state = 3;
  int post_right_double_support_state = 4;
  double left_support_duration = osc_walking_gains.ss_time;
  double right_support_duration = osc_walking_gains.ss_time;
  double double_support_duration = osc_walking_gains.ds_time;
  //  vector<int> fsm_states;
  //  vector<double> state_durations;
  if (has_double_stance) {
    fsm_states = {left_stance_state, post_left_double_support_state,
                  right_stance_state, post_right_double_support_state};
    state_durations = {left_support_duration, double_support_duration,
                       right_support_duration, double_support_duration};
    unordered_fsm_states = {left_stance_state, right_stance_state,
                            post_left_double_support_state,
                            post_right_double_support_state};
    unordered_state_durations = {left_support_duration, right_support_duration,
                                 double_support_duration,
                                 double_support_duration};
    contact_points_in_each_state.push_back({left_toe_mid});
    contact_points_in_each_state.push_back({right_toe_mid});
    contact_points_in_each_state.push_back({left_toe_mid});
    contact_points_in_each_state.push_back({right_toe_mid});

  } else {
    fsm_states = {left_stance_state, right_stance_state};
    state_durations = {left_support_duration, right_support_duration};
    unordered_fsm_states = {left_stance_state, right_stance_state};
    unordered_state_durations = {left_support_duration, right_support_duration};
    contact_points_in_each_state.push_back({left_toe_mid});
    contact_points_in_each_state.push_back({right_toe_mid});
  }
  single_support_states = {left_stance_state, right_stance_state};
  double_support_states = {post_left_double_support_state,
                           post_right_double_support_state};
  left_right_support_fsm_states = {left_stance_state, right_stance_state};
  left_right_support_state_durations = {left_support_duration,
                                        right_support_duration};
  swing_ft_gain_multiplier_breaks = {0, left_support_duration / 2,
                                     left_support_duration};
  swing_ft_gain_multiplier_samples = std::vector<drake::MatrixX<double>>(
      3, drake::MatrixX<double>::Identity(3, 3));
  swing_ft_gain_multiplier_samples[2](2, 2) *= 0.3;
  auto swing_ft_gain_multiplier_gain_multiplier =
      std::make_shared<PiecewisePolynomial<double>>(
          PiecewisePolynomial<double>::FirstOrderHold(
              swing_ft_gain_multiplier_breaks,
              swing_ft_gain_multiplier_samples));
  swing_ft_accel_gain_multiplier_breaks = {0, left_support_duration / 2,
                                           left_support_duration * 3 / 4,
                                           left_support_duration};
  swing_ft_accel_gain_multiplier_samples = std::vector<drake::MatrixX<double>>(
      4, drake::MatrixX<double>::Identity(3, 3));
  swing_ft_accel_gain_multiplier_samples[2](2, 2) *= 0;
  swing_ft_accel_gain_multiplier_samples[3](2, 2) *= 0;
  auto swing_ft_accel_gain_multiplier_gain_multiplier =
      std::make_shared<PiecewisePolynomial<double>>(PiecewisePolynomial<double>::FirstOrderHold(
          swing_ft_accel_gain_multiplier_breaks,
          swing_ft_accel_gain_multiplier_samples));

  /**** Initialize all the leaf systems ****/

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), true);
  auto fsm = builder.AddSystem<systems::TimeBasedFiniteStateMachine>(
      plant, fsm_states, state_durations);
  auto liftoff_event_time =
      builder.AddSystem<systems::FiniteStateMachineEventTime>(
          plant, single_support_states);
  auto touchdown_event_time =
      builder.AddSystem<systems::FiniteStateMachineEventTime>(
          plant, double_support_states);
  auto radio_parser = builder.AddSystem<systems::RadioParser>();
  liftoff_event_time->set_name("liftoff_time");
  touchdown_event_time->set_name("touchdown_time");
  /**** OSC setup ****/

  /// REGULARIZATION COSTS
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();
  MatrixXd Q_accel = osc_walking_gains.w_accel * MatrixXd::Identity(n_v, n_v);
  osc->SetAccelerationCostWeights(Q_accel);
  osc->SetInputSmoothingCostWeights(osc_walking_gains.w_input_reg *
                                    MatrixXd::Identity(n_u, n_u));
  // Soft constraint on contacts
  osc->SetContactSoftConstraintWeight(osc_walking_gains.w_soft_constraint);

  // Contact information for OSC
  osc->SetContactFriction(osc_walking_gains.mu);

  osc->AddStateAndContactPoint(left_stance_state, &left_toe_evaluator);
  osc->AddStateAndContactPoint(left_stance_state, &left_heel_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_toe_evaluator);
  osc->AddStateAndContactPoint(right_stance_state, &right_heel_evaluator);
  if (has_double_stance) {
    osc->AddStateAndContactPoint(post_left_double_support_state,
                                 &left_toe_evaluator);
    osc->AddStateAndContactPoint(post_left_double_support_state,
                                 &left_heel_evaluator);
    osc->AddStateAndContactPoint(post_left_double_support_state,
                                 &right_toe_evaluator);
    osc->AddStateAndContactPoint(post_left_double_support_state,
                                 &right_heel_evaluator);
    osc->AddStateAndContactPoint(post_right_double_support_state,
                                 &left_toe_evaluator);
    osc->AddStateAndContactPoint(post_right_double_support_state,
                                 &left_heel_evaluator);
    osc->AddStateAndContactPoint(post_right_double_support_state,
                                 &right_toe_evaluator);
    osc->AddStateAndContactPoint(post_right_double_support_state,
                                 &right_heel_evaluator);
  }

  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Fix the springs in the dynamics
  evaluators.add_evaluator(&left_fixed_knee_spring);
  evaluators.add_evaluator(&right_fixed_knee_spring);
  evaluators.add_evaluator(&left_fixed_ankle_spring);
  evaluators.add_evaluator(&right_fixed_ankle_spring);

  osc->AddKinematicConstraint(&evaluators);

  /**** Trajectory Generators ****/

  std::cout << "Creating output trajectory leaf systems. " << std::endl;

  cassie::osc::HighLevelCommand* high_level_command;
  high_level_command = builder.AddSystem<cassie::osc::HighLevelCommand>(
      plant, plant_context.get(), osc_walking_gains.vel_scale_rot,
      osc_walking_gains.vel_scale_trans_sagital,
      osc_walking_gains.vel_scale_trans_lateral);
  auto head_traj_gen = builder.AddSystem<cassie::osc::HeadingTrajGenerator>(
      plant, plant_context.get());
  auto lipm_traj_generator = builder.AddSystem<systems::LIPMTrajGenerator>(
      plant, plant_context.get(), osc_walking_gains.lipm_height,
      unordered_fsm_states, unordered_state_durations,
      contact_points_in_each_state);
  auto pelvis_traj_generator = builder.AddSystem<systems::LIPMTrajGenerator>(
      plant, plant_context.get(), osc_walking_gains.lipm_height,
      unordered_fsm_states, unordered_state_durations,
      contact_points_in_each_state, false);
  auto swing_ft_traj_generator =
      builder.AddSystem<systems::AlipSwingFootTrajGenerator>(
          plant, plant_context.get(), left_right_support_fsm_states,
          left_right_support_state_durations, left_right_foot, "pelvis",
          double_support_duration, osc_walking_gains.mid_foot_height,
          osc_walking_gains.final_foot_height,
          osc_walking_gains.final_foot_velocity_z,
          osc_walking_gains.max_CoM_to_footstep_dist,
          osc_walking_gains.footstep_offset,
          osc_walking_gains.center_line_offset);
  auto left_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
          plant, plant_context.get(), pos_map["toe_left"], left_foot_points,
          "left_toe_angle_traj");
  auto right_toe_angle_traj_gen =
      builder.AddSystem<cassie::osc::SwingToeTrajGenerator>(
          plant, plant_context.get(), pos_map["toe_right"], right_foot_points,
          "right_toe_angle_traj");

  auto alip_traj_generator = builder.AddSystem<systems::ALIPTrajGenerator>(
      plant, plant_context.get(), osc_walking_gains.lipm_height,
      unordered_fsm_states, unordered_state_durations,
      contact_points_in_each_state,
      osc_walking_gains.Q_alip_kalman_filter.asDiagonal(),
      osc_walking_gains.R_alip_kalman_filter.asDiagonal());

  builder.Connect(fsm->get_output_port(0),
                  alip_traj_generator->get_input_port_fsm());
  builder.Connect(touchdown_event_time->get_output_port_event_time(),
                  alip_traj_generator->get_input_port_touchdown_time());
  builder.Connect(state_receiver->get_output_port(0),
                  alip_traj_generator->get_input_port_state());

  /**** Tracking Data *****/

  std::cout << "Creating tracking data. " << std::endl;

  swing_foot_data = std::make_unique<TransTaskSpaceTrackingData>(
      "swing_ft_data", osc_walking_gains.K_p_swing_foot,
      osc_walking_gains.K_d_swing_foot, osc_walking_gains.W_swing_foot, plant,
      plant);
  swing_foot_data->AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_foot_data->AddStateAndPointToTrack(right_stance_state, "toe_left");
  com_data = std::make_unique<ComTrackingData>(
      "com_data", osc_walking_gains.K_p_swing_foot,
      osc_walking_gains.K_d_swing_foot, osc_walking_gains.W_swing_foot, plant,
      plant);
  com_data->AddFiniteStateToTrack(left_stance_state);
  com_data->AddFiniteStateToTrack(right_stance_state);
  swing_ft_traj_local_ = std::make_unique<RelativeTranslationTrackingData>(
      "swing_ft_traj", osc_walking_gains.K_p_swing_foot,
      osc_walking_gains.K_d_swing_foot, osc_walking_gains.W_swing_foot, plant,
      plant, swing_foot_data.get(), com_data.get());
  swing_ft_traj_local_->SetViewFrame(view_frame_);

  swing_ft_traj_global_ = std::make_unique<TransTaskSpaceTrackingData>(
      "swing_ft_traj", osc_walking_gains.K_p_swing_foot,
      osc_walking_gains.K_d_swing_foot, osc_walking_gains.W_swing_foot, plant,
      plant);
  swing_ft_traj_global_->AddStateAndPointToTrack(left_stance_state, "toe_right");
  swing_ft_traj_global_->AddStateAndPointToTrack(right_stance_state, "toe_left");
  swing_ft_traj_local_->SetTimeVaryingPDGainMultiplier(
      swing_ft_gain_multiplier_gain_multiplier);
  swing_ft_traj_local_->SetTimerVaryingFeedForwardAccelMultiplier(
      swing_ft_accel_gain_multiplier_gain_multiplier);
  osc->AddTrackingData(std::move(swing_ft_traj_local_));
  bool use_pelvis_for_lipm_tracking = true;

  pelvis_traj_ = std::make_unique<TransTaskSpaceTrackingData>(
      "lipm_traj", osc_walking_gains.K_p_com, osc_walking_gains.K_d_com,
      osc_walking_gains.W_com, plant, plant);
  pelvis_traj_->AddPointToTrack("pelvis");
  osc->AddTrackingData(std::move(pelvis_traj_));

  pelvis_balance_traj_ = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_balance_traj_", osc_walking_gains.K_p_pelvis_balance,
      osc_walking_gains.K_d_pelvis_balance, osc_walking_gains.W_pelvis_balance,
      plant, plant);
  pelvis_balance_traj_->AddFrameToTrack("pelvis");
  osc->AddTrackingData(std::move(pelvis_balance_traj_));
  // Pelvis rotation tracking (yaw)
  pelvis_heading_traj_ = std::make_unique<RotTaskSpaceTrackingData>(
      "pelvis_heading_traj_", osc_walking_gains.K_p_pelvis_heading,
      osc_walking_gains.K_d_pelvis_heading, osc_walking_gains.W_pelvis_heading,
      plant, plant);
  pelvis_heading_traj_->AddFrameToTrack("pelvis");
  osc->AddTrackingData(std::move(pelvis_heading_traj_),
                       osc_walking_gains.period_of_no_heading_control);

  swing_toe_traj_left_ = std::make_unique<JointSpaceTrackingData>(
      "left_toe_angle_traj", osc_walking_gains.K_p_swing_toe,
      osc_walking_gains.K_d_swing_toe, osc_walking_gains.W_swing_toe, plant,
      plant);
  swing_toe_traj_right_ = std::make_unique<JointSpaceTrackingData>(
      "right_toe_angle_traj", osc_walking_gains.K_p_swing_toe,
      osc_walking_gains.K_d_swing_toe, osc_walking_gains.W_swing_toe, plant,
      plant);
  swing_toe_traj_left_->AddStateAndJointToTrack(right_stance_state, "toe_left",
                                                "toe_leftdot");
  swing_toe_traj_right_->AddStateAndJointToTrack(left_stance_state, "toe_right",
                                                 "toe_rightdot");
  osc->AddTrackingData(std::move(swing_toe_traj_left_));
  osc->AddTrackingData(std::move(swing_toe_traj_right_));

  swing_hip_yaw_traj_ = std::make_unique<JointSpaceTrackingData>(
      "swing_hip_yaw_traj_", osc_walking_gains.K_p_hip_yaw,
      osc_walking_gains.K_d_hip_yaw, osc_walking_gains.W_hip_yaw, plant, plant);
  swing_hip_yaw_traj_->AddStateAndJointToTrack(
      left_stance_state, "hip_yaw_right", "hip_yaw_rightdot");
  swing_hip_yaw_traj_->AddStateAndJointToTrack(
      right_stance_state, "hip_yaw_left", "hip_yaw_leftdot");
  osc->AddConstTrackingData(std::move(swing_hip_yaw_traj_), VectorXd::Zero(1));

  /**** OSC settings ****/

  // Build OSC problem
  // Set double support duration for force blending
  osc->SetUpDoubleSupportPhaseBlending(
      double_support_duration, left_stance_state, right_stance_state,
      {post_left_double_support_state, post_right_double_support_state});

  osc->SetOsqpSolverOptionsFromYaml(osqp_settings_filename);

  // Build OSC problem
  osc->Build();
  std::cout << "Built OSC" << std::endl;

  /*****Connect ports*****/

  // OSC connections
  //  builder.Connect(cassie_out_receiver->get_output_port(),
  //                  high_level_command->get_cassie_out_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  high_level_command->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  head_traj_gen->get_state_input_port());
  builder.Connect(high_level_command->get_output_port_yaw(),
                  head_traj_gen->get_yaw_input_port());
  builder.Connect(state_receiver->get_output_port(0),
                  fsm->get_state_input_port());
  builder.Connect(fsm->get_output_port(0),
                  liftoff_event_time->get_input_port_fsm());
  builder.Connect(state_receiver->get_output_port(0),
                  liftoff_event_time->get_input_port_state());
  builder.Connect(fsm->get_output_port(0),
                  touchdown_event_time->get_input_port_fsm());
  builder.Connect(state_receiver->get_output_port(0),
                  touchdown_event_time->get_input_port_state());
  builder.Connect(fsm->get_output_port(0),
                  lipm_traj_generator->get_input_port_fsm());
  builder.Connect(touchdown_event_time->get_output_port_event_time(),
                  lipm_traj_generator->get_input_port_touchdown_time());
  builder.Connect(state_receiver->get_output_port(0),
                  lipm_traj_generator->get_input_port_state());
  builder.Connect(fsm->get_output_port(0),
                  pelvis_traj_generator->get_input_port_fsm());
  builder.Connect(touchdown_event_time->get_output_port_event_time(),
                  pelvis_traj_generator->get_input_port_touchdown_time());
  builder.Connect(state_receiver->get_output_port(0),
                  pelvis_traj_generator->get_input_port_state());
  builder.Connect(fsm->get_output_port(0),
                  swing_ft_traj_generator->get_input_port_fsm());
  builder.Connect(liftoff_event_time->get_output_port_event_time_of_interest(),
                  swing_ft_traj_generator->get_input_port_fsm_switch_time());
  builder.Connect(state_receiver->get_output_port(0),
                  swing_ft_traj_generator->get_input_port_state());
  builder.Connect(alip_traj_generator->get_output_port_alip_state(),
                  swing_ft_traj_generator->get_input_port_alip_state());
  builder.Connect(high_level_command->get_output_port_xy(),
                  swing_ft_traj_generator->get_input_port_vdes());
  builder.Connect(state_receiver->get_output_port(0),
                  left_toe_angle_traj_gen->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  right_toe_angle_traj_gen->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  builder.Connect(fsm->get_output_port(0), osc->get_input_port_fsm());
  builder.Connect(pelvis_traj_generator->get_output_port_lipm_from_touchdown(),
                  osc->get_input_port_tracking_data("lipm_traj"));
  builder.Connect(swing_ft_traj_generator->get_output_port(0),
                  osc->get_input_port_tracking_data("swing_ft_traj"));
  builder.Connect(head_traj_gen->get_output_port(0),
                  osc->get_input_port_tracking_data("pelvis_heading_traj_"));
  builder.Connect(head_traj_gen->get_output_port(0),
                  osc->get_input_port_tracking_data("pelvis_balance_traj_"));
  builder.Connect(left_toe_angle_traj_gen->get_output_port(0),
                  osc->get_input_port_tracking_data("left_toe_angle_traj"));
  builder.Connect(right_toe_angle_traj_gen->get_output_port(0),
                  osc->get_input_port_tracking_data("right_toe_angle_traj"));
  builder.Connect(osc->get_output_port(0), command_sender->get_input_port(0));
  builder.Connect(radio_parser->get_output_port(),
                  high_level_command->get_input_port_radio());

  // Publisher connections
  builder.ExportInput(state_receiver->get_input_port(), "x, u, t");
  builder.ExportInput(radio_parser->get_input_port(), "raw_radio");
  builder.ExportOutput(command_sender->get_output_port(), "lcmt_robot_input");
  builder.ExportOutput(osc->get_output_port_osc_command(), "u, t");

  //  builder.ExportOutput(failure_aggregator->get_status_output_port(),
  //  "failure_status");

  builder.BuildInto(this);
  this->set_name(("osc_walking_controller_diagram"));
  DrawAndSaveDiagramGraph(*this);
  std::cout << "Built controller" << std::endl;
}

}  // namespace controllers
}  // namespace examples
}  // namespace dairlib
