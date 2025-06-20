
#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/sampling_c3/parameter_headers/franka_lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/franka_osc_controller_params.h"
#include "systems/end_effector_force_trajectory.h"
#include "systems/end_effector_orientation.h"
#include "systems/end_effector_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/gravity_compensator.h"
#include "systems/controllers/osc/external_force_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

using drake::math::RigidTransform;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

using systems::controllers::ExternalForceTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

DEFINE_string(osqp_settings,
              "examples/sampling_c3/shared_parameters/franka_osc_qp_settings.yaml",
              "Filepath containing qp settings");
DEFINE_string(controller_parameters,
              "examples/sampling_c3/box_topple/parameters/franka_osc_controller_params.yaml",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");
DEFINE_string(lcm_channels,
              "examples/sampling_c3/shared_parameters/lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");
DEFINE_string(lcm_url,
              "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo_name,
              "box_topple",
              "Name for the demo, used when building filepaths for output.");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::string base_path = "examples/sampling_c3/" + FLAGS_demo_name + "/";

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  FrankaControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaControllerParams>(base_path + "parameters/franka_osc_controller_params.yaml");

  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(FLAGS_lcm_channels);
  OSCGains gains = drake::yaml::LoadYamlFile<OSCGains>(
      FindResourceOrThrow(base_path + "parameters/franka_osc_controller_params.yaml"), {}, {}, yaml_options);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(FLAGS_osqp_settings))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());
  DiagramBuilder<double> builder;

  drake::multibody::MultibodyPlant<double> plant(0.0);
  Parser parser(&plant, nullptr);

  parser.SetAutoRenaming(true);
  drake::multibody::ModelInstanceIndex franka_index =
      parser.AddModelsFromUrl(controller_params.franka_model)[0];
  drake::multibody::ModelInstanceIndex ground_index =
      parser.AddModels(FindResourceOrThrow(controller_params.ground_model))[0];
  drake::multibody::ModelInstanceIndex platform_index =
      parser.AddModels(FindResourceOrThrow(controller_params.platform_model))[0];
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser.AddModels(FindResourceOrThrow(controller_params.end_effector_model))[0];

  // All the urdfs have their origins at the world frame origin. We define all 
  // the offsets by welding the frames such that changing the offsets in 
  // the param file moves them to where we want in the world frame.
  RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(
        drake::math::RollPitchYaw<double>(3.1415, 0, 0)),
        controller_params.tool_attachment_frame);
  RigidTransform<double> X_F_P =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             controller_params.p_franka_to_platform);
  RigidTransform<double> X_F_G_franka =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             controller_params.p_franka_to_ground);

  // Create a rigid transform from the world frame to the panda_link0 frame.
  // Franka base is 2.45cm above the ground.
  RigidTransform<double> X_F_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), controller_params.p_world_to_franka);

  plant.WeldFrames(plant.world_frame(), 
                   plant.GetFrameByName("panda_link0"), X_F_W);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"), 
                   plant.GetFrameByName("end_effector_base"), T_EE_W);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"),
                   plant.GetFrameByName("ground"), X_F_G_franka);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"),
                   plant.GetFrameByName("platform"), X_F_P);

  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);
  
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto end_effector_trajectory_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.tracking_trajectory_actor_channel, &lcm));
  auto end_effector_position_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>(
          "end_effector_position_target");
  auto end_effector_force_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>(
          "end_effector_force_target");
  auto end_effector_orientation_receiver =
      builder.AddSystem<systems::LcmOrientationTrajectoryReceiver>(
          "end_effector_orientation_target");
  auto franka_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.franka_input_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto osc_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.osc_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto franka_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);
  auto osc_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);
  auto end_effector_trajectory =
      builder.AddSystem<EndEffectorTrajectoryGenerator>(
        plant, plant_context.get(),
        controller_params.neutral_position,
        controller_params.teleop_neutral_position,
        controller_params.end_effector_name);
  VectorXd neutral_position = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      controller_params.neutral_position.data(),
      controller_params.neutral_position.size());
  end_effector_trajectory->SetRemoteControlParameters(
      neutral_position, controller_params.x_scale, controller_params.y_scale,
      controller_params.z_scale);
  auto end_effector_orientation_trajectory =
      builder.AddSystem<EndEffectorOrientationGenerator>(plant,
                                                         plant_context.get());
  end_effector_orientation_trajectory->SetTrackOrientation(
      controller_params.track_end_effector_orientation);
  auto end_effector_force_trajectory =
      builder.AddSystem<EndEffectorForceTrajectoryGenerator>(
          plant, plant_context.get());
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));
  // TODO:  Why are there 5 arguments here? Clean this up at some point.
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), false);
  if (controller_params.publish_debug_info){
    auto osc_debug_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
            lcm_channel_params.osc_debug_channel, &lcm,
            TriggerTypeSet({TriggerType::kForced})));
    builder.Connect(osc->get_output_port_osc_debug(),
                    osc_debug_pub->get_input_port());
  }

  auto end_effector_position_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target", controller_params.K_p_end_effector,
          controller_params.K_d_end_effector, controller_params.W_end_effector,
          plant, plant);
  end_effector_position_tracking_data->AddPointToTrack(
      controller_params.end_effector_name);
  const VectorXd& end_effector_acceleration_limits =
      controller_params.end_effector_acceleration * Vector3d::Ones();
  end_effector_position_tracking_data->SetCmdAccelerationBounds(
      -end_effector_acceleration_limits, end_effector_acceleration_limits);
  auto mid_link_position_tracking_data_for_rel =
      std::make_unique<JointSpaceTrackingData>(
          "panda_joint2_target", controller_params.K_p_mid_link,
          controller_params.K_d_mid_link, controller_params.W_mid_link, plant,
          plant);
  mid_link_position_tracking_data_for_rel->AddJointToTrack("panda_joint2",
                                                           "panda_joint2dot");

  auto end_effector_force_tracking_data =
      std::make_unique<ExternalForceTrackingData>(
          "end_effector_force", controller_params.W_ee_lambda, plant, plant,
          controller_params.end_effector_name, Vector3d::Zero());

  auto end_effector_orientation_tracking_data =
      std::make_unique<RotTaskSpaceTrackingData>(
          "end_effector_orientation_target",
          controller_params.K_p_end_effector_rot,
          controller_params.K_d_end_effector_rot,
          controller_params.W_end_effector_rot, plant, plant);
  end_effector_orientation_tracking_data->AddFrameToTrack(
      controller_params.end_effector_name);
  Eigen::VectorXd orientation_target = Eigen::VectorXd::Zero(4);
  orientation_target(0) = 1;
  osc->AddTrackingData(std::move(end_effector_position_tracking_data));
  // This 1.1 value is trying to track the panda_joint_2 so that we avoid the
  // null space associated with trying to control 7 joints with 6 DOF. 
  // The value is currently set to 1.1 to have it be more vertical for the jack
  // example but not enough that it hits a singularity or ends up with too small 
  // a workspace. 
  osc->AddConstTrackingData(std::move(mid_link_position_tracking_data_for_rel),
                            1.1 * VectorXd::Ones(1));
  osc->AddTrackingData(std::move(end_effector_orientation_tracking_data));
  osc->AddForceTrackingData(std::move(end_effector_force_tracking_data));
  osc->SetAccelerationCostWeights(gains.W_acceleration);
  osc->SetInputCostWeights(gains.W_input_regularization);
  osc->SetInputSmoothingCostWeights(gains.W_input_smoothing_regularization);
  osc->SetAccelerationConstraints(
      controller_params.enforce_acceleration_constraints);

  osc->SetContactFriction(controller_params.mu);
  osc->SetOsqpSolverOptions(solver_options);

  osc->Build();

  if (controller_params.cancel_gravity_compensation) {
    if (FLAGS_lcm_channels == base_path + "shared_parameters/lcm_channels_simulation.yaml"){
      std::cerr << "In simulation, OSC needs to have cancel_gravity_compensation: false" << std::endl;
      return -1;
    }
    auto gravity_compensator =
        builder.AddSystem<systems::GravityCompensationRemover>(plant,
                                                               *plant_context);
    builder.Connect(osc->get_output_port_osc_command(),
                    gravity_compensator->get_input_port());
    builder.Connect(gravity_compensator->get_output_port(),
                    franka_command_sender->get_input_port());
  } else {
    if (FLAGS_lcm_channels == base_path + "shared_parameters/lcm_channels_hardware.yaml"){
      std::cerr << "In hardware, OSC needs to have cancel_gravity_compensation: true"<< std::endl;
      return -1;
    }
    builder.Connect(osc->get_output_port_osc_command(),
                    franka_command_sender->get_input_port(0));
  }

  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_trajectory->get_input_port_radio());
  builder.Connect(state_receiver->get_output_port(0),
                  end_effector_orientation_trajectory->get_input_port_state());
  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_orientation_trajectory->get_input_port_radio());
  builder.Connect(state_receiver->get_output_port(0),
                  end_effector_force_trajectory->get_input_port_state());
  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_force_trajectory->get_input_port_radio());
  builder.Connect(franka_command_sender->get_output_port(),
                  franka_command_pub->get_input_port());
  builder.Connect(osc_command_sender->get_output_port(),
                  osc_command_pub->get_input_port());
  builder.Connect(osc->get_output_port_osc_command(),
                  osc_command_sender->get_input_port(0));

  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  builder.Connect(end_effector_trajectory_sub->get_output_port(),
                  end_effector_position_receiver->get_input_port_trajectory());
  builder.Connect(end_effector_trajectory_sub->get_output_port(),
                  end_effector_force_receiver->get_input_port_trajectory());
  builder.Connect(
      end_effector_trajectory_sub->get_output_port(),
      end_effector_orientation_receiver->get_input_port_trajectory());
  builder.Connect(end_effector_position_receiver->get_output_port(0),
                  end_effector_trajectory->get_input_port_trajectory());
  builder.Connect(state_receiver->get_output_port(0),
                  end_effector_trajectory->get_input_port_state());
  builder.Connect(
      end_effector_orientation_receiver->get_output_port(0),
      end_effector_orientation_trajectory->get_input_port_trajectory());
  builder.Connect(end_effector_trajectory->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_target"));
  builder.Connect(
      end_effector_orientation_trajectory->get_output_port(0),
      osc->get_input_port_tracking_data("end_effector_orientation_target"));
  builder.Connect(end_effector_force_receiver->get_output_port(0),
                  end_effector_force_trajectory->get_input_port_trajectory());
  builder.Connect(end_effector_force_trajectory->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_force"));

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_osc_controller"));
  DrawAndSaveDiagramGraph(*owned_diagram, "/home/dair/opt/sampling_based_c3/dairlib/examples/sampling_c3/diagram/" + FLAGS_demo_name + "/franka_osc_controller_diagram");
  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), state_receiver,
      lcm_channel_params.franka_state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram(), "/home/dair/opt/sampling_based_c3/dairlib/examples/sampling_c3/diagram/" + FLAGS_demo_name + "/loop_franka_osc_controller_diagram");
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }