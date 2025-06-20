#include <dairlib/lcmt_robot_input.hpp>
#include <drake/common/yaml/yaml_io.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/lcmt_panda_command.hpp>
#include <drake/lcmt_panda_status.hpp>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <gflags/gflags.h>
#include "drake/common/find_resource.h"

#include "examples/sampling_c3/parameter_headers/franka_drake_lcm_driver_channels.h"
#include "examples/sampling_c3/parameter_headers/franka_lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/franka_sim_params.h"
#include "franka_state_translator.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

using drake::math::RigidTransform;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using dairlib::systems::RobotInputReceiver;
using dairlib::systems::RobotOutputSender;
using dairlib::systems::SubvectorPassThrough;
using dairlib::systems::TimestampedVector;

DEFINE_string(lcm_channels,
              "examples/sampling_c3/shared_parameters/lcm_channels_hardware.yaml",
              "Filepath containing lcm channels");
DEFINE_string(franka_driver_channels,
              "examples/sampling_c3/shared_parameters/franka_drake_lcm_driver_channels.yaml",
              "Filepath containing drake franka driver channels");
// NOTE:  While most module's TTL is set to 0 by default, this one is set to 1
// since it necessarily needs to communicate with the Franka.
DEFINE_string(lcm_url,
              "udpm://239.255.76.67:7667?ttl=1",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo_name,
            "box_topple",
            "Name for the demo, used when building filepaths for output.");

namespace dairlib {

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::string base_path = "examples/sampling_c3/" + FLAGS_demo_name + "/";

  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(FLAGS_lcm_channels);
  FrankaDrakeLcmDriverChannels franka_driver_channel_params =
      drake::yaml::LoadYamlFile<FrankaDrakeLcmDriverChannels>(FLAGS_franka_driver_channels);
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      "examples/sampling_c3/box_topple/parameters/franka_sim_params.yaml");

  DiagramBuilder<double> builder;

  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant);
  parser.AddModelsFromUrl(sim_params.franka_model);

//   parser.AddModelsFromUrl(sim_params.franka_model);
  RigidTransform<double> W_X_R = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), sim_params.p_world_to_franka);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"),
                   W_X_R);
  plant.Finalize();

  auto pos_map = multibody::MakeNameToPositionsMap(plant);
  auto vel_map = multibody::MakeNameToVelocitiesMap(plant);
  auto act_map = multibody::MakeNameToActuatorsMap(plant);

  auto pos_names = multibody::ExtractOrderedNamesFromMap(pos_map);
  auto vel_names = multibody::ExtractOrderedNamesFromMap(vel_map);
  auto act_names = multibody::ExtractOrderedNamesFromMap(act_map);

  /* -------------------------------------------------------------------------------------------*/
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);


  auto franka_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.franka_state_channel, &lcm,
          1.0 / 1000.0));
  auto franka_state_translator = builder.AddSystem<systems::FrankaStateOutTranslator>(
      pos_names, vel_names, act_names);

  builder.Connect(*franka_state_translator, *franka_state_pub);

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_bridge_driver_out"));

  systems::LcmDrivenLoop<drake::lcmt_panda_status> loop(
      &lcm, std::move(owned_diagram), franka_state_translator,
      franka_driver_channel_params.franka_status_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram(), "/home/dair/opt/sampling_based_c3/dairlib/examples/sampling_c3/diagram/" + FLAGS_demo_name + "/franka_bridge_driver_out_diagram");
  loop.Simulate();

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv); }