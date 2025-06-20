
#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/sampling_c3/parameter_headers/franka_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/franka_lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/franka_sim_params.h"
#include "systems/controllers/sampling_params.h"
#include "examples/sampling_c3/parameter_headers/trajectory_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "systems/sender_systems/c3_state_sender.h"
#include "control_target_generator.h"
#include "systems/franka_kinematics.h"
#include "systems/sender_systems/sample_buffer_sender.h"
#include "multibody/multibody_utils.h"
#include "solvers/lcs_factory.h"
#include "systems/controllers/sampling_based_c3_controller.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/c3_output_systems.h"

namespace dairlib {

using dairlib::solvers::LCSFactory;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
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
using std::vector;

// TODO: what does gflags mean?
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
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);
  std::string base_path = "examples/sampling_c3/" + FLAGS_demo_name + "/";

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  FrankaC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaC3ControllerParams>(
          base_path + "parameters/franka_c3_controller_params.yaml");
  SamplingC3TrajectoryParams trajectory_params =
      drake::yaml::LoadYamlFile<SamplingC3TrajectoryParams>(
          base_path + "parameters/trajectory_params.yaml");
	// Sim params are only used to keep the offsets between different models 
	// in the scene consistent across all systems.
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(
      base_path + "parameters/franka_sim_params.yaml");
  FrankaLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<FrankaLcmChannels>(FLAGS_lcm_channels);
  SamplingC3Options sampling_c3_options =
      drake::yaml::LoadYamlFile<SamplingC3Options>(
          base_path + "parameters/sampling_c3_options.yaml");

  C3Options c3_options;
  SamplingC3SamplingParams sampling_params;
  c3_options = drake::yaml::LoadYamlFile<C3Options>(
                controller_params.c3_options_file);
  sampling_params = drake::yaml::LoadYamlFile<SamplingC3SamplingParams>(
                controller_params.sampling_params_file);

  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(controller_params.osqp_settings_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  DiagramBuilder<double> plant_builder;

  // Loading the full franka model that will go into franka kinematics system
  // This needs to load the full franka and full end effector model. Connections
  // made around line 184 to FrankaKinematics module.
  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);	
  drake::multibody::ModelInstanceIndex franka_index =
      parser_franka.AddModelsFromUrl(controller_params.franka_model)[0];
  drake::multibody::ModelInstanceIndex ground_index =
      parser_franka.AddModels(FindResourceOrThrow(controller_params.ground_model))[0];
  drake::multibody::ModelInstanceIndex platform_index =
      parser_franka.AddModels(FindResourceOrThrow(controller_params.platform_model))[0];
  drake::multibody::ModelInstanceIndex end_effector_index =
      parser_franka.AddModels(FindResourceOrThrow(controller_params.end_effector_model))[0];
			
  // All the urdfs have their origins at the world frame origin. We define all 
  // the offsets by welding the frames such that changing the offsets in 
  // the param file moves them to where we want in the world frame.
  RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(
        drake::math::RollPitchYaw<double>(3.1415, 0, 0)),
        sim_params.tool_attachment_frame);
  RigidTransform<double> X_F_P =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             sim_params.p_franka_to_platform);
  RigidTransform<double> X_F_G_franka =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             sim_params.p_franka_to_ground);

  // Create a rigid transform from the world frame to the panda_link0 frame.
  // Franka base is 2.45cm above the ground.
  RigidTransform<double> X_F_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), sim_params.p_world_to_franka);

  plant_franka.WeldFrames(plant_franka.world_frame(), 
                   plant_franka.GetFrameByName("panda_link0"), X_F_W);
  plant_franka.WeldFrames(plant_franka.GetFrameByName("panda_link7"), 
                   plant_franka.GetFrameByName("end_effector_base"), T_EE_W);
  plant_franka.WeldFrames(plant_franka.GetFrameByName("panda_link0"),
                   plant_franka.GetFrameByName("ground"), X_F_G_franka);
  plant_franka.WeldFrames(plant_franka.GetFrameByName("panda_link0"),
                   plant_franka.GetFrameByName("platform"), X_F_P);

  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();

  /// adding the jack model (TODO: Change to object instead of jack)
  MultibodyPlant<double> plant_jack(0.0);
  Parser parser_jack(&plant_jack, nullptr);
  parser_jack.AddModels(controller_params.jack_model);
  plant_jack.Finalize();
  auto jack_context = plant_jack.CreateDefaultContext();

  /// Creating the plant for lcs which will contain only end effector and jack
  auto [plant_for_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_builder, 0.0);

  Parser parser_for_lcs(&plant_for_lcs);
  parser_for_lcs.SetAutoRenaming(true);
  /// Loading simple model of end effector (just a sphere) for the lcs plant
  drake::multibody::ModelInstanceIndex franka_lcs_index =
  parser_for_lcs.AddModels(controller_params.end_effector_simple_model)[0];

   drake::multibody::ModelInstanceIndex object_lcs_index =
  parser_for_lcs.AddModels(controller_params.jack_model)[0];

  parser_for_lcs.AddModels(controller_params.ground_model);
	
  // TO DO: The base link may change to the simple end effector model link name
  // or might just be removed entirely.
	RigidTransform<double> X_WI = RigidTransform<double>::Identity();
    RigidTransform<double> ee_Z = RigidTransform<double>::Identity();
    ee_Z.set_translation(Eigen::Matrix<double,3,1>(0, 0, sampling_params.sampling_height));

  Eigen::Vector3d p_world_to_ground = sim_params.p_world_to_franka + 
                                      sim_params.p_franka_to_ground;
  RigidTransform<double> X_W_G =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             p_world_to_ground);
  plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                           plant_for_lcs.GetFrameByName("base_link"), X_WI);
  plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                           plant_for_lcs.GetFrameByName("ground"),
                           X_W_G);

  if (sampling_c3_options.with_z) {
    plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                         plant_for_lcs.GetFrameByName("base_link"), X_WI);
  } else {
    plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                          plant_for_lcs.GetFrameByName("base_link"),ee_Z);
  }



  plant_for_lcs.Finalize();
  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_for_lcs_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_for_lcs);

  auto plant_diagram = plant_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      plant_diagram->CreateDefaultContext();
  auto& plant_for_lcs_context = plant_diagram->GetMutableSubsystemContext(
      plant_for_lcs, diagram_context.get());
  auto plant_for_lcs_context_ad = plant_for_lcs_autodiff->CreateDefaultContext();

  std::vector<std::vector<SortedPair<GeometryId>>>
  contact_pairs;  // Exact list depends on sampling_c3_options.num_contacts_index,
                  // but e.g. num_contacts_index = 0 means this will have
                  // [[(ee,cap1), (ee,cap2), (ee_cap3)],
                  //  [(ground,cap1sph1), (ground,cap1sph2),
                  //   (ground,cap2sph1), (ground,cap2sph2),
                  //   (ground,cap3sph1), (ground,cap3sph2)]]
                  // and the first list (of 3) will get resolved to a single
                  // ee-jack contact, and the second list (of 6) will get
                  // resolved to 3 ground-jack contacts.

  //   Creating a map of contact geoms
  std::unordered_map<std::string, drake::geometry::GeometryId> contact_geoms;

//   Change contact geoms based on the demo_name
if(FLAGS_demo_name == "push_t") {

    drake::geometry::GeometryId ee_contact_points =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("end_effector_simple"))[0];
    drake::geometry::GeometryId horizontal_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("horizontal_link"))[0];
    drake::geometry::GeometryId vertical_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("vertical_link"))[0];

    drake::geometry::GeometryId corner_nxynz_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("corner_nxynz"))[0];
    drake::geometry::GeometryId corner_nxnynz_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("corner_nxnynz"))[0];
    drake::geometry::GeometryId corner_xynz_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("corner_xynz"))[0];

    drake::geometry::GeometryId ground_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("ground"))[0];

    //   Creating a map of contact geoms
    contact_geoms["EE"] = ee_contact_points;
    contact_geoms["horizontal_link"] = horizontal_geoms;
    contact_geoms["vertical_link"] = vertical_geoms;
    contact_geoms["corner_nxynz"] = corner_nxynz_geoms;
    contact_geoms["corner_nxnynz"] = corner_nxnynz_geoms;
    contact_geoms["corner_xynz"] = corner_xynz_geoms;
    contact_geoms["GROUND"] = ground_geoms;

    std::vector<SortedPair<GeometryId>> ee_contact_pairs;

    //   Creating a list of contact pairs for the end effector and the jack to
    //   hand over to lcs factory in the controller to resolve
    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["horizontal_link"]));
    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["vertical_link"]));

    //   Creating a list of contact pairs for the jack and the ground
        SortedPair<GeometryId> ground_contact_1{
        SortedPair(contact_geoms["corner_nxynz"], contact_geoms["GROUND"])};
        SortedPair<GeometryId> ground_contact_2{
        SortedPair(contact_geoms["corner_nxnynz"], contact_geoms["GROUND"])};
        SortedPair<GeometryId> ground_contact_3{
        SortedPair(contact_geoms["corner_xynz"], contact_geoms["GROUND"])};

    contact_pairs.push_back(ee_contact_pairs);

    if(sampling_c3_options.num_contacts_index == 2 || sampling_c3_options.num_contacts_index == 3){
        // If num_contacts_index is 2 or 3, we add an additional contact pair 
        // between the end effector and the ground.
        std::vector<SortedPair<GeometryId>> ee_ground_contact{
        SortedPair(contact_geoms["EE"], contact_geoms["GROUND"])};
        contact_pairs.push_back(ee_ground_contact);
    }
    std::vector<SortedPair<GeometryId>> ground_object_contact_pairs;
    ground_object_contact_pairs.push_back(ground_contact_1);
    ground_object_contact_pairs.push_back(ground_contact_2);
    ground_object_contact_pairs.push_back(ground_contact_3);
    contact_pairs.push_back(ground_object_contact_pairs);
}
else if(FLAGS_demo_name == "box_topple") {
    drake::geometry::GeometryId ee_contact_points =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("end_effector_simple"))[0];
    drake::geometry::GeometryId box_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("box"))[0];

    drake::geometry::GeometryId corner_xynz_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("corner_xynz"))[0];
    drake::geometry::GeometryId corner_xnynz_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("corner_xnynz"))[0];
    drake::geometry::GeometryId corner_nxynz_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("corner_nxynz"))[0];
    drake::geometry::GeometryId corner_nxnynz_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("corner_nxnynz"))[0];
    drake::geometry::GeometryId corner_xyz_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("corner_xyz"))[0];
    drake::geometry::GeometryId corner_xnyz_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("corner_xnyz"))[0];
    drake::geometry::GeometryId corner_nxyz_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("corner_nxyz"))[0];
    drake::geometry::GeometryId corner_nxnyz_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("corner_nxnyz"))[0];

    drake::geometry::GeometryId ground_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("ground"))[0];

    //   Creating a map of contact geoms
    // std::unordered_map<std::string, drake::geometry::GeometryId> contact_geoms;
    contact_geoms["EE"] = ee_contact_points;
    contact_geoms["box"] = box_geoms;
    contact_geoms["corner_xynz"] = corner_xynz_geoms;
    contact_geoms["corner_xnynz"] = corner_xnynz_geoms;
    contact_geoms["corner_nxynz"] = corner_nxynz_geoms;
    contact_geoms["corner_nxnynz"] = corner_nxnynz_geoms;
    contact_geoms["corner_xyz"] = corner_xyz_geoms;
    contact_geoms["corner_xnyz"] = corner_xnyz_geoms;
    contact_geoms["corner_nxyz"] = corner_nxyz_geoms;
    contact_geoms["corner_nxnyz"] = corner_nxnyz_geoms;
    contact_geoms["GROUND"] = ground_geoms;

    std::vector<SortedPair<GeometryId>> ee_contact_pairs;

    //   Creating a list of contact pairs for the end effector and the jack to
    //   hand over to lcs factory in the controller to resolve
    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["box"]));

    //   Creating a list of contact pairs for the jack and the ground
        SortedPair<GeometryId> ground_contact_1{
        SortedPair(contact_geoms["corner_xynz"], contact_geoms["GROUND"])};
        SortedPair<GeometryId> ground_contact_2{
        SortedPair(contact_geoms["corner_xnynz"], contact_geoms["GROUND"])};
        SortedPair<GeometryId> ground_contact_3{
        SortedPair(contact_geoms["corner_nxynz"], contact_geoms["GROUND"])};
        SortedPair<GeometryId> ground_contact_4{
        SortedPair(contact_geoms["corner_nxnynz"], contact_geoms["GROUND"])};
        SortedPair<GeometryId> ground_contact_5{
        SortedPair(contact_geoms["corner_xyz"], contact_geoms["GROUND"])};
        SortedPair<GeometryId> ground_contact_6{
        SortedPair(contact_geoms["corner_xnyz"], contact_geoms["GROUND"])};
        SortedPair<GeometryId> ground_contact_7{
        SortedPair(contact_geoms["corner_nxyz"], contact_geoms["GROUND"])};
        SortedPair<GeometryId> ground_contact_8{
        SortedPair(contact_geoms["corner_nxnyz"], contact_geoms["GROUND"])};

    // std::vector<std::vector<SortedPair<GeometryId>>>
    //     contact_pairs;  // Exact list depends on c3_options.num_contacts_index,
                        // but e.g. num_contacts_index = 0 means this will have
                        // [[(ee,cap1), (ee,cap2), (ee_cap3)],
                        //  [(ground,cap1sph1), (ground,cap1sph2),
                        //   (ground,cap2sph1), (ground,cap2sph2),
                        //   (ground,cap3sph1), (ground,cap3sph2)]]
                        // and the first list (of 3) will get resolved to a single
                        // ee-jack contact, and the second list (of 6) will get
                        // resolved to 3 ground-jack contacts.
    contact_pairs.push_back(ee_contact_pairs);

    if(sampling_c3_options.num_contacts_index == 2 || sampling_c3_options.num_contacts_index == 3){
        // If num_contacts_index is 2 or 3, we add an additional contact pair 
        // between the end effector and the ground.
        std::vector<SortedPair<GeometryId>> ee_ground_contact{
        SortedPair(contact_geoms["EE"], contact_geoms["GROUND"])};
        contact_pairs.push_back(ee_ground_contact);
    }
    std::vector<SortedPair<GeometryId>> ground_object_contact_pairs;
    ground_object_contact_pairs.push_back(ground_contact_1);
    ground_object_contact_pairs.push_back(ground_contact_2);
    ground_object_contact_pairs.push_back(ground_contact_3);
    ground_object_contact_pairs.push_back(ground_contact_4);
    ground_object_contact_pairs.push_back(ground_contact_5);
    ground_object_contact_pairs.push_back(ground_contact_6);
    ground_object_contact_pairs.push_back(ground_contact_7);
    ground_object_contact_pairs.push_back(ground_contact_8);
    contact_pairs.push_back(ground_object_contact_pairs);
}
else if(FLAGS_demo_name == "jacktoy") {
        drake::geometry::GeometryId ee_contact_points =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("end_effector_simple"))[0];
    drake::geometry::GeometryId capsule1_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("capsule_1"))[0];
    drake::geometry::GeometryId capsule2_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("capsule_2"))[0];
    drake::geometry::GeometryId capsule3_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("capsule_3"))[0];

    drake::geometry::GeometryId capsule1_sphere1_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("capsule_1"))[1];
    drake::geometry::GeometryId capsule1_sphere2_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("capsule_1"))[2];
    drake::geometry::GeometryId capsule2_sphere1_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("capsule_2"))[1];
    drake::geometry::GeometryId capsule2_sphere2_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("capsule_2"))[2];
    drake::geometry::GeometryId capsule3_sphere1_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("capsule_3"))[1];
    drake::geometry::GeometryId capsule3_sphere2_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("capsule_3"))[2];

    drake::geometry::GeometryId ground_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("ground"))[0];

    //   Creating a map of contact geoms
    // std::unordered_map<std::string, drake::geometry::GeometryId> contact_geoms;
    contact_geoms["EE"] = ee_contact_points;
    contact_geoms["CAPSULE_1"] = capsule1_geoms;
    contact_geoms["CAPSULE_2"] = capsule2_geoms;
    contact_geoms["CAPSULE_3"] = capsule3_geoms;
    contact_geoms["CAPSULE_1_SPHERE_1"] = capsule1_sphere1_geoms;
    contact_geoms["CAPSULE_1_SPHERE_2"] = capsule1_sphere2_geoms;
    contact_geoms["CAPSULE_2_SPHERE_1"] = capsule2_sphere1_geoms;
    contact_geoms["CAPSULE_2_SPHERE_2"] = capsule2_sphere2_geoms;
    contact_geoms["CAPSULE_3_SPHERE_1"] = capsule3_sphere1_geoms;
    contact_geoms["CAPSULE_3_SPHERE_2"] = capsule3_sphere2_geoms;
    contact_geoms["GROUND"] = ground_geoms;

    std::vector<SortedPair<GeometryId>> ee_contact_pairs;

    //   Creating a list of contact pairs for the end effector and the jack to
    //   hand over to lcs factory in the controller to resolve
    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_1"]));
    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_2"]));
    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_3"]));

    //   Creating a list of contact pairs for the jack and the ground
    SortedPair<GeometryId> ground_contact_1_1{
        SortedPair(contact_geoms["CAPSULE_1_SPHERE_1"], contact_geoms["GROUND"])};
    SortedPair<GeometryId> ground_contact_1_2{
        SortedPair(contact_geoms["CAPSULE_1_SPHERE_2"], contact_geoms["GROUND"])};
    SortedPair<GeometryId> ground_contact_2_1{
        SortedPair(contact_geoms["CAPSULE_2_SPHERE_1"], contact_geoms["GROUND"])};
    SortedPair<GeometryId> ground_contact_2_2{
        SortedPair(contact_geoms["CAPSULE_2_SPHERE_2"], contact_geoms["GROUND"])};
    SortedPair<GeometryId> ground_contact_3_1{
        SortedPair(contact_geoms["CAPSULE_3_SPHERE_1"], contact_geoms["GROUND"])};
    SortedPair<GeometryId> ground_contact_3_2{
        SortedPair(contact_geoms["CAPSULE_3_SPHERE_2"], contact_geoms["GROUND"])};

    // std::vector<std::vector<SortedPair<GeometryId>>>
    //     contact_pairs;  // Exact list depends on c3_options.num_contacts_index,
                        // but e.g. num_contacts_index = 0 means this will have
                        // [[(ee,cap1), (ee,cap2), (ee_cap3)],
                        //  [(ground,cap1sph1), (ground,cap1sph2),
                        //   (ground,cap2sph1), (ground,cap2sph2),
                        //   (ground,cap3sph1), (ground,cap3sph2)]]
                        // and the first list (of 3) will get resolved to a single
                        // ee-jack contact, and the second list (of 6) will get
                        // resolved to 3 ground-jack contacts.
    contact_pairs.push_back(ee_contact_pairs);

    if(sampling_c3_options.num_contacts_index == 2 || sampling_c3_options.num_contacts_index == 3){
    // If num_contacts_index is 2 or 3, we add an additional contact pair 
    // between the end effector and the ground.
    std::vector<SortedPair<GeometryId>> ee_ground_contact{
        SortedPair(contact_geoms["EE"], contact_geoms["GROUND"])};
    contact_pairs.push_back(ee_ground_contact);
    }
    std::vector<SortedPair<GeometryId>> ground_object_contact_pairs;
    ground_object_contact_pairs.push_back(ground_contact_1_1);
    ground_object_contact_pairs.push_back(ground_contact_1_2);
    ground_object_contact_pairs.push_back(ground_contact_2_1);
    ground_object_contact_pairs.push_back(ground_contact_2_2);
    ground_object_contact_pairs.push_back(ground_contact_3_1);
    ground_object_contact_pairs.push_back(ground_contact_3_2);
    contact_pairs.push_back(ground_object_contact_pairs);
}
else if(FLAGS_demo_name == "ball_rolling"){
    drake::geometry::GeometryId ee_contact_points =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("end_effector_simple"))[0];
    drake::geometry::GeometryId object_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("sphere"))[0];
    drake::geometry::GeometryId ground_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("ground"))[0];

    //   Creating a map of contact geoms
    contact_geoms["EE"] = ee_contact_points;
    contact_geoms["SPHERE"] = object_geoms;
    contact_geoms["GROUND"] = ground_geoms;

    std::vector<SortedPair<GeometryId>> ee_contact_pairs;

    //   Creating a list of contact pairs for the end effector and the jack to
    //   hand over to lcs factory in the controller to resolve
    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["SPHERE"]));
    //   Creating a list of contact pairs for the jack and the ground
    std::vector<SortedPair<GeometryId>> ground_contact{
        SortedPair(contact_geoms["SPHERE"], contact_geoms["GROUND"])};

    // will have [[(ee,sphere)], [(sphere, ground)]]
    contact_pairs.push_back(ee_contact_pairs);

    if (sampling_c3_options.num_contacts_index == 1){
        // If num_contacts_index is 2 or 3, we add an additional contact pair 
        // between the end effector and the ground.
        std::vector<SortedPair<GeometryId>> ee_ground_contact{
        SortedPair(contact_geoms["EE"], contact_geoms["GROUND"])};
        contact_pairs.push_back(ee_ground_contact);
    }
    contact_pairs.push_back(ground_contact);
}

  else if (FLAGS_demo_name == "push_anything") {

    drake::geometry::GeometryId ee_contact_points =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("end_effector_simple"))[0];
    drake::geometry::GeometryId horizontal_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("body"))[0];


    drake::geometry::GeometryId corner_nxynz_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("body"))[1];
    drake::geometry::GeometryId corner_nxnynz_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("body"))[2];
    drake::geometry::GeometryId corner_xynz_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("body"))[3];

    drake::geometry::GeometryId ground_geoms =
        plant_for_lcs.GetCollisionGeometriesForBody(
            plant_for_lcs.GetBodyByName("ground"))[0];

    //   Creating a map of contact geoms
    contact_geoms["EE"] = ee_contact_points;
    contact_geoms["horizontal_link"] = horizontal_geoms;
    contact_geoms["corner_nxynz"] = corner_nxynz_geoms;
    contact_geoms["corner_nxnynz"] = corner_nxnynz_geoms;
    contact_geoms["corner_xynz"] = corner_xynz_geoms;
    contact_geoms["GROUND"] = ground_geoms;

    std::vector<SortedPair<GeometryId>> ee_contact_pairs;


    //   Creating a list of contact pairs for the end effector and the jack to
    //   hand over to lcs factory in the controller to resolve
    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], contact_geoms["horizontal_link"]));

    //   Creating a list of contact pairs for the jack and the ground
    SortedPair<GeometryId> ground_contact_1{
      SortedPair(contact_geoms["corner_nxynz"], contact_geoms["GROUND"])};
    SortedPair<GeometryId> ground_contact_2{
      SortedPair(contact_geoms["corner_nxnynz"], contact_geoms["GROUND"])};
    SortedPair<GeometryId> ground_contact_3{
      SortedPair(contact_geoms["corner_xynz"], contact_geoms["GROUND"])};

    contact_pairs.push_back(ee_contact_pairs);

    if(sampling_c3_options.num_contacts_index == 2 || sampling_c3_options.num_contacts_index == 3){
      // If num_contacts_index is 2 or 3, we add an additional contact pair
      // between the end effector and the ground.
      std::vector<SortedPair<GeometryId>> ee_ground_contact{
        SortedPair(contact_geoms["EE"], contact_geoms["GROUND"])};
      contact_pairs.push_back(ee_ground_contact);
    }
    std::vector<SortedPair<GeometryId>> ground_object_contact_pairs;
    ground_object_contact_pairs.push_back(ground_contact_1);
    ground_object_contact_pairs.push_back(ground_contact_2);
    ground_object_contact_pairs.push_back(ground_contact_3);
    contact_pairs.push_back(ground_object_contact_pairs);

  }

  DiagramBuilder<double> builder;

  auto object_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, &lcm));
  auto franka_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);
  auto object_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(plant_jack);
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));

  // System that takes in the state of the franka and jack and outputs a
  // reduced order lcs state vector.
  auto reduced_order_model_receiver =
      builder.AddSystem<systems::FrankaKinematics>(
          plant_franka, franka_context.get(), plant_jack, jack_context.get(),
          controller_params.end_effector_name, 
          controller_params.object_body_name,
          controller_params.include_end_effector_orientation);

    // Systems involved in setting up the target for the end effector and the
    // object.
    // Build the appropriate generator at run time
    std::unique_ptr<systems::TargetGenerator> target_generator;

    if (FLAGS_demo_name == "jacktoy") {
    target_generator = std::make_unique<systems::TargetGeneratorJacktoy>(plant_jack);
    } 
    else if (FLAGS_demo_name == "box_topple") {
    target_generator = std::make_unique<systems::TargetGeneratorBoxTopple>(plant_jack);
    } 
    else if (FLAGS_demo_name == "push_t") {
    target_generator = std::make_unique<systems::TargetGeneratorPushT>(plant_jack);
    } 
    else if (FLAGS_demo_name == "ball_rolling") {
    target_generator = std::make_unique<systems::TargetGeneratorBallRolling>(plant_jack);
    }
    else if (FLAGS_demo_name == "push_anything") {
      target_generator = std::make_unique<systems::TargetGeneratorPushT>(plant_jack);
    }

    else {
    throw std::runtime_error("Unknown --demo_name value: " + FLAGS_demo_name);
    }

    // Add to the diagram; the pointer you get back keeps the final static type.
    auto* control_target = builder.AddSystem(std::move(target_generator));

  control_target->SetRemoteControlParameters(
      trajectory_params.trajectory_type,
      trajectory_params.use_changing_final_goal,
      trajectory_params.changing_final_goal_type,
      trajectory_params.prevent_three_topples_for_random_goal_gen,
      trajectory_params.traj_radius,
      trajectory_params.x_c, trajectory_params.y_c,
      trajectory_params.lead_angle,
      trajectory_params.fixed_target_position,
      trajectory_params.fixed_target_orientation,
      trajectory_params.step_size,
      trajectory_params.start_point_x,
      trajectory_params.start_point_y,
      trajectory_params.end_point_x,
      trajectory_params.end_point_y,
      trajectory_params.lookahead_step_size,
      trajectory_params.lookahead_angle,
      trajectory_params.angle_hysteresis,
      trajectory_params.angle_err_to_vel_factor,
      trajectory_params.max_step_size,
      trajectory_params.ee_goal_height,
      trajectory_params.object_half_width,
      trajectory_params.position_success_threshold,
      trajectory_params.orientation_success_threshold,
      trajectory_params.random_goal_x_limits,
      trajectory_params.random_goal_y_limits,
      trajectory_params.random_goal_radius_limits,
      trajectory_params.resting_object_height);
  std::vector<int> input_sizes = {3, 7, 3, 6};
  auto target_state_mux =
      builder.AddSystem<drake::systems::Multiplexer>(input_sizes);
  auto final_target_state_mux =
      builder.AddSystem<drake::systems::Multiplexer>(input_sizes);
  auto end_effector_zero_velocity_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(3));
  auto object_zero_velocity_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(6));
  auto target_gen_info_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.target_generator_info_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));


  builder.Connect(control_target->get_output_port_end_effector_target(),
                  target_state_mux->get_input_port(0));
  builder.Connect(control_target->get_output_port_object_target(),
                  target_state_mux->get_input_port(1));
  builder.Connect(end_effector_zero_velocity_source->get_output_port(),
                  target_state_mux->get_input_port(2));
  builder.Connect(control_target->get_output_port_object_velocity_target(),
                  target_state_mux->get_input_port(3));
  builder.Connect(control_target->get_output_port_target_gen_info(),
                  target_gen_info_publisher->get_input_port());
  builder.Connect(control_target->get_output_port_end_effector_target(),
                  final_target_state_mux->get_input_port(0));
  builder.Connect(control_target->get_output_port_object_final_target(),
                  final_target_state_mux->get_input_port(1));
  builder.Connect(end_effector_zero_velocity_source->get_output_port(),
                  final_target_state_mux->get_input_port(2));
  builder.Connect(object_zero_velocity_source->get_output_port(),
                  final_target_state_mux->get_input_port(3));

  // Instantiating the sampling based c3 controller.
  auto controller = builder.AddSystem<systems::SamplingC3Controller>(
      plant_for_lcs, &plant_for_lcs_context, *plant_for_lcs_autodiff,
      plant_for_lcs_context_ad.get(), contact_pairs, c3_options,
      sampling_c3_options, sampling_params, franka_lcs_index, object_lcs_index);

  // These systems publish the current and best planned trajectories.
  auto actor_trajectory_sender_curr_plan = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_curr_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto object_trajectory_sender_curr_plan = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_object_curr_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto actor_trajectory_sender_best_plan = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_actor_best_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto object_trajectory_sender_best_plan = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_object_best_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  // The following systems consume the planned trajectories and output it to an 
  // LCM publisher for debugging.
  auto c3_output_sender_curr_plan =
      builder.AddNamedSystem("c3_output_sender_curr_plan", 
                             std::make_unique<systems::C3OutputSender>());
  auto c3_output_publisher_curr_plan =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_output>(
          lcm_channel_params.c3_debug_output_curr_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_forces_publisher_curr_plan =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_forces>(
          lcm_channel_params.c3_force_curr_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  auto c3_output_sender_best_plan =
      builder.AddNamedSystem("c3_output_sender_best_plan", 
                             std::make_unique<systems::C3OutputSender>());
  auto c3_output_publisher_best_plan =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_output>(
          lcm_channel_params.c3_debug_output_best_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_forces_publisher_best_plan =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_forces>(
          lcm_channel_params.c3_force_best_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  // These systems publish the tracking output.
  auto actor_c3_execution_trajectory_sender = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.c3_trajectory_exec_actor_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto actor_repos_execution_trajectory_sender = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.repos_trajectory_exec_actor_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto actor_tracking_trajectory_sender = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.tracking_trajectory_actor_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto object_tracking_trajectory_sender = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.tracking_trajectory_object_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  auto sample_buffer_sender = builder.AddSystem<systems::SampleBufferSender>(
    sampling_params.N_sample_buffer, 10);

  // These systems publish the sample locations and sample costs over LCM.
  auto sample_locations_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.sample_locations_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto sample_costs_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.sample_costs_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto curr_and_best_sample_costs_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.curr_and_best_sample_costs_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto controller_debug_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_sampling_c3_debug>(
          lcm_channel_params.sampling_c3_debug_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto is_c3_mode_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.is_c3_mode_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto dynamically_feasible_curr_plan_actor_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.dynamically_feasible_curr_actor_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto dynamically_feasible_curr_plan_object_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.dynamically_feasible_curr_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto dynamically_feasible_best_plan_actor_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.dynamically_feasible_best_actor_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto dynamically_feasible_best_plan_object_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.dynamically_feasible_best_plan_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto sample_buffer_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_sample_buffer>(
          lcm_channel_params.sample_buffer_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  std::vector<std::string> state_names = {
      "end_effector_x",  "end_effector_y", "end_effector_z",  "object_qw",
      "object_qx",       "object_qy",      "object_qz",       "object_x",
      "object_y",        "object_z",       "end_effector_vx", "end_effector_vy",
      "end_effector_vz", "object_wx",      "object_wy",       "object_wz",
      "object_vz",       "object_vz",      "object_vz",
  };
  // This system consumes the desired lcs state and the current lcs state and
  // outputs both.
  auto c3_state_sender =
      builder.AddSystem<systems::C3StateSender>(3 + 7 + 3 + 6, state_names);
  auto c3_target_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_target_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_actual_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_final_target_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_final_target_state_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  controller->SetOsqpSolverOptions(solver_options);
  builder.Connect(franka_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_franka_state());
  builder.Connect(object_state_sub->get_output_port(),
                  object_state_receiver->get_input_port());
  builder.Connect(object_state_receiver->get_output_port(),
                  reduced_order_model_receiver->get_input_port_object_state());
  // CONTROLLER INPUT CONNECTIONS
  builder.Connect(reduced_order_model_receiver->get_output_port(),
                  controller->get_input_port_lcs_state());
  builder.Connect(object_state_receiver->get_output_port(),
                  control_target->get_input_port_object_state());
  // control_target output connections to mux set up earlier.
  builder.Connect(target_state_mux->get_output_port(),
                  controller->get_input_port_target());
  builder.Connect(final_target_state_mux->get_output_port(),
                  controller->get_input_port_final_target());
  builder.Connect(radio_sub->get_output_port(),
                  controller->get_input_port_radio());

  // Sending xbox output to control target generator.
  builder.Connect(radio_sub->get_output_port(),
                  control_target->get_input_port_radio());

  builder.Connect(
      controller->get_output_port_c3_solution_curr_plan_actor(),
      actor_trajectory_sender_curr_plan->get_input_port());
  builder.Connect(
      controller->get_output_port_c3_solution_curr_plan_object(),
      object_trajectory_sender_curr_plan->get_input_port());

  builder.Connect(
      controller->get_output_port_c3_solution_best_plan_actor(),
      actor_trajectory_sender_best_plan->get_input_port());
  builder.Connect(
      controller->get_output_port_c3_solution_best_plan_object(),
      object_trajectory_sender_best_plan->get_input_port());

  // DEBUGGING OUTPUT CONNECTIONS
  builder.Connect(controller->get_output_port_c3_solution_curr_plan(),
                  c3_output_sender_curr_plan->get_input_port_c3_solution());
  builder.Connect(controller->get_output_port_c3_intermediates_curr_plan(),
                  c3_output_sender_curr_plan->get_input_port_c3_intermediates());
  builder.Connect(controller->get_output_port_lcs_contact_jacobian_curr_plan(),
                  c3_output_sender_curr_plan->get_input_port_lcs_contact_info());
  builder.Connect(c3_output_sender_curr_plan->get_output_port_c3_debug(),
                  c3_output_publisher_curr_plan->get_input_port());
  builder.Connect(c3_output_sender_curr_plan->get_output_port_c3_force(),
                  c3_forces_publisher_curr_plan->get_input_port());
  builder.Connect(controller->get_output_port_c3_solution_best_plan(),
                  c3_output_sender_best_plan->get_input_port_c3_solution());
  builder.Connect(controller->get_output_port_c3_intermediates_best_plan(),
                  c3_output_sender_best_plan->get_input_port_c3_intermediates());
  builder.Connect(controller->get_output_port_lcs_contact_jacobian_best_plan(),
                  c3_output_sender_best_plan->get_input_port_lcs_contact_info());
  builder.Connect(c3_output_sender_best_plan->get_output_port_c3_debug(),
                  c3_output_publisher_best_plan->get_input_port());
  builder.Connect(c3_output_sender_best_plan->get_output_port_c3_force(),
                  c3_forces_publisher_best_plan->get_input_port());

  builder.Connect(controller->get_output_port_dynamically_feasible_curr_plan_actor(),
                  dynamically_feasible_curr_plan_actor_publisher->get_input_port());

  builder.Connect(controller->get_output_port_dynamically_feasible_best_plan_actor(),
                  dynamically_feasible_best_plan_actor_publisher->get_input_port());

  builder.Connect(controller->get_output_port_dynamically_feasible_curr_plan_object(),
                  dynamically_feasible_curr_plan_object_publisher->get_input_port());

  builder.Connect(controller->get_output_port_dynamically_feasible_curr_plan_object(),
                  dynamically_feasible_best_plan_object_publisher->get_input_port());

  // ACTUAL AND TARGET LCS_STATE CONNECTIONS
  builder.Connect(target_state_mux->get_output_port(),
                  c3_state_sender->get_input_port_target_state());
  builder.Connect(final_target_state_mux->get_output_port(),
                  c3_state_sender->get_input_port_final_target_state());
  builder.Connect(reduced_order_model_receiver->get_output_port_lcs_state(),
                  c3_state_sender->get_input_port_actual_state());
  builder.Connect(c3_state_sender->get_output_port_target_c3_state(),
                  c3_target_state_publisher->get_input_port());
  builder.Connect(c3_state_sender->get_output_port_final_target_c3_state(),
                  c3_final_target_state_publisher->get_input_port());
  builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
                  c3_actual_state_publisher->get_input_port());

  // TRACKING TRAJECTORY CONNECTIONS
  builder.Connect(
      controller->get_output_port_c3_traj_execute_actor(),
      actor_c3_execution_trajectory_sender->get_input_port());
  builder.Connect(controller->get_output_port_repos_traj_execute_actor(),
      actor_repos_execution_trajectory_sender
                      ->get_input_port());

  builder.Connect(
      controller->get_output_port_traj_execute_actor(),
      actor_tracking_trajectory_sender->get_input_port());
  builder.Connect(
      controller->get_output_port_traj_execute_object(),
      object_tracking_trajectory_sender->get_input_port());

  // Add connections to sample location sender system, cost sender system
  // and c3_mode_sender system.
  builder.Connect(controller->get_output_port_all_sample_locations(),
                  sample_locations_publisher->get_input_port());
  builder.Connect(controller->get_output_port_all_sample_costs(),
                  sample_costs_publisher->get_input_port());
  builder.Connect(controller->get_output_port_is_c3_mode(),
                  is_c3_mode_publisher->get_input_port());
  builder.Connect(controller->get_output_port_curr_and_best_sample_costs(),
  curr_and_best_sample_costs_publisher->get_input_port());
  builder.Connect(controller->get_output_port_debug(),
                  controller_debug_publisher->get_input_port());
  builder.Connect(sample_buffer_sender->get_output_port_sample_buffer(),
                  sample_buffer_publisher->get_input_port());
  builder.Connect(controller->get_output_port_sample_buffer_configurations(),
                  sample_buffer_sender->get_input_port_samples());
  builder.Connect(controller->get_output_port_sample_buffer_costs(),
                  sample_buffer_sender->get_input_port_sample_costs());

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_c3_controller"));
  plant_diagram->set_name(("franka_c3_plant"));
  DrawAndSaveDiagramGraph(*owned_diagram, "/home/sharanya/workspace/diagrams/" + FLAGS_demo_name + "/franka_c3_controller_diagram");
  DrawAndSaveDiagramGraph(*plant_diagram,"/home/sharanya/workspace/diagrams/" + FLAGS_demo_name + "/franka_c3_plant");

  // Run lcm-driven simulation
  int lcm_buffer_size = 200;
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), franka_state_receiver,
      lcm_channel_params.franka_state_channel, true, lcm_buffer_size);
  DrawAndSaveDiagramGraph(*loop.get_diagram(),"/home/sharanya/workspace/diagrams/" + FLAGS_demo_name + "/loop_franka_c3_controller_diagram");
  //  auto& controller_context = loop.get_diagram()->GetMutableSubsystemContext(
  //      *controller, &loop.get_diagram_mutable_context());
  //  controller->get_input_port_target().FixValue(&controller_context, x_des);
  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return object_state_sub->GetInternalMessageCount() > 1; });
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }