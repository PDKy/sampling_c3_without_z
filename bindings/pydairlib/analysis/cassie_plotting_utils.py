# python imports
import lcm
import numpy as np

# lcmtype imports
import dairlib
import drake

# dairlib python binding imports
from pydairlib.cassie.cassie_utils import AddCassieMultibody

# drake imports
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder

cassie_urdf = "examples/Cassie/urdf/cassie_v2.urdf"
cassie_urdf_no_springs = "examples/Cassie/urdf/cassie_fixed_springs.urdf"
cassie_default_channels = \
    {'CASSIE_STATE_SIMULATION': dairlib.lcmt_robot_output,
     'CASSIE_STATE_DISPATCHER': dairlib.lcmt_robot_output,
     'CASSIE_INPUT': dairlib.lcmt_robot_input,
     'OSC_WALKING': dairlib.lcmt_robot_input,
     'OSC_STANDING': dairlib.lcmt_robot_input,
     'OSC_JUMPING': dairlib.lcmt_robot_input,
     'OSC_RUNNING': dairlib.lcmt_robot_input,
     'CASSIE_OUTPUT': dairlib.lcmt_cassie_out,
     'OSC_DEBUG_STANDING': dairlib.lcmt_osc_output,
     'OSC_DEBUG_WALKING': dairlib.lcmt_osc_output,
     'OSC_DEBUG_JUMPING': dairlib.lcmt_osc_output,
     'OSC_DEBUG_RUNNING': dairlib.lcmt_osc_output}

cassie_default_channels_archive = \
    {'CASSIE_STATE_SIMULATION': dairlib.lcmt_robot_output,
     'CASSIE_STATE_DISPATCHER': dairlib.lcmt_robot_output,
     'CASSIE_INPUT': dairlib.lcmt_robot_input,
     'OSC_WALKING': dairlib.lcmt_robot_input,
     'OSC_STANDING': dairlib.lcmt_robot_input,
     'OSC_JUMPING': dairlib.lcmt_robot_input,
     'OSC_RUNNING': dairlib.lcmt_robot_input,
     'CASSIE_OUTPUT': dairlib.lcmt_cassie_out}

cassie_contact_channels = \
    {'CASSIE_CONTACT_DRAKE': drake.lcmt_contact_results_for_viz,
     'CASSIE_GM_CONTACT_DISPATCHER': drake.lcmt_contact_results_for_viz}


def make_plant_and_context(floating_base=True, springs=True):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    if springs:
        AddCassieMultibody(plant, scene_graph,
                           floating_base, cassie_urdf, True, True, True)
    else:
        AddCassieMultibody(plant, scene_graph,
                           floating_base, cassie_urdf_no_springs, False, True, True)

    plant.Finalize()
    return plant, plant.CreateDefaultContext()


def get_toe_frames_and_points(plant):
    front_contact_pt = np.array((-0.0457, 0.112, 0))
    rear_contact_pt = np.array((0.088, 0, 0))
    mid_contact_pt = 0.5 * (front_contact_pt + rear_contact_pt)

    left_frame = plant.GetBodyByName("toe_left")
    right_frame = plant.GetBodyByName("toe_right")

    frames = {"left": left_frame, "right": right_frame}
    pts = {"rear": rear_contact_pt, "mid": mid_contact_pt,
           "front": front_contact_pt}

    return frames, pts
