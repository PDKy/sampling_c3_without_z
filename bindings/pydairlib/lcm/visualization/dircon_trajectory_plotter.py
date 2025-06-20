import sys
import matplotlib.pyplot as plt
from pydairlib.lcm import lcm_trajectory
from pydairlib.common import FindResourceOrThrow
from pydrake.trajectories import PiecewisePolynomial
import numpy as np

from pydrake.all import (DiagramBuilder, AddMultibodyPlantSceneGraph)
from pydairlib.cassie.cassie_utils import AddCassieMultibody

def main():

  builder = DiagramBuilder()
  plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
  AddCassieMultibody(plant, scene_graph,
                     True, "examples/Cassie/urdf/cassie_fixed_springs.urdf", False, True)

  plant.Finalize()
  # Default filename for the example
  # filename = FindResourceOrThrow("examples/Cassie/saved_trajectories/jumping_0.15h_0.3d")
  filename = FindResourceOrThrow(
    "examples/Cassie/saved_trajectories/jumping_0.0h_0.6d_4")
  # filename = FindResourceOrThrow(
  #   "examples/Cassie/saved_trajectories/jumping_box_0.5h_0.3d_1")
  # filename = FindResourceOrThrow(
  #   "examples/Cassie/saved_trajectories/jumping_box_0.4h_0.3d_5")

  # filename = FindResourceOrThrow("examples/Cassie/saved_trajectories/" + sys.argv[1])
  dircon_traj = lcm_trajectory.DirconTrajectory(plant, filename)

  # Reconstructing state and input trajectory as piecewise polynomials
  state_traj = dircon_traj.ReconstructStateTrajectory()
  input_traj = dircon_traj.ReconstructInputTrajectory()
  state_datatypes = dircon_traj.GetTrajectory("state_traj0").datatypes
  input_datatypes = dircon_traj.GetTrajectory("input_traj").datatypes
  force_samples = dircon_traj.GetTrajectory("force_vars2").datapoints
  force_t_samples = dircon_traj.GetStateBreaks(0)
  force_traj = dircon_traj.ReconstructLambdaTrajectory()
  # force_datatypes = dircon_traj.GetTrajectory("force_vars0").datatypes
  force_datatypes = dircon_traj.GetTrajectory("force_vars2").datatypes

  # impulse_samples = dircon_traj.GetImpulseSamples(2)
  collocation_force_points = dircon_traj.GetCollocationForceSamples(0)
  # M = reflected_joints()
  #
  # mirror_traj = lcm_trajectory.Trajectory()
  # mirror_traj.traj_name = 'mirror_matrix'
  # mirror_traj.time_vector = np.zeros(M.shape[0])
  # mirror_traj.datapoints = MCc
  # mirror_traj.datatypes = [''] * M.shape[0]
  #
  # dircon_traj.AddTrajectory('mirror_matrix', mirror_traj)
  # dircon_traj.WriteToFile(filename)


  plot_only_at_knotpoints = False

  n_points = 500
  t = np.linspace(state_traj.start_time(), state_traj.end_time(), n_points)
  if plot_only_at_knotpoints:
    n_points = force_t_samples.shape[0]
    t = force_t_samples
  state_samples = np.zeros((n_points, state_traj.value(0).shape[0]))
  state_derivative_samples = np.zeros((n_points, state_traj.value(0).shape[0]))
  input_samples = np.zeros((n_points, input_traj.value(0).shape[0]))
  # force_samples = np.zeros((n_points, force_traj[0].value(0).shape[0]))
  for i in range(n_points):
    state_samples[i] = state_traj.value(t[i])[:, 0]
    state_derivative_samples[i] = state_traj.EvalDerivative(t[i], 1)[:, 0]
    input_samples[i] = input_traj.value(t[i])[:, 0]
    # force_samples[i] = force_traj[0].value(t[i])[:, 0]

  # reflected_state_samples = state_samples @ M
  # Plotting reconstructed state trajectories
  plt.figure("state trajectory")
  plt.plot(t, state_samples[:, 4:7])
  # plt.plot(t, state_samples[:, 22:25])
  # plt.plot(t + state_traj.end_time(), reflected_state_samples[:, 0:7])
  # plt.plot(t, state_samples[:, -18:])
  # plt.plot(t + state_traj.end_time(), reflected_state_samples[:, 7:13])
  # plt.plot(t, state_samples[:, 25:31])
  # plt.plot(t + state_traj.end_time(), reflected_state_samples[:, 25:31])
  plt.legend(state_datatypes[4:7])
  # plt.legend(state_datatypes[22:25])

  # Velocity Error
  # plt.figure('velocity_error')
  # import pdb; pdb.set_trace()
  # plt.plot(t, (state_samples[:, (19 + 3):] - state_derivative_samples[:, 4:19]))
  # plt.plot(t, (state_samples[:, -2:] - state_derivative_samples[:, 21:23]))
  # plt.legend(state_datatypes[26:])
  # plt.plot(t, state_samples[:, (23 + 3):])


  plt.figure("input trajectory")
  print(np.diff(input_samples[:, 7]))
  plt.plot(t, input_samples[:, :])
  plt.legend(input_datatypes[:])

  plt.figure("force trajectory")
  # plt.plot(t, force_samples[:, :12])
  plt.plot(force_t_samples, force_samples.T)
  plt.legend(force_datatypes)

  plt.show()

def reflected_joints():

  mirror = np.zeros((37, 37))
  mirror[0:7, 0:7] = np.eye(7)
  mirror[19:25, 19:25] = np.eye(6)
  joint_slice = range(7, 19, 2)
  joint_vel_slice = range(19 + 6, 19 + 18, 2)
  asy_indices = {7, 9, 25, 27}
  mirror[1, 1] = -1
  mirror[3, 3] = -1
  mirror[5, 5] = -1

  mirror[19, 19] = -1
  mirror[21, 21] = -1
  mirror[23, 23] = -1
  for i in joint_slice:
    if(i in asy_indices):
      mirror[i,i+1] = -1
      mirror[i+1,i] = -1
    else:
      mirror[i,i+1] = 1
      mirror[i+1,i] = 1
  for i in joint_vel_slice:
    if(i in asy_indices):
      mirror[i,i+1] = -1
      mirror[i+1,i] = -1
    else:
      mirror[i,i+1] = 1
      mirror[i+1,i] = 1
  return mirror

if __name__ == "__main__":
  main()


