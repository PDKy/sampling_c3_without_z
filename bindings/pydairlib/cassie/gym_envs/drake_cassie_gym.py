import numpy as np

from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import *
from pydrake.systems.analysis import Simulator

from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.cassie_utils import *
from pydairlib.multibody import *
from pydairlib.systems.primitives import *
from pydairlib.systems.robot_lcm_systems import RobotOutputSender
from dairlib import lcmt_radio_out
from pydairlib.cassie.simulators import CassieSimDiagram
from pydairlib.cassie.gym_envs.cassie_env_state import CassieEnvState


class DrakeCassieGym():
    def __init__(self, reward_func, visualize=False):
        self.sim_dt = 1e-3
        self.visualize = visualize
        self.reward_func = reward_func
        self.start_time = 0.35
        self.current_time = 0.00
        # self.end_time = 0.05
        self.hardware_traj = None
        self.action_dim = 10
        self.state_dim = 45
        self.x_init = np.array(
            [1, 0, 0, 0, 0, 0, 0.9, -0.0358636, 0, 0.67432, -1.588, -0.0458742, 1.90918,
             -0.0381073, -1.82312, 0.0358636, 0, 0.67432, -1.588, -0.0457885, 1.90919, -0.0382424, -1.82321,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.prev_cassie_state = None
        self.controller = None
        self.terminated = False
        self.initialized = False


    def make(self, controller, urdf='examples/Cassie/urdf/cassie_v2.urdf'):
        self.builder = DiagramBuilder()
        self.dt = 1e-3
        self.plant = MultibodyPlant(self.dt)
        self.controller = controller
        self.simulator = CassieSimDiagram(self.plant, urdf, self.visualize, 0.8, 1e4, 1e2)
        self.new_plant = self.simulator.get_plant()
        self.builder.AddSystem(self.controller)
        self.builder.AddSystem(self.simulator)

        self.builder.Connect(self.controller.get_output_port_robot_input(), self.simulator.get_input_port_actuation())
        self.builder.Connect(self.simulator.get_output_port_state(), self.controller.get_input_port_state())
        # self.builder.Connect(self.simulator.get_output_port_cassie_out(),
        #                      self.controller.get_cassie_out_input_port())
        # self.builder.Connect(self.controller, self.simulator.get_input_port_radio())

        self.diagram = self.builder.Build()
        self.sim = Simulator(self.diagram)
        self.simulator_context = self.diagram.GetMutableSubsystemContext(self.simulator, self.sim.get_mutable_context())
        self.controller_context = self.diagram.GetMutableSubsystemContext(self.controller, self.sim.get_mutable_context())
        self.controller_output_port = self.controller.get_output_port_torque()
        self.sim.get_mutable_context().SetTime(self.start_time)
        self.reset()
        self.initialized = True

    def reset(self):
        self.new_plant.SetPositionsAndVelocities(self.new_plant.GetMyMutableContextFromRoot(
            self.sim.get_mutable_context()), self.x_init)
        self.sim.get_mutable_context().SetTime(self.start_time)
        x = self.plant.GetPositionsAndVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_context()))
        u = np.zeros(10)
        self.sim.Initialize()
        self.current_time = self.start_time
        self.prev_cassie_state = CassieEnvState(self.current_time, x, u, np.zeros(18))
        self.cassie_state = CassieEnvState(self.current_time, x, u, np.zeros(18))
        self.terminated = False
        return

    def advance_to(self, time):
        cumulative_reward = 0
        while self.current_time < time and not self.terminated:
            _, reward = self.step()
            cumulative_reward += reward
        return cumulative_reward

    def check_termination(self):
        return self.cassie_state.get_fb_positions()[2] < 0.4


    def step(self, action=np.zeros(18)):
        if not self.initialized:
            print("Call make() before calling step() or advance()")
        next_timestep = self.sim.get_context().get_time() + self.sim_dt
        action = self.velocity_profile(next_timestep)
        self.simulator.get_input_port_radio().FixValue(self.simulator_context, action)
        self.controller.get_input_port_radio().FixValue(self.controller_context, action)
        self.sim.AdvanceTo(next_timestep)
        self.current_time = self.sim.get_context().get_time()

        x = self.plant.GetPositionsAndVelocities(
            self.plant.GetMyMutableContextFromRoot(
                self.sim.get_context()))
        u = self.controller_output_port.Eval(self.controller_context)[:-1] # remove the timestamp
        # u = self.controller_output_port.Eval(self.controller_context)[:-1] # remove the timestamp
        # print(u)
        self.cassie_state = CassieEnvState(self.current_time, x, u, action)
        reward = self.reward_func.compute_reward(self.sim_dt, self.cassie_state, self.prev_cassie_state)
        self.terminated = self.check_termination()
        self.prev_cassie_state = self.cassie_state
        return self.cassie_state, reward

    def velocity_profile(self, timestep):
        velocity_command = np.zeros(18)
        velocity_command[2] = min(0.1 * timestep, 1.0)
        # velocity_command[2] = 5.0
        return velocity_command

    def get_traj(self):
        return self.traj

    # Some simulators for Cassie require cleanup
    def free_sim(self):
        return
