import io
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


class FrankaPlotConfig():
    def __init__(self, filename):
        data = load(io.open(filename, 'r'), Loader=Loader)
        self.channel_x = data['channel_x']
        self.channel_u = data['channel_u']
        self.channel_osc = data['channel_osc']
        self.channel_c3 = data['channel_c3']
        self.channel_tray = data['channel_tray']
        self.channel_c3_target = data['channel_c3_target']
        self.channel_c3_actual = data['channel_c3_actual']
        self.plot_style = data['plot_style']

        self.start_time = data['start_time']
        self.duration = data['duration']
        self.plot_joint_positions = data['plot_joint_positions']
        self.plot_joint_velocities = data['plot_joint_velocities']
        self.plot_measured_efforts = data['plot_measured_efforts']
        self.plot_commanded_efforts = data['plot_commanded_efforts']
        self.plot_contact_forces = data['plot_contact_forces']
        self.plot_end_effector = data['plot_end_effector']
        self.pos_names = \
            data['special_positions_to_plot'] if \
            data['special_positions_to_plot'] else []
        self.vel_names = \
            data['special_velocities_to_plot'] if \
            data['special_velocities_to_plot'] else []
        self.act_names = \
            data['special_efforts_to_plot'] if \
            data['special_efforts_to_plot'] else []

        self.fsm_state_names = data['fsm_state_names']

        self.plot_qp_costs = data['plot_qp_costs']
        self.plot_qp_solve_time = data['plot_qp_solve_time']
        self.plot_qp_solutions = data['plot_qp_solutions']
        self.plot_tracking_costs = data['plot_tracking_costs']
        self.plot_active_tracking_datas = data['plot_active_tracking_datas']
        self.tracking_datas_to_plot = \
            data['tracking_datas_to_plot'] if data['tracking_datas_to_plot'] else []
        self.plot_c3_debug = data['plot_c3_debug']
        self.plot_c3_tracking = data['plot_c3_tracking']
        self.plot_object_state = data['plot_object_state']
