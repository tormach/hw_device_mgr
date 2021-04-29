import rospy
from .pins import HALPins
from .state_machine_402 import StateMachine402

# import messages from the ROS node
from hal_402_device_mgr.msg import msg_error


class Drive402:
    GENERIC_ERROR_DESCRIPTION = 'This is an unknown error'
    GENERIC_ERROR_SOLUTION = (
        'Please consult the troubleshooting section of your hardware manual'
    )

    MODE_DEFAULT = StateMachine402.MODE_CSP
    DEVICE_FAULT_CODE_PARAM = 'device_fault_code_list'

    ########################################
    # Init

    def __init__(
        self,
        drive_name,
        drive_type,
        slave_number,
        comp,
        transition_timeout=1.0,
        sim=False,
    ):
        self.drive_name = drive_name
        self.drive_type = drive_type
        self.slave_number = slave_number
        self.comp = comp
        self.transition_timeout = transition_timeout
        self.sim = sim

        self.pins = {}
        self.control_flags = dict()
        self.topics = {}
        self.devices_error_list = {}
        self.sm402 = StateMachine402()

    @property
    def compname(self):
        return self.comp.getprefix()

    def init(self):
        self.setup_pins()

        # read in the error list from parameters
        self.device_error_list = self.read_device_error_list()

        self.create_topics()

    pin_specs = {
        # NOTE: error-code is deprecated (need the space in PDO for
        # homing options)
        'error-code': dict(ptype='u32', pdir='in'),
        'aux-error-code': dict(ptype='u32', pdir='in'),
        'status-word': dict(ptype='u32', pdir='in'),
        'status-word-sim': dict(ptype='u32', pdir='out'),
        'control-word': dict(ptype='u32', pdir='out'),
        'control-word-fb': dict(ptype='u32', pdir='in'),
        'drive-mode-cmd': dict(ptype='u32', pdir='out'),
        'drive-mode-fb': dict(ptype='u32', pdir='in'),
    }

    def setup_pins(self):
        prefix = f'{self.drive_name}.'
        self.pins = HALPins(self.comp, self.pin_specs, prefix=prefix)
        self.pins.init_pins()
        self.set_control_mode(self.MODE_DEFAULT)

    def read_device_error_list(self):
        param = f'/{self.DEVICE_FAULT_CODE_PARAM}/{self.drive_type}'
        if not rospy.has_param(param):
            rospy.logerr(f"ROS param {param} unset")
            return {}

        return rospy.get_param(param)

    def create_topics(self):
        # for each drive, an error and status topic are created
        # messages are defined in msg/ directory of this package
        self.topics = {
            'error': rospy.Publisher(
                f'{self.compname}/{self.drive_name}_error',
                msg_error,
                queue_size=1,
                latch=True,
            ),
        }

    ########################################
    # Main logic and external interface

    def set_goal_state(self, goal_state):
        self.sm402.set_goal_state(goal_state)

    def get_goal_state(self):
        return self.sm402.get_goal_state()

    def read_state(self):
        self.read_halpins()
        self.update_state_machine()

    def write_state(self):
        self.effect_next_transition()
        self.publish_status()
        self.write_halpins()
        # In sim mode, update status word to simulate next state
        if self.sim is True:
            self.sim_fake_next_inputs()

    def read_halpins(self):
        # Read from input HAL pins
        self.pins.read_all()

    def update_state_machine(self):
        self.sm402.update_state(self.pins.status_word.get())

    def set_control_flags(self, **flags):
        self.control_flags = flags

    def get_status_flag(self, flag):
        return self.sm402.get_status_flag(flag)

    def effect_next_transition(self):
        # Set drive control mode
        self.pins.drive_mode_cmd.set(self.sm402.get_control_mode())

        # Set next control word
        control_word = self.sm402.get_control_word(**self.control_flags)
        self.pins.control_word.set(control_word)

        if self.pins.control_word.changed or self.pins.status_word.changed:
            transition = self.sm402.get_next_transition()
            curr_state = self.sm402.curr_state
            next_state = self.sm402.get_next_state()
            if transition is None:
                transition = '(Hold state)'
            rospy.loginfo(
                f"{self.drive_name} control word 0x{control_word:04X}"
                f" {transition}:  {curr_state} -> {next_state}"
            )
        return

    def write_halpins(self):
        # Write to output HAL pins
        self.pins.write_all()

    def is_goal_state_reached(self):
        return self.sm402.is_goal_state_reached()

    def sim_fake_next_inputs(self):
        # Note:  After read/update/write, this simulates external
        # changes, so HAL pins are explicitly read from/written to.

        # Fake incoming control mode on HAL pin
        control_mode = self.pins.drive_mode_cmd.hal_pin.get()
        self.pins.drive_mode_fb.hal_pin.set(control_mode)

        # Fake status word on HAL pin
        cw = self.pins.control_word_fb.hal_pin.get()
        state, status_word = self.sm402.fake_status_word(cw)
        self.pins.status_word_sim.hal_pin.set(status_word)

        prev_inputs = getattr(self, 'prev_fake_inputs', (None, None, None))
        if prev_inputs != (control_mode, state, status_word):
            rospy.loginfo(
                f'{self.drive_name} next inputs:  mode=0x{control_mode:04X};'
                f' state="{state}"; status word=0x{status_word:04X}'
            )
            self.prev_fake_inputs = (control_mode, state, status_word)

    def set_control_mode(self, mode):
        rospy.loginfo(f"{self.drive_name} entering drive control mode {mode}")
        mode = self.normalize_control_mode(mode)
        self.sm402.set_control_mode(mode)

    def get_control_mode(self):
        return self.sm402.get_control_mode()

    @classmethod
    def normalize_control_mode(cls, mode):
        if isinstance(mode, str):
            mode = getattr(StateMachine402, mode)
        return mode

    ########################################
    # ROS topics

    def publish_status(self):
        self.publish_fault_state()
        self.publish_error()

    def publish_fault_state(self):
        if not self.sm402.drive_state_changed():
            return
        if self.sm402.curr_state == 'FAULT':
            rospy.logwarn(f"{self.drive_name} entered 'FAULT' state")
        elif self.sm402.prev_state == 'FAULT':
            rospy.loginfo(f"{self.drive_name} left 'FAULT' state")

    @staticmethod
    def error_code_hex(error_code):
        return f"0x{0xFFFF & error_code:04X}" if error_code else ''

    def get_error_info(self, err_code):
        if not err_code:
            return dict(
                description="No error",
                solution="",
            )
        err_code_str = f'0x{err_code:04X}'

        err_info = self.device_error_list.get(err_code_str, None)
        if err_info is not None:
            return err_info

        return dict(
            description=Drive402.GENERIC_ERROR_DESCRIPTION,
            solution=Drive402.GENERIC_ERROR_SOLUTION,
        )

    def publish_error(self):
        if not self.pins.error_code.changed:
            return
        err_code = self.pins.error_code.get() & 0xFFFF
        err_hex = self.error_code_hex(err_code)
        err_info = self.get_error_info(err_code)
        description = err_info['description']
        solution = err_info.get('solution', '')
        self.topics['error'].publish(
            self.drive_name,
            self.drive_type,
            err_hex,
            description,
            solution,
        )
        if not self.pins.error_code.get():
            rospy.loginfo(f"{self.drive_name}: No Error")
        else:
            rospy.logerr(
                f"{self.drive_name}: Error {err_hex}: {description}\n{solution}"
            )
