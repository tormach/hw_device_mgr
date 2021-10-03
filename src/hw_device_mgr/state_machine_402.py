import rospy


class StateMachine402:
    def __init__(self):
        self.slave_online = self.prev_slave_online = False
        self.slave_oper = self.prev_slave_oper = False
        self.status_word = self.prev_status_word = 0x0000
        self.curr_state = self.prev_state = 'START'
        self.goal_state = 'SWITCH ON DISABLED'
        self.control_mode = self.MODE_CSP
        self.curr_state_flags = dict()
        self.param_605a = 3  # QUICK STOP ACTIVE auto TRANSITION_12
        self.voltage_enabled = False

    # Modes available by setting 6060h, feedback in 6061h;
    # supported modes have corresponding bit set in 6502h
    MODE_PP = 1
    MODE_PV = 3
    MODE_PT = 4
    MODE_HM = 6
    MODE_IP = 7
    MODE_CSP = 8
    MODE_CSV = 9
    MODE_CST = 10

    # Control and status word bits not used for CiA402 state machine
    # operation may have other purposes
    cw_extra_bits = dict(
        # SWITCH_ON=0,                # (CiA402)
        # ENABLE_VOLTAGE=1,           # (CiA402)
        # QUICK_STOP=2,               # (CiA402)
        # ENABLE_OPERATION (S-ON)=3,  # (CiA402)
        OPERATION_MODE_SPECIFIC_1=4,  # HM=HOMING_START
        OPERATION_MODE_SPECIFIC_2=5,
        OPERATION_MODE_SPECIFIC_3=6,
        # FAULT_RESET=7,              # (CiA402)
        HALT=8,
        NA_1=9,
        NA_2=10,
        MANUFACTURER_SPECIFIC_1=11,
        MANUFACTURER_SPECIFIC_2=12,
        MANUFACTURER_SPECIFIC_3=13,
        MANUFACTURER_SPECIFIC_4=14,
        MANUFACTURER_SPECIFIC_5=15,
    )
    sw_extra_bits = dict(
        # READY_TO_SWITCH_ON=0,        # (CiA402)
        # SWITCH_ON=1,                 # (CiA402)
        # OPERATION_ENABLED=2,         # (CiA402)
        # FAULT=3,                     # (CiA402)
        VOLTAGE_ENABLED=4,
        # QUICK_STOP_ACTIVE=5,         # (CiA402)
        # SWITCH_ON_DISABLED=6,        # (CiA402)
        WARNING=7,  # (CiA402)
        MANUFACTURER_SPECIFIC_1=8,
        REMOTE=9,
        TARGET_REACHED=10,
        INTERNAL_LIMIT_ACTIVE=11,
        OPERATION_MODE_SPECIFIC_1=12,  # HM=HOMING_ATTAINED
        OPERATION_MODE_SPECIFIC_2=13,  # HM=HOMING_ERROR; others=FOLLOWING_ERROR
        MANUFACTURER_SPECIFIC_2=14,
        HOMING_COMPLETED=15,
    )

    def set_control_mode(self, mode):
        if isinstance(mode, str):
            mode = getattr(self, mode)
        self.control_mode = mode

    def get_control_mode(self):
        return self.control_mode

    states_402 = {
        'START': None,
        'NOT READY TO SWITCH ON': [0x4F, 0x00],
        'SWITCH ON DISABLED': [0x4F, 0x40],
        'READY TO SWITCH ON': [0x6F, 0x21],
        'SWITCHED ON': [0x6F, 0x23],
        'OPERATION ENABLED': [0x6F, 0x27],
        'FAULT REACTION ACTIVE': [0x4F, 0x0F],
        'FAULT': [0x4F, 0x08],
        'QUICK STOP ACTIVE': [0x6F, 0x07],
    }

    def check_voltage_enabled(self, control_word):
        # Power at drive motor power inputs when status word bit 4 set
        self.voltage_enabled = bool(control_word & 0x10)

    def fake_status_word(self, control_word):
        # Given a state name and flags, return a manufactured status
        # word; used in simulation
        masked_cw = control_word & 0x008F
        state = self.curr_state  # No change by default
        if (
            self.curr_state
            in [
                'START',
                'NOT READY TO SWITCH ON',
                'FAULT REACTION ACTIVE',
            ]
            + (['QUICK STOP ACTIVE'] if self.param_605a <= 3 else [])
        ):
            # Automatic transitions ignore control word
            state = self.get_next_state()
        else:
            for test_cw, test_states, next_state in [
                # Order matters!  Non-matches will fall through; None
                # matches any starting state
                (0x0080, ['FAULT'], 'SWITCH ON DISABLED'),
                (0x0000, ['FAULT'], 'FAULT'),
                (0x0000, None, 'SWITCH ON DISABLED'),
                (0x0002, ['FAULT'], 'FAULT'),
                (0x0002, ['OPERATION ENABLED'], 'QUICK STOP ACTIVE'),
                (0x0002, None, 'SWITCH ON DISABLED'),
                (
                    0x0006,
                    # (I forgot where I read quick stop triggers these
                    # transitions)
                    [
                        'SWITCH ON DISABLED',
                        'SWITCHED ON',
                        'OPERATION ENABLED',
                    ],
                    'READY TO SWITCH ON',
                ),
                (
                    0x0007,
                    [
                        'READY TO SWITCH ON',
                        'OPERATION ENABLED',
                    ],
                    'SWITCHED ON',
                ),
                (
                    0x000F,
                    ['SWITCHED ON', 'QUICK STOP ACTIVE'],
                    'OPERATION ENABLED',
                ),
            ]:
                if masked_cw == test_cw and (
                    test_states is None or self.curr_state in test_states
                ):
                    state = next_state
                    break

        status_word = self.states_402[state][1]

        cw_flags = self.get_control_word_flags(control_word)
        sw_flags = dict(
            HOMING_COMPLETED=cw_flags['OPERATION_MODE_SPECIFIC_1'],
            VOLTAGE_ENABLED=getattr(self, 'sim_voltage_enabled', True),
        )
        status_word = self.add_status_word_flags(status_word, **sw_flags)
        return state, status_word

    @classmethod
    def add_status_word_flags(cls, status_word, **flags):
        # Add flags by name to status word; used in simulation
        for flag, val in flags.items():
            operand = 1 << cls.sw_extra_bits[flag]
            if val:
                status_word |= operand
            else:
                status_word &= ~operand & 0xFFFF

        return status_word

    @classmethod
    def get_control_word_flags(cls, control_word):
        # Get flags by name from control word; used in simulation
        flags = dict()
        for name, bitnum in cls.cw_extra_bits.items():
            flags[name] = bool(control_word & 1 << bitnum)
        return flags

    def update_state(self, slave_online, slave_oper, status_word):
        # EtherCAT State Machine
        self.prev_slave_online = self.slave_online
        self.slave_online = slave_online
        self.prev_slave_oper = self.slave_oper
        self.slave_oper = slave_oper
        if not slave_online or not slave_oper:
            return

        # 402 State Machine
        self.prev_status_word = self.status_word
        self.status_word = status_word
        self.prev_state = self.curr_state
        self.check_voltage_enabled(status_word)
        for key, state in self.states_402.items():
            if state is None:
                continue  # Start state
            # check if the value after applying the bitmask (value[0])
            # corresponds with the value[1] to determine the current status
            s_mask, s_word = state
            if status_word & s_mask == s_word:
                self.curr_state = key
                break
        else:
            rospy.logwarn(
                f"Unknown status word 0x{status_word:X}; "
                f"state {self.curr_state} unchanged"
            )

        # - Extract extra mode-specific flags from status word
        self.curr_state_flags.clear()
        for flag, bitnum in self.sw_extra_bits.items():
            self.curr_state_flags[flag] = bool(status_word & (1 << bitnum))

    @property
    def operational(self):
        return bool(self.slave_online and self.slave_oper)

    @property
    def slave_online_changed(self):
        return self.slave_online is not self.prev_slave_online

    @property
    def slave_oper_changed(self):
        return self.slave_oper is not self.prev_slave_oper

    def get_status_flag(self, flag):
        return self.curr_state_flags.get(flag, False)

    def drive_state_changed(self):
        return self.prev_state != self.curr_state

    goal_paths = {
        # These dicts map the current state to [next state,
        # transition] to arrive at some goal state.  When the
        # transition is `None`, the final state has been reached.
        # Some transitions happen automatically; those are marked with
        # a `None` value in the `transitions` dict.
        'SWITCHED ON': {
            # Drives in OPERATION ENABLED move to QUICK STOP ACTIVE
            'OPERATION ENABLED': ['QUICK STOP ACTIVE', 'TRANSITION_11'],
            # QUICK STOP ACTIVE: if 605Ah from 0-3, automatic
            # TRANSITION_12 to SWITCH ON DISABLED; otherwise hold
            'QUICK STOP ACTIVE': ['QUICK STOP ACTIVE', None],  # End state
            # Transition other drives to SWITCHED ON
            'START': ['NOT READY TO SWITCH ON', 'TRANSITION_0'],
            'NOT READY TO SWITCH ON': ['SWITCH ON DISABLED', 'TRANSITION_1'],
            'SWITCH ON DISABLED': ['READY TO SWITCH ON', 'TRANSITION_2'],
            'READY TO SWITCH ON': ['SWITCHED ON', 'TRANSITION_3'],
            'SWITCHED ON': ['SWITCHED ON', None],  # End state
            'FAULT': ['SWITCH ON DISABLED', 'TRANSITION_15'],
            'FAULT REACTION ACTIVE': ['FAULT', 'TRANSITION_14'],
        },
        'OPERATION ENABLED': {
            # Drives transition to OPERATION ENABLED; note the
            # Hal402Mgr always brings drives to SWITCHED ON state
            # first before setting OPERATION ENABLED goal state
            'SWITCHED ON': ['OPERATION ENABLED', 'TRANSITION_4'],
            'OPERATION ENABLED': ['OPERATION ENABLED', None],  # End
            'START': ['START', 'TRANSITION_0'],
            'NOT READY TO SWITCH ON': ['SWITCH ON DISABLED', 'TRANSITION_1'],
            'SWITCH ON DISABLED': ['READY TO SWITCH ON', 'TRANSITION_2'],
            'READY TO SWITCH ON': ['SWITCHED ON', 'TRANSITION_3'],
            'FAULT': ['SWITCH ON DISABLED', 'TRANSITION_15'],
            'FAULT REACTION ACTIVE': ['FAULT', 'TRANSITION_14'],
            'QUICK STOP ACTIVE': ['OPERATION ENABLED', 'TRANSITION_16'],
        },
        # These transitions take longer from OPERATION ENABLED -> SWITCH ON DISABLED
        # 'OPERATION ENABLED':        ['SWITCHED ON', 'TRANSITION_5'],
        # 'SWITCHED ON':              ['READY TO SWITCH ON', 'TRANSITION_6'],
        # 'READY TO SWITCH ON':       ['SWITCH ON DISABLED', 'TRANSITION_7']
        'SWITCH ON DISABLED': {
            'START': ['NOT READY TO SWITCH ON', 'TRANSITION_0'],
            'NOT READY TO SWITCH ON': ['SWITCH ON DISABLED', 'TRANSITION_1'],
            'SWITCH ON DISABLED': ['SWITCH ON DISABLED', None],  # End State
            'READY TO SWITCH ON': ['SWITCH ON DISABLED', 'TRANSITION_7'],
            'SWITCHED ON': ['SWITCH ON DISABLED', 'TRANSITION_10'],
            'FAULT REACTION ACTIVE': ['FAULT', 'TRANSITION_14'],
            'FAULT': ['SWITCH ON DISABLED', 'TRANSITION_15'],
            # ENABLED state effect QUICK STOP ACTIVE; then (605Ah set
            # to 0-3) automatic TRANSITION_12 to SWITCH ON DISABLED
            'OPERATION ENABLED': ['QUICK STOP ACTIVE', 'TRANSITION_11'],
            'QUICK STOP ACTIVE': ['SWITCH ON DISABLED', 'TRANSITION_12'],
        },
        # Fault state has three possible final states; see inline notes
        'FAULT': {
            # Drives in FAULT state remain in that state
            'FAULT REACTION ACTIVE': ['FAULT', 'TRANSITION_14'],
            'FAULT': ['FAULT', None],  # End state
            # ENABLED state effect QUICK STOP ACTIVE; then (605Ah set
            # to 0-3) automatic TRANSITION_12 to SWITCH ON DISABLED
            'OPERATION ENABLED': ['QUICK STOP ACTIVE', 'TRANSITION_11'],
            'QUICK STOP ACTIVE': ['QUICK STOP ACTIVE', None],  # End state
            # Drives in all other states transition to SWITCH ON DISABLED
            'START': ['NOT READY TO SWITCH ON', 'TRANSITION_0'],
            'NOT READY TO SWITCH ON': ['SWITCH ON DISABLED', 'TRANSITION_1'],
            'SWITCH ON DISABLED': ['SWITCH ON DISABLED', None],  # End state
            'READY TO SWITCH ON': ['SWITCH ON DISABLED', 'TRANSITION_7'],
            'SWITCHED ON': ['SWITCH ON DISABLED', 'TRANSITION_10'],
        },
    }

    def set_goal_state(self, state):
        if state not in self.goal_path:
            raise KeyError(f"Path to goal '{state}' unknown")
        self.goal_state = state

    def get_goal_state(self):
        return self.goal_state

    @property
    def goal_path(self):
        return self.goal_paths[self.goal_state]

    def get_next_state(self):
        # Return next expected state
        # Special case:  QUICK STOP ACTIVE automatic TRANSITION_12
        if self.curr_state == 'QUICK STOP ACTIVE' and self.param_605a <= 3:
            return 'SWITCH ON DISABLED'
        return self.goal_path[self.curr_state][0]

    def get_next_transition(self):
        # Special case:  QUICK STOP ACTIVE automatic TRANSITION_12
        if self.curr_state == 'QUICK STOP ACTIVE' and self.param_605a <= 3:
            return 'TRANSITION_12'
        return self.goal_path[self.curr_state][1]

    def is_goal_state_reached(self):
        return self.operational and self.get_next_transition() is None

    def get_control_word(self, **flags):
        # If drive not operational, all bets are off
        if not self.operational:
            return self.hold_state_control_word['SWITCH ON DISABLED']

        # Get base control word
        if self.is_goal_state_reached():
            # Holding current state
            control_word = self.get_hold_state_control_word()
        else:
            # Transitioning to next state
            control_word = self.get_transition_control_word()
        if control_word is None:
            # `None` only possible when goal state not reached and
            # current state is in one of the four automatic
            # transitions.  In all four cases, 0x0000 works:
            #
            # - Transitions 0 and 1:  Final state SWITCH ON DISABLED,
            #   control word is 0x0000
            #
            # - Transitions 13 and 14:  Final state FAULT, control
            #   word is anything BUT 0x0080
            control_word = 0x0000

        # Add flags and return
        return self.add_control_word_flags(control_word, **flags)

    # Map drive states to control word that maintains the state.
    # `None` indicates hold state N/A in automatic transition states
    # where control word is ignored
    hold_state_control_word = {
        'START': None,
        'NOT READY TO SWITCH ON': None,
        'SWITCH ON DISABLED': 0x0000,
        'READY TO SWITCH ON': 0x0006,
        'SWITCHED ON': 0x0007,
        'OPERATION ENABLED': 0x000F,
        'QUICK STOP ACTIVE': 0x0002,
        'FAULT REACTION ACTIVE': None,
        'FAULT': 0x0000,  # Anything but 0x0080 will hold state
    }

    def get_hold_state_control_word(self):
        control_word = self.hold_state_control_word[self.curr_state]
        if control_word is None:
            raise ValueError(
                f"BUG:  No hold state control word for {self.curr_state}"
            )
        return control_word

    # Map transition to the control word that would effect the
    # transition; None indicates automatic transition where control
    # word is ignored
    transitions = dict(
        TRANSITION_0=None,  # START->NOT READY TO SWITCH ON
        TRANSITION_1=None,  # NOT READY TO SWITCH ON->SWITCH ON DISABLED
        TRANSITION_2=0x0006,  # SWITCH ON DISABLED->READY TO SWITCH ON
        TRANSITION_3=0x0007,  # READY TO SWITCH ON->SWITCHED ON
        TRANSITION_4=0x000F,  # SWITCHED ON->OPERATION ENABLED
        TRANSITION_5=0x0007,  # OPERATION ENABLED->SWITCHED ON
        TRANSITION_6=0x0006,  # SWITCHED ON->READY TO SWITCH ON
        TRANSITION_7=0x0000,  # READY TO SWITCH ON->SWITCH ON DISABLED
        TRANSITION_8=0x0006,  # OPERATION ENABLED->READY TO SWITCH ON
        TRANSITION_9=0x0000,  # OPERATION ENABLED->SWITCH ON DISABLED
        TRANSITION_10=0x0000,  # SWITCHED ON->SWITCH ON DISABLED
        TRANSITION_11=0x0002,  # OPERATION ENABLED->QUICK STOP ACTIVE
        TRANSITION_12=0x0000,  # QUICK STOP ACTIVE->SWITCH ON DISABLED *
        TRANSITION_13=None,  # (Any)->FAULT REACTION ACTIVE
        TRANSITION_14=None,  # FAULT REACTION ACTIVE->FAULT
        TRANSITION_15=0x0080,  # FAULT->SWITCH ON DISABLED **
        TRANSITION_16=0x000F,  # QUICK STOP ACTIVE->OPERATION ENABLED *
        # * Transitions 12, 16:  Set 605Ah "Quick stop option code" value:
        # - 0:    coast to stop; automatic TRANSITION_12
        # - 1-3:  stop @ 2007-10h torque; automatic TRANSITION_12
        # - 5-7:  stop @ 2007-10h torque; hold; then can TRANSITION_12 or 16
        # ** Transition 15:  Fault cleared on rising edge of bit 7
    )

    def get_transition_control_word(self):
        # Look up next transition and return control word to effect it
        transition = self.get_next_transition()
        if transition is None:
            # Goal state reached
            return None
        else:
            return self.transitions[transition]

    @classmethod
    def add_control_word_flags(cls, control_word, **flags):
        # Add mode-specific flags
        for flag, val in flags.items():
            operand = 1 << cls.cw_extra_bits[flag]
            if val:
                control_word |= operand
            else:
                control_word &= ~operand & 0xFFFF

        return control_word
