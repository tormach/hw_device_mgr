from ..cia_301.device import (
    CiA301Device,
    CiA301DataType,
    CiA301SimDevice,
)


class CiA402Device(CiA301Device):
    """
    Manage a CiA 402 motor drive's state machine.

    This device will attempt to step the drive through the CiA 402
    state machine to the goal `state` (e.g. `OPERATION ENABLED`), and
    will manage other goals.

    Goal parameters:
    - `state`:  The CiA 402 goal state
    - `control_mode`:  Drive control mode, e.g. `MODE_CSP`
    - `control_word_flags`:  Control word bits to set
    - `state_flags`:  Status word bits to match for `goal_reached` feedback
    """

    data_types = CiA301DataType

    # ------- FEEDBACK -------

    # Modes available by setting 6060h, feedback in 6061h;
    # supported modes have corresponding bit set in 6502h
    MODE_NA = 0
    MODE_PP = 1
    MODE_PV = 3
    MODE_PT = 4
    MODE_HM = 6
    MODE_IP = 7
    MODE_CSP = 8
    MODE_CSV = 9
    MODE_CST = 10
    DEFAULT_CONTROL_MODE = MODE_CSP

    @classmethod
    def control_mode_int(cls, mode):
        """
        Translate control mode to integer value.

        E.g. `MODE_CSP` to `int` `8`; pass `int` back unchanged.
        """
        return getattr(cls, mode) if isinstance(mode, str) else mode

    @classmethod
    def control_mode_str(cls, mode):
        """
        Translate control mode to string value.

        E.g. `8` to `MODE_CSP`; pass `MODE_CSP` back unchanged.
        """
        if isinstance(mode, str):
            assert mode.startswith("MODE_") and hasattr(cls, mode)
            return mode
        for attr in [a for a in dir(cls) if a.startswith("MODE_")]:
            if getattr(cls, attr) == mode:
                return attr
        else:
            return f"MODE_UNKNOWN({mode})"

    # Status word bits not used for CiA402 state machine operation may
    # have other purposes
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

    # Incoming feedback from drives:  param_name : data_type
    feedback_in_data_types = dict(
        status_word="uint16",
        control_mode_fb="int8",
    )

    # Incoming feedback from drives:  param_name : inital_value
    feedback_in_defaults = dict(
        status_word=0,
        control_mode_fb=0,
    )

    # Outgoing feedback to controller:  param_name : inital_value
    feedback_out_defaults = dict(
        state="START",
        transition=None,
        state_flags={bit: False for bit in sw_extra_bits},
        **feedback_in_defaults,
    )
    feedback_out_initial_values = dict(
        status_word=0,
        control_mode_fb="MODE_NA",
    )

    def get_feedback(self):
        fb_out = super().get_feedback()

        # If lower layer goals not reached (not operational), set
        # default feedback ("START" state)
        if not fb_out.get("goal_reached"):
            self.feedback_out.update(**self.feedback_out_initial_values)
            return self.feedback_out

        # Goal reached vars
        goal_reached = True
        goal_reasons = list()

        # Status word, control mode from fb in
        sw = self.feedback_in.get("status_word")
        cm = fb_out.get("control_mode_fb")
        cm_str = self.control_mode_str(cm)
        self.feedback_out.update(status_word=sw, control_mode_fb=cm_str)
        cm_cmd = self.command_in.get("control_mode")
        if cm_str != cm_cmd:
            goal_reached = False
            goal_reasons.append(f"control_mode {cm_str} != {cm_cmd}")

        # Calculate 'state' feedback
        for state, bits in self.state_bits.items():
            # Compare masked status word with pattern to determine current state
            sw_mask, sw_pat = bits
            if sw & sw_mask == sw_pat:
                self.feedback_out.update(state=state)
                break
        else:
            raise ValueError(
                f"Unknown status word 0x{sw:X}; "
                f"state {self.feedback_out.get('state')} unchanged"
            )
        if self._get_next_transition() is not None:
            goal_reached = False
            state_cmd = self.command_in.get("state")
            sw = self.feedback_in.get("status_word")
            goal_reasons.append(f"state {state} (0x{sw:08X}) != {state_cmd}")

        # Calculate 'transition' feedback
        new_st, old_st = self.feedback_out.changed("state", return_vals=True)
        if (old_st, new_st) == ("START", "NOT READY TO SWITCH ON"):
            self.feedback_out.update(transition="TRANSITION_0")
        elif new_st == "FAULT REACTION ACTIVE":
            self.feedback_out.update(transition="TRANSITION_13")
        elif self._get_next_state(curr_state=old_st) == new_st:
            next_trans = self._get_next_transition(curr_state=old_st)
            self.feedback_out.update(transition=next_trans)
        else:
            self.feedback_out.update(transition=None)

        # Calculate 'state_flags' feedback from status word
        sf = {
            flag: bool(sw & (1 << bitnum))
            for flag, bitnum in self.sw_extra_bits.items()
        }
        self.feedback_out.update(state_flags=sf)
        sf_cmd = self.command_in.get("state_flags")
        for flag_name, flag_val in sf_cmd.items():
            if sf.get(flag_name, False) != flag_val:
                goal_reached = False
                goal_reasons.append(f"state flag {flag_name} != {not flag_val}")

        if not goal_reached:
            goal_reason = "; ".join(goal_reasons)
            fb_out.update(goal_reached=False, goal_reason=goal_reason)
            if fb_out.changed("goal_reason"):
                self.logger.debug(f"{self}:  Goal not reached: {goal_reason}")
        return fb_out

    state_bits = {
        # 'START': None,  # Initial state; no status words
        "NOT READY TO SWITCH ON": [0x4F, 0x00],
        "SWITCH ON DISABLED": [0x4F, 0x40],
        "READY TO SWITCH ON": [0x6F, 0x21],
        "SWITCHED ON": [0x6F, 0x23],
        "OPERATION ENABLED": [0x6F, 0x27],
        "FAULT REACTION ACTIVE": [0x4F, 0x0F],
        "FAULT": [0x4F, 0x08],
        "QUICK STOP ACTIVE": [0x6F, 0x07],
    }

    # ------- Command in -------

    # Incoming goals from above:  param_name : inital_value
    command_in_defaults = dict(
        state="SWITCH ON DISABLED",
        control_mode=DEFAULT_CONTROL_MODE,
        control_word_flags=dict(),
        state_flags=dict(),  # Required, even if not commanded
    )

    # ------- Command out -------

    goal_paths = {
        # These dicts map the current state to [next state,
        # transition] to arrive at some goal state.  When the
        # transition is `None`, the final state has been reached.
        # Some transitions happen automatically; those are marked with
        # a `None` value in the `transitions` dict.
        #
        # Note that these all show QUICK STOP ACTIVE transition 12 to
        # SWITCH ON DISABLED; drive settings also allow transition 16
        # to OPERATION ENABLED; this can be overridden in drive
        # subclasses
        "SWITCHED ON": {
            # Drives in OPERATION ENABLED move to QUICK STOP ACTIVE
            "OPERATION ENABLED": ["QUICK STOP ACTIVE", "TRANSITION_11"],
            "QUICK STOP ACTIVE": ["SWITCH ON DISABLED", "TRANSITION_12"],
            # Transition other drives to SWITCHED ON
            "START": ["NOT READY TO SWITCH ON", "TRANSITION_0"],
            "NOT READY TO SWITCH ON": ["SWITCH ON DISABLED", "TRANSITION_1"],
            "SWITCH ON DISABLED": ["READY TO SWITCH ON", "TRANSITION_2"],
            "READY TO SWITCH ON": ["SWITCHED ON", "TRANSITION_3"],
            "SWITCHED ON": ["SWITCHED ON", None],  # End state
            "FAULT": ["SWITCH ON DISABLED", "TRANSITION_15"],
            "FAULT REACTION ACTIVE": ["FAULT", "TRANSITION_14"],
        },
        "OPERATION ENABLED": {
            # Drives transition to OPERATION ENABLED; note the
            # Hal402Mgr always brings drives to SWITCHED ON state
            # first before setting OPERATION ENABLED goal state
            "SWITCHED ON": ["OPERATION ENABLED", "TRANSITION_4"],
            "OPERATION ENABLED": ["OPERATION ENABLED", None],  # End
            "START": ["START", "TRANSITION_0"],
            "NOT READY TO SWITCH ON": ["SWITCH ON DISABLED", "TRANSITION_1"],
            "SWITCH ON DISABLED": ["READY TO SWITCH ON", "TRANSITION_2"],
            "READY TO SWITCH ON": ["SWITCHED ON", "TRANSITION_3"],
            "FAULT": ["SWITCH ON DISABLED", "TRANSITION_15"],
            "FAULT REACTION ACTIVE": ["FAULT", "TRANSITION_14"],
            "QUICK STOP ACTIVE": ["SWITCH ON DISABLED", "TRANSITION_12"],
        },
        # These tr'ns take longer from OPERATION ENABLED -> SWITCH ON DISABLED
        # 'OPERATION ENABLED':        ['SWITCHED ON', 'TRANSITION_5'],
        # 'SWITCHED ON':              ['READY TO SWITCH ON', 'TRANSITION_6'],
        # 'READY TO SWITCH ON':       ['SWITCH ON DISABLED', 'TRANSITION_7']
        "SWITCH ON DISABLED": {
            "START": ["NOT READY TO SWITCH ON", "TRANSITION_0"],
            "NOT READY TO SWITCH ON": ["SWITCH ON DISABLED", "TRANSITION_1"],
            "SWITCH ON DISABLED": ["SWITCH ON DISABLED", None],  # End State
            "READY TO SWITCH ON": ["SWITCH ON DISABLED", "TRANSITION_7"],
            "SWITCHED ON": ["SWITCH ON DISABLED", "TRANSITION_10"],
            "FAULT REACTION ACTIVE": ["FAULT", "TRANSITION_14"],
            "FAULT": ["SWITCH ON DISABLED", "TRANSITION_15"],
            "OPERATION ENABLED": ["QUICK STOP ACTIVE", "TRANSITION_11"],
            "QUICK STOP ACTIVE": ["SWITCH ON DISABLED", "TRANSITION_12"],
        },
        # Fault state has three possible final states; see inline notes
        "FAULT": {
            # Drives in FAULT state remain in that state
            "FAULT REACTION ACTIVE": ["FAULT", "TRANSITION_14"],
            "FAULT": ["FAULT", None],  # End state
            # Drives in OPERATION ENABLED quick stop & disable
            "OPERATION ENABLED": ["QUICK STOP ACTIVE", "TRANSITION_11"],
            "QUICK STOP ACTIVE": ["SWITCH ON DISABLED", "TRANSITION_12"],
            # Drives in all other states transition to SWITCH ON DISABLED
            "START": ["NOT READY TO SWITCH ON", "TRANSITION_0"],
            "NOT READY TO SWITCH ON": ["SWITCH ON DISABLED", "TRANSITION_1"],
            "SWITCH ON DISABLED": ["SWITCH ON DISABLED", None],  # End state
            "READY TO SWITCH ON": ["SWITCH ON DISABLED", "TRANSITION_7"],
            "SWITCHED ON": ["SWITCH ON DISABLED", "TRANSITION_10"],
        },
    }

    command_out_data_types = dict(
        control_word="uint16",
        control_mode="int8",
    )

    command_out_defaults = dict(
        control_word=0x0000,
        control_mode=DEFAULT_CONTROL_MODE,
    )

    def set_command(self, **kwargs):
        cmd_out = super().set_command(**kwargs)
        if not self.feedback_in.get("oper"):
            cmd_out.update(**self.command_out_defaults)
            return cmd_out
        cmd_out.update(
            # Command sent to device
            control_word=self._get_next_control_word(),
            control_mode=self._get_next_control_mode(),
        )
        return cmd_out

    def _get_next_control_word(self):
        # Get base control word
        if self._get_next_transition() is None:
            # Holding current state
            control_word = self._get_hold_state_control_word()
        else:
            # Transitioning to next state
            control_word = self._get_transition_control_word()

        # Add flags and return
        flags = self.command_in.get("control_word_flags")
        return self._add_control_word_flags(control_word, **flags)

    # Map drive states to control word that maintains the state.
    # `None` indicates hold state N/A in automatic transition states
    # where control word is ignored
    hold_state_control_word = {
        "START": None,
        "NOT READY TO SWITCH ON": None,
        "SWITCH ON DISABLED": 0x0000,
        "READY TO SWITCH ON": 0x0006,
        "SWITCHED ON": 0x0007,
        "OPERATION ENABLED": 0x000F,
        "QUICK STOP ACTIVE": None,
        "FAULT REACTION ACTIVE": None,
        "FAULT": 0x0000,  # Anything but 0x0080 will hold state
    }

    def _get_hold_state_control_word(self):
        control_word = self.hold_state_control_word[
            self.feedback_out.get("state")
        ]
        if control_word is None:
            raise ValueError(
                f"BUG:  No hold state cw for {self.feedback_out.get('state')}"
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

    # Control word bits not used for CiA402 state machine operation
    # may have other purposes
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

    def _get_transition_control_word(self):
        # Look up next transition and return control word to effect it
        transition = self._get_next_transition()
        if transition is None:
            # Goal state reached; shouldn't be here
            raise ValueError(
                "BUG:  No transition control word when goal reached"
            )

        cw = self.transitions[transition]
        if cw is None:
            # Current state is in one of the four automatic
            # transitions.  In all four cases, 0x0000 works to hold
            # the destination state:
            #
            # - Transitions 0 and 1 -> SWITCH ON DISABLED:  hold state
            #   control word is 0x0000
            #
            # - Transitions 13 and 14 -> FAULT:  hold state control
            #   word is anything BUT 0x0080
            cw = 0x0000
        return cw

    def _get_next_transition(self, curr_state=None):
        return self._get_next_state(curr_state=curr_state, transition=True)

    def _get_next_state(self, curr_state=None, transition=False):
        gp = self.goal_paths[self.command_in.get("state")]
        return gp[curr_state or self.feedback_out.get("state")][transition]

    @classmethod
    def _add_control_word_flags(cls, control_word, **flags):
        # Add mode-specific flags
        for flag, val in flags.items():
            operand = 1 << cls.cw_extra_bits[flag]
            if val:
                control_word |= operand
            else:
                control_word &= ~operand & 0xFFFF

        return control_word

    def _get_next_control_mode(self):
        # Get control_mode from command_in; translate e.g. "MODE_CSP" to 8
        cm = self.command_in.get("control_mode")
        return self.control_mode_int(cm)


class CiA402SimDevice(CiA402Device, CiA301SimDevice):
    """
    Manage a simulated CiA 402 motor drive's state machine.

    The `set_sim_feedback()` method will crudely mimic behavior of a
    real drive.
    """

    sim_feedback_data_types = CiA402Device.feedback_in_data_types
    sim_feedback_defaults = CiA402Device.feedback_in_defaults

    # ------- Sim feedback -------

    def set_sim_feedback(self):
        sfb = super().set_sim_feedback()
        control_word = self.command_out.get("control_word")

        if not self.feedback_in.get("oper"):
            sfb.update(**self.sim_feedback_defaults)
            if sfb.get("online"):
                sw = self._add_status_word_flags(0x0000, VOLTAGE_ENABLED=True)
                sfb.update(status_word=sw)
            return sfb

        # State
        masked_cw = control_word & 0x008F
        state = self.feedback_out.get("state")  # No change by default
        if self.hold_state_control_word[state] is None:
            # Automatic transitions ignore control word
            state = self._get_next_state()
        else:
            for test_cw, test_states, next_state in self.sim_drive_states:
                if masked_cw == test_cw and (
                    test_states is None
                    or self.feedback_out.get("state") in test_states
                ):
                    state = next_state
                    break

        # Status word (incl. flags)
        status_word = 0x0000 if state == "START" else self.state_bits[state][1]
        cw_flags = self._get_control_word_flags(control_word)
        sw_flags = dict(
            HOMING_COMPLETED=cw_flags["OPERATION_MODE_SPECIFIC_1"],
            VOLTAGE_ENABLED=sfb.get("online"),
        )
        status_word = self._add_status_word_flags(status_word, **sw_flags)

        # Control mode
        control_mode = (
            0 if state == "START" else self.command_out.get("control_mode")
        )

        sfb.update(
            status_word=self.data_types.uint16(status_word),
            control_mode_fb=control_mode,
        )

        if self.feedback_in.get("oper"):
            # Log changes
            if self.sim_feedback.changed("control_mode_fb"):
                cm = self.sim_feedback.get("control_mode_fb")
                self.logger.info(f"{self} sim control_mode_fb:  0x{cm:04X}")
            if self.sim_feedback.changed("status_word"):
                sw = self.sim_feedback.get("status_word")
                flags = ",".join(k for k, v in sw_flags.items() if v)
                flags = f" flags:  {flags}" if flags else ""
                self.logger.info(
                    f"{self} sim status_word:  0x{sw:04X} {state} {flags}"
                )

        return sfb

    sim_drive_states = [
        # (test_cw, [test_state, ...], next_state)
        #
        # Order matters!  Non-matches against test_cw and test_state
        # will fall through; None matches any starting state
        (0x0080, ["FAULT"], "SWITCH ON DISABLED"),
        (0x0000, ["FAULT"], "FAULT"),
        (0x0000, None, "SWITCH ON DISABLED"),
        (0x0002, ["FAULT"], "FAULT"),
        (0x0002, ["OPERATION ENABLED"], "QUICK STOP ACTIVE"),
        # (I forgot where I read quick stop triggers these
        # transitions)
        (0x0002, None, "SWITCH ON DISABLED"),
        (
            0x0006,
            [
                "SWITCH ON DISABLED",
                "SWITCHED ON",
                "OPERATION ENABLED",
            ],
            "READY TO SWITCH ON",
        ),
        (
            0x0007,
            [
                "READY TO SWITCH ON",
                "OPERATION ENABLED",
            ],
            "SWITCHED ON",
        ),
        (
            0x000F,
            ["SWITCHED ON", "QUICK STOP ACTIVE"],
            "OPERATION ENABLED",
        ),
    ]

    @classmethod
    def _add_status_word_flags(cls, status_word, **flags):
        # Add flags by name to status word; used in set_sim_feedback
        for flag, val in flags.items():
            operand = 1 << cls.sw_extra_bits[flag]
            if val:
                status_word |= operand
            else:
                status_word &= ~operand & 0xFFFF

        return status_word

    @classmethod
    def _get_control_word_flags(cls, control_word):
        # Get flags by name from control word; used in set_sim_feedback
        flags = dict()
        for name, bitnum in cls.cw_extra_bits.items():
            flags[name] = bool(control_word & 1 << bitnum)
        return flags
