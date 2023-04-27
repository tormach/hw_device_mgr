from ..cia_301.device import (
    CiA301Device,
    CiA301DataType,
    CiA301SimDevice,
)
from ..errors.device import ErrorDevice, ErrorSimDevice
from functools import lru_cache


class CiA402Device(CiA301Device, ErrorDevice):
    """
    Manage a CiA 402 motor drive's state machine.

    This device will attempt to step the drive through the CiA 402
    state machine to the goal `state` (e.g. `OPERATION ENABLED`), and
    will manage other goals.

    Command parameters:
    - `state`:  The CiA 402 goal state
    - `control_mode`:  Drive control mode, e.g. `MODE_CSP`
    - `home_request`:  Command homing operation (HM mode)
    - `move_request`:  Command move operation (PP mode)
    - `relative_target`:  Relative vs Absolute move operation (PP mode)

    Feedback parameters:
    - `home_success`:  Drive completed homing successfully
    - `home_error`:  Drive reports homing error
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

    have_sto = False

    home_timeout = 15  # seconds
    move_timeout = 15  # seconds

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
    sw_bits = dict(
        READY_TO_SWITCH_ON=0,  # (CiA402)
        SWITCH_ON=1,  # (CiA402)
        OPERATION_ENABLED=2,  # (CiA402)
        FAULT=3,  # (CiA402)
        VOLTAGE_ENABLED=4,
        QUICK_STOP_ACTIVE=5,  # (CiA402)
        SWITCH_ON_DISABLED=6,  # (CiA402)
        WARNING=7,
        MANUFACTURER_SPECIFIC_1=8,
        REMOTE=9,
        TARGET_REACHED=10,
        INTERNAL_LIMIT_ACTIVE=11,
        OPERATION_MODE_SPECIFIC_1=12,  # HM=HOMING_ATTAINED, PP=SETPOINT_ACK
        OPERATION_MODE_SPECIFIC_2=13,  # HM=HOMING_ERROR; others=FOLLOWING_ERROR
        MANUFACTURER_SPECIFIC_2=14,
        MANUFACTURER_SPECIFIC_3=15,
    )

    # Incoming feedback from drives:  param_name : data_type
    feedback_in_data_types = dict(
        status_word="uint16",
        control_mode_fb="int8",
        sto="bit",
    )

    # Incoming feedback from drives:  param_name : inital_value
    feedback_in_defaults = dict(
        status_word=0,
        control_mode_fb=0,
        sto=False,
    )

    # Outgoing feedback to controller:  param_name : inital_value
    feedback_out_data_types = dict(
        **feedback_in_data_types,
        state="str",
        transition="int8",
        home_success="bit",
        home_error="bit",
        move_success="bit",
        move_error="bit",
    )
    feedback_out_defaults = dict(
        **feedback_in_defaults,
        state="START",
        transition=-1,
        home_success=False,
        home_error=False,
        move_success=False,
        move_error=False,
    )

    log_status_word_changes = True

    @classmethod
    def test_sw_bit(cls, word, name):
        return bool(word & (1 << cls.sw_bits[name]))

    @classmethod
    def test_cw_bit(cls, word, name):
        return bool(word & (1 << cls.cw_bits[name]))

    def get_feedback_hm(self, sw):
        # Control mode is HM
        if not self.command_in.get("home_request"):
            # Home command cleared; reset feedback and return success result
            self.feedback_out.update(home_success=False, home_error=False)
            return True, None

        # Home command in effect
        if self.feedback_out.get("state") != "OPERATION ENABLED":
            fault_desc = "Home request while drive not enabled"
            self.feedback_out.update(
                home_success=False,
                home_error=True,
                fault=True,
                fault_desc=fault_desc,
            )
            return False, fault_desc

        success, error, reason = False, False, None
        if self.test_sw_bit(sw, "OPERATION_MODE_SPECIFIC_2"):
            # HOMING_ERROR bit set
            error = True
            reason = "homing error"
            self.feedback_out.update(fault=True, fault_desc=reason)
        elif self.test_sw_bit(sw, "OPERATION_MODE_SPECIFIC_1"):
            # HOMING_ATTAINED bit set
            success = True
        else:
            reason = "homing not complete"

        self.feedback_out.update(home_success=success, home_error=error)
        return success, reason

    def get_feedback_pp(self, sw):
        # Control mode is PP
        if not self.command_in.get("move_request"):
            self.feedback_out.update(move_success=False, move_error=False)
            return True, None
        if self.feedback_out.get("state") != "OPERATION ENABLED":
            reason = "Move request while drive not enabled"
            self.feedback_out.update(
                move_success=False,
                move_error=True,
                fault=True,
                fault_desc=reason,
            )
            return False, reason

        success, error, reason = False, False, None
        if self.test_sw_bit(sw, "TARGET_REACHED"):
            # done bit set
            success = True
        else:
            reason = "move not complete"

        self.feedback_out.update(move_success=success, move_error=error)
        return success, reason

    def get_feedback_sto(self):
        # Process active STO:  Raise fault on OPERATION ENABLED command
        if not self.feedback_in.get("sto"):
            # STO inactive (low)
            if self.feedback_in.changed("sto"):  # Log once
                self.logger.info("STO input inactive")
            return True, None

        # STO active (high)
        state_cmd = self.command_in.get("state")
        if state_cmd in ("SWITCHED ON", "OPERATION ENABLED"):
            # ENABLE_OPERATION command in effect; treat as fault (This fault
            # condition will last just one update; top-level manager will react
            # by latching its own fault, which will stop the ENABLE_OPERATION
            # command)
            msg = "STO input active during enable command"
            self.feedback_out.update(fault=True, fault_desc=msg)
            if self.feedback_in.changed("sto"):  # Log once
                self.logger.error(f"{msg} (fault)")
            return False, f"{msg} (fault)"
        else:
            # No ENABLE_OPERATION command; log (warning, once) and return
            if self.feedback_in.changed("sto"):
                self.logger.warning("STO input active")
            return True, None

    @property
    def goal_reached_timeout(self):
        """Increase goal_reached timeout during homing or PP moves."""
        if self.command_in.get("home_request"):
            return self.home_timeout
        if self.command_in.get("move_request"):
            return self.move_timeout
        return super().goal_reached_timeout

    def get_feedback(self):
        fb_out = super().get_feedback()
        fb_in = self.feedback_in

        # If device not operational, set default "START" state
        if not fb_out.get("oper"):
            fb_out.update(**self.feedback_out_defaults)
            return fb_out

        # Goal reached, fault var defaults
        goal_reached = True
        goal_reasons = list()
        fault = False
        fault_desc = ""

        # Status word, control mode from fb in
        sw = fb_in.get("status_word")
        cm = fb_in.get("control_mode_fb")
        fb_out.update(status_word=sw, control_mode_fb=cm)
        cm_cmd = self.command_in.get("control_mode")
        if cm != self.MODE_HM and cm != cm_cmd:
            goal_reached = False
            cm_str = self.control_mode_str(cm)
            cm_cmd_str = self.control_mode_str(cm_cmd)
            goal_reasons.append(f"control_mode {cm_str} != {cm_cmd_str}")

        # Calculate 'state' feedback
        for state, bits in self.state_bits.items():
            # Compare masked status word with pattern to determine current state
            sw_mask, sw_pat = bits
            if sw & sw_mask == sw_pat:
                fb_out.update(state=state)
                break
        else:
            fault = True
            fault_desc = (
                f"Unknown status word 0x{sw:X}; "
                f"state {fb_out.get('state')} unchanged"
            )
        if self._get_next_transition() >= 0:
            goal_reached = False
            state_cmd = self.command_in.get("state")
            sw = fb_in.get("status_word")
            goal_reasons.append(f"state {state} != {state_cmd}")
            if state_cmd in (
                "SWITCHED ON",
                "OPERATION ENABLED",
            ) and not self.test_sw_bit(sw, "VOLTAGE_ENABLED"):
                fault = True
                fault_desc = "Enable command while no voltage at motor"
                goal_reasons.append(fault_desc)

        # Raise fault if device unexpectedly goes offline
        if self.command_in.get(
            "state"
        ) == "OPERATION ENABLED" and not self.test_sw_bit(
            sw, "READY_TO_SWITCH_ON"
        ):
            fault = True
            fault_desc = "Enabled drive unexpectedly disabled"
            goal_reasons.append(fault_desc)

        # Calculate 'transition' feedback
        new_st, old_st = fb_out.changed("state", return_vals=True)
        if (old_st, new_st) == ("START", "NOT READY TO SWITCH ON"):
            fb_out.update(transition=0)
        elif new_st == "FAULT REACTION ACTIVE":
            # Fault will be interpreted below when SW FAULT bit tested
            fb_out.update(transition=13)
        elif self._get_next_state(curr_state=old_st) == new_st:
            next_trans = self._get_next_transition(curr_state=old_st)
            fb_out.update(transition=next_trans)
        else:
            fb_out.update(transition=-1)

        # Mode-specific functions
        if cm == self.MODE_HM:
            # Calculate homing status
            hm_success, hm_reason = self.get_feedback_hm(sw)
            if not hm_success:
                goal_reached = False
                goal_reasons.append(hm_reason)
        elif cm == self.MODE_PP:
            # Calculate move status
            pp_success, pp_reason = self.get_feedback_pp(sw)
            if not pp_success:
                goal_reached = False
                goal_reasons.append(pp_reason)

        # Handle STO
        if self.have_sto:
            sto_success, sto_reason = self.get_feedback_sto()
            if not sto_success:
                goal_reached = False
                goal_reasons.append(sto_reason)

        # Fault reported by drive
        if self.test_sw_bit(sw, "FAULT"):
            fault = True
            if fb_out.get("error_code"):
                error_code = fb_out.get("error_code")
                error_desc = fb_out.get("description")
                fault_desc = f"{error_code} {error_desc}"
            else:
                fault_desc = "Fault (no error code)"
            goal_reasons.append(fault_desc)

        # If in CiA402 FAULT state, set device fault
        if self.command_in.get("state") == "FAULT":
            fault = True
            if not fault_desc:
                # Recycle previous description if possible
                fault_desc = fb_out.get_old("fault_desc")
                # If FAULT is commanded & no device fault, this will be an empty
                # string
                if not fault_desc:
                    fault_desc = f"FAULT command from controller (was {old_st})"
                goal_reasons.append(fault_desc)

        # Update feedback to controller
        if fault:
            fb_out.update(fault=True, fault_desc=fault_desc)

        if self.log_status_word_changes and fb_out.changed("status_word"):
            self.logger.info(f"status_word:  {self.sw_to_str(sw)}")

        if not goal_reached:
            goal_reason = "; ".join(goal_reasons)
            fb_out.update(goal_reached=False, goal_reason=goal_reason)
            if fb_out.changed("goal_reason"):
                self.logger.info(f"Goal not reached: {goal_reason}")
        elif fb_out.changed("goal_reached"):  # Goal just now reached
            self.logger.info("Goal reached")
        return fb_out

    @classmethod
    @lru_cache
    def sw_to_str(cls, sw):
        # Decode status word
        for state, bits in cls.state_bits.items():
            # Compare masked status word with pattern to determine current state
            sw_mask, sw_pat = bits
            if sw & sw_mask == sw_pat:
                break
        else:
            state = "Unknown state"
        flags = [
            k
            for k, v in cls.sw_bits.items()
            if not sw_mask & (1 << v) and cls.test_sw_bit(sw, k)
        ]
        flags = ",".join(flags) if flags else "(none)"
        sw = cls.data_types.uint16(sw)  # For formatting
        return f"{sw} {state} flags: {flags}"

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
        home_request=False,
        move_request=False,
        relative_target=False,
    )
    command_in_data_types = dict(
        state="str",
        control_mode="int8",
        home_request="bit",
        move_request="bit",
        relative_target="bit",
    )

    # ------- Command out -------

    goal_paths = {
        # These dicts map the current state to [next state,
        # transition] to arrive at some goal state.  When the
        # transition is -1, the final state has been reached.
        # Some transitions happen automatically; those are marked with
        # a `None` value in the `transitions` dict.
        #
        # Note that these all show QUICK STOP ACTIVE transition 12 to
        # SWITCH ON DISABLED; drive settings also allow transition 16
        # to OPERATION ENABLED; this can be overridden in drive
        # subclasses
        "SWITCHED ON": {
            # Drives in OPERATION ENABLED move to QUICK STOP ACTIVE
            "OPERATION ENABLED": ["QUICK STOP ACTIVE", 11],
            "QUICK STOP ACTIVE": ["SWITCH ON DISABLED", 12],
            # Transition other drives to SWITCHED ON
            "START": ["NOT READY TO SWITCH ON", 0],
            "NOT READY TO SWITCH ON": ["SWITCH ON DISABLED", 1],
            "SWITCH ON DISABLED": ["READY TO SWITCH ON", 2],
            "READY TO SWITCH ON": ["SWITCHED ON", 3],
            "SWITCHED ON": ["SWITCHED ON", -1],  # End state
            "FAULT": ["SWITCH ON DISABLED", 15],
            "FAULT REACTION ACTIVE": ["FAULT", 14],
        },
        "OPERATION ENABLED": {
            # Drives transition to OPERATION ENABLED; note the
            # Hal402Mgr always brings drives to SWITCHED ON state
            # first before setting OPERATION ENABLED goal state
            "SWITCHED ON": ["OPERATION ENABLED", 4],
            "OPERATION ENABLED": ["OPERATION ENABLED", -1],  # End
            "START": ["START", 0],
            "NOT READY TO SWITCH ON": ["SWITCH ON DISABLED", 1],
            "SWITCH ON DISABLED": ["READY TO SWITCH ON", 2],
            "READY TO SWITCH ON": ["SWITCHED ON", 3],
            "FAULT": ["SWITCH ON DISABLED", 15],
            "FAULT REACTION ACTIVE": ["FAULT", 14],
            "QUICK STOP ACTIVE": ["SWITCH ON DISABLED", 12],
        },
        # These tr'ns take longer from OPERATION ENABLED -> SWITCH ON DISABLED
        # 'OPERATION ENABLED':        ['SWITCHED ON', 5],
        # 'SWITCHED ON':              ['READY TO SWITCH ON', 6],
        # 'READY TO SWITCH ON':       ['SWITCH ON DISABLED', 7]
        "SWITCH ON DISABLED": {
            "START": ["NOT READY TO SWITCH ON", 0],
            "NOT READY TO SWITCH ON": ["SWITCH ON DISABLED", 1],
            "SWITCH ON DISABLED": ["SWITCH ON DISABLED", -1],  # End State
            "READY TO SWITCH ON": ["SWITCH ON DISABLED", 7],
            "SWITCHED ON": ["SWITCH ON DISABLED", 10],
            "FAULT REACTION ACTIVE": ["FAULT", 14],
            "FAULT": ["SWITCH ON DISABLED", 15],
            "OPERATION ENABLED": ["QUICK STOP ACTIVE", 11],
            "QUICK STOP ACTIVE": ["SWITCH ON DISABLED", 12],
        },
        # Fault state has three possible final states; see inline notes
        "FAULT": {
            # Drives in FAULT state remain in that state
            "FAULT REACTION ACTIVE": ["FAULT", 14],
            "FAULT": ["FAULT", -1],  # End state
            # Drives in OPERATION ENABLED quick stop & disable
            "OPERATION ENABLED": ["QUICK STOP ACTIVE", 11],
            "QUICK STOP ACTIVE": ["SWITCH ON DISABLED", 12],
            # Drives in all other states transition to SWITCH ON DISABLED
            "START": ["NOT READY TO SWITCH ON", 0],
            "NOT READY TO SWITCH ON": ["SWITCH ON DISABLED", 1],
            "SWITCH ON DISABLED": ["SWITCH ON DISABLED", -1],  # End state
            "READY TO SWITCH ON": ["SWITCH ON DISABLED", 7],
            "SWITCHED ON": ["SWITCH ON DISABLED", 10],
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
        self._get_next_control_mode(cmd_out)
        self._get_next_control_word(cmd_out)
        return cmd_out

    def _check_hm_request(self):
        # Check for home request
        home_request = False
        if self.command_in.get("home_request"):
            if self.command_in.changed("home_request"):
                self.logger.info("Homing operation requested")
            if self.feedback_out.get("control_mode_fb") == self.MODE_HM:
                # Don't actually set HOMING_START until in MODE_HM
                home_request = True
        elif self.command_in.changed("home_request"):  # home_request cleared
            self.logger.info("Homing operation complete")
        return home_request

    def _check_pp_request(self):
        # Check for move request
        move_request = False
        relative_target = False
        if self.command_in.get("move_request"):
            if self.command_in.changed("move_request"):
                self.logger.info("Move operation requested")
                move_request = True
                if self.command_in.get("relative_target"):
                    self.logger.info("Target position is relative")
                    relative_target = True
        else:
            # Clear move request unless setpoint ack not set after previous new
            # set point
            cw = self.command_out.get_old("control_word")
            prev_nsp = self.test_cw_bit(cw, "OPERATION_MODE_SPECIFIC_1")
            sw = self.feedback_in.get("status_word")
            setpoint_ack = self.test_sw_bit(sw, "OPERATION_MODE_SPECIFIC_1")
            move_request = prev_nsp and not setpoint_ack
            if self.command_in.changed("move_request"):  # move_request cleared
                self.logger.info("Move operation request cleared")
        return move_request, relative_target

    @classmethod
    @lru_cache
    def cw_to_str(cls, cw):
        # Decode control word
        mask = 0x008F  # Mask for bits relevant to command
        cmd = cls.cw_to_cmd_map().get(cw & mask, "Unknown command")
        flags = [
            k  # Names of set bits not part of the CiA402 command
            for k, v in cls.cw_bits.items()
            if not (1 << v) & mask and cw & (1 << v)
        ]
        flags = ",".join(flags) if flags else "(none)"
        cw = cls.data_types.by_shared_name("uint16")(cw)  # Format
        return f"{cw} {cmd} flags: {flags}"

    def _get_next_control_word(self, cmd_out):
        # Get base control word
        if self._get_next_transition() < 0:
            # Holding current state
            control_word = self._get_hold_state_control_word()
        else:
            # Transitioning to next state
            control_word = self._get_transition_control_word()

        # Add flags and return
        next_cm = cmd_out.get("control_mode")
        operation_mode_specific_3 = False
        # operation mode specific 3 sets the target to relative position
        # when in PP mode
        if next_cm == self.MODE_HM:
            operation_mode_specific_1 = self._check_hm_request()
        elif next_cm == self.MODE_PP:
            (
                operation_mode_specific_1,
                operation_mode_specific_3,
            ) = self._check_pp_request()
        else:
            operation_mode_specific_1 = False
        next_cw = self._add_control_word_flags(
            control_word,
            OPERATION_MODE_SPECIFIC_1=operation_mode_specific_1,
            OPERATION_MODE_SPECIFIC_3=operation_mode_specific_3,
        )
        cmd_out.update(control_word=next_cw)
        if cmd_out.changed("control_word"):
            cw_str = self.cw_to_str(next_cw)
            self.logger.info(f"control_word:  {cw_str}")

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

    @classmethod
    @lru_cache
    def cw_to_cmd_map(cls):
        res = {
            v: k
            for k, v in cls.hold_state_control_word.items()
            if v is not None and k != "FAULT"
        }
        res[0x0002] = "QUICK STOP ACTIVE"
        res[0x0080] = "CLEAR FAULT"
        return res

    def _get_hold_state_control_word(self):
        state = self.feedback_out.get("state")
        control_word = self.hold_state_control_word[state]
        if control_word is None:
            raise ValueError(
                f"BUG:  No hold state cw for {self.feedback_out.get('state')}"
            )
        return control_word

    # Map transition to the control word that would effect the
    # transition; None indicates automatic transition where control
    # word is ignored
    transitions = {
        0: None,  # START->NOT READY TO SWITCH ON
        1: None,  # NOT READY TO SWITCH ON->SWITCH ON DISABLED
        2: 0x0006,  # SWITCH ON DISABLED->READY TO SWITCH ON
        3: 0x0007,  # READY TO SWITCH ON->SWITCHED ON
        4: 0x000F,  # SWITCHED ON->OPERATION ENABLED
        5: 0x0007,  # OPERATION ENABLED->SWITCHED ON
        6: 0x0006,  # SWITCHED ON->READY TO SWITCH ON
        7: 0x0000,  # READY TO SWITCH ON->SWITCH ON DISABLED
        8: 0x0006,  # OPERATION ENABLED->READY TO SWITCH ON
        9: 0x0000,  # OPERATION ENABLED->SWITCH ON DISABLED
        10: 0x0000,  # SWITCHED ON->SWITCH ON DISABLED
        11: 0x0002,  # OPERATION ENABLED->QUICK STOP ACTIVE
        12: 0x0000,  # QUICK STOP ACTIVE->SWITCH ON DISABLED *
        13: None,  # (Any)->FAULT REACTION ACTIVE
        14: None,  # FAULT REACTION ACTIVE->FAULT
        15: 0x0080,  # FAULT->SWITCH ON DISABLED **
        16: 0x000F,  # QUICK STOP ACTIVE->OPERATION ENABLED *
        # * Transitions 12, 16:  Set 605Ah "Quick stop option code" value:
        # - 0:    coast to stop; automatic transition 12
        # - 1-3:  stop @ 2007-10h torque; automatic transition 12
        # - 5-7:  stop @ 2007-10h torque; hold; then can transition 12 or 16
        # ** Transition 15:  Fault cleared on rising edge of bit 7
    }

    # Control word bits 0-3,7 control CiA402 state machine transitions.
    cw_bits = dict(
        SWITCH_ON=0,  # (state machine)
        ENABLE_VOLTAGE=1,  # (state machine)
        QUICK_STOP=2,  # (state machine)
        ENABLE_OPERATION=3,  # (state machine) AKA S-ON
        OPERATION_MODE_SPECIFIC_1=4,  # HM=HOMING_START; PP=NEW_SETPOINT
        OPERATION_MODE_SPECIFIC_2=5,  # PP=CHANGE_SET_IMMEDIATE
        OPERATION_MODE_SPECIFIC_3=6,
        FAULT_RESET=7,  # (state machine)
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
        if transition < 0:
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
            operand = 1 << cls.cw_bits[flag]
            if val:
                control_word |= operand
            else:
                control_word &= ~operand & 0xFFFF

        return control_word

    def _get_next_control_mode(self, cmd_out):
        if self.command_in.get("home_request"):
            # If `home_request` is set, command homing mode
            next_cm = self.MODE_HM
        else:
            # Otherwise, copy control_mode from command_in
            next_cm = self.command_in.get("control_mode")

        cmd_out.update(control_mode=next_cm)
        if cmd_out.changed("control_mode"):
            cm_str = self.control_mode_str(next_cm)
            self.logger.info(f"control_mode:  {cm_str}")


class CiA402SimDevice(CiA402Device, CiA301SimDevice, ErrorSimDevice):
    """
    Manage a simulated CiA 402 motor drive's state machine.

    The `set_sim_feedback()` method will crudely mimic behavior of a
    real drive.
    """

    # Incoming feedback from drives
    feedback_in_data_types = dict(
        position_cmd="float",
        position_fb="float",
    )
    feedback_in_defaults = dict(
        position_cmd=0.0,
        position_fb=0.0,
    )

    feedback_out_data_types = dict(**feedback_in_data_types)
    feedback_out_defaults = dict(**feedback_in_defaults)

    sim_feedback_data_types = dict(
        **CiA402Device.feedback_in_data_types,
        **feedback_in_data_types,
    )
    sim_feedback_defaults = dict(
        **CiA402Device.feedback_in_defaults,
        **feedback_in_defaults,
    )

    # diff. btw. pos. cmd + fb to signal target reached
    position_goal_tolerance = 0.01

    # ------- Sim feedback -------

    def set_sim_feedback_hm(self, cw):
        # In MODE_HM, cw OPERATION_MODE_SPECIFIC_1 is HOMING_START cmd, sw
        # OPERATION_MODE_SPECIFIC_1 is HOMING_ATTAINED fb
        if self.test_cw_bit(cw, "OPERATION_MODE_SPECIFIC_1"):
            return dict(OPERATION_MODE_SPECIFIC_1=True)
        else:
            return dict()

    def target_reached(self, sw, cw):
        fb_in = self.interface("feedback_in")
        setpoint_ack = self.test_sw_bit(sw, "OPERATION_MODE_SPECIFIC_1")
        new_setpoint = self.test_cw_bit(cw, "OPERATION_MODE_SPECIFIC_1")
        if new_setpoint or setpoint_ack:
            # Pretend we haven't reached new target before it's even set
            return False
        dtg = abs(fb_in.get("position_cmd") - fb_in.get("position_fb"))
        return dtg < self.position_goal_tolerance

    def set_sim_feedback_pp(self, cw, sw):
        # In MODE_PP, cw OPERATION_MODE_SPECIFIC_1 is NEW_SETPOINT cmd, sw
        # OPERATION_MODE_SPECIFIC_1 is SETPOINT_ACKNOWLEDGE fb
        if self.test_cw_bit(cw, "OPERATION_MODE_SPECIFIC_1"):
            # If cw NEW_SETPOINT is set, then set sw SETPOINT_ACKNOWLEDGE
            return dict(OPERATION_MODE_SPECIFIC_1=True)
        elif self.target_reached(sw, cw):
            # Target reached when target position reached
            return dict(TARGET_REACHED=True)
        else:
            return dict()

    def set_sim_feedback(self):
        sw_prev = self.sim_feedback.get("status_word")
        sfb = super().set_sim_feedback()
        control_word, cw_prev = self.command_out.changed(
            "control_word", return_vals=True
        )

        if not self.feedback_in.get("online"):
            sfb_updates = self.sim_feedback_defaults.copy()
            sfb_updates["sto"] = False  # Too early to read STO
            return sfb
        if not self.feedback_in.get("oper"):
            sfb_updates = self.sim_feedback_defaults.copy()
            sw = self.add_status_word_flags(0x0000, VOLTAGE_ENABLED=True)
            sfb_updates["status_word"] = sw
            sfb.update(**sfb_updates)
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
                    test_states is None or state in test_states
                ):
                    # Special case:  No motor power before enabling drive
                    if next_state == "READY TO SWITCH ON":
                        sw = self.feedback_in.get("status_word")
                        if not self.test_sw_bit(sw, "VOLTAGE_ENABLED"):
                            break  # Silently stay in SWITCH ON DISABLED state
                    state = next_state
                    break

        # Control mode
        control_mode = (
            0 if state == "START" else self.command_out.get("control_mode")
        )

        # Status word (incl. flags)
        status_word = 0x0000 if state == "START" else self.state_bits[state][1]
        sw_flags = dict(VOLTAGE_ENABLED=sfb.get("online"))
        if self.feedback_out.get("fault"):
            pass  # Don't update mode-specific flags
        elif control_mode == self.MODE_HM:
            sw_flags.update(self.set_sim_feedback_hm(control_word))
        elif control_mode == self.MODE_PP:
            # Test previous cw because target_reached() looks at fb_in, which is
            # set after command_in
            sw_flags.update(self.set_sim_feedback_pp(cw_prev, sw_prev))
        status_word = self.add_status_word_flags(status_word, **sw_flags)

        sfb.update(
            status_word=status_word,
            control_mode_fb=control_mode,
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
    def add_status_word_flags(cls, status_word, **flags):
        # Add flags by name to status word; used in set_sim_feedback
        for flag, val in flags.items():
            operand = 1 << cls.sw_bits[flag]
            if val:
                status_word |= operand
            else:
                status_word &= ~operand & 0xFFFF

        return cls.data_types.uint16(status_word)
