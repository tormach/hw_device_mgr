import pytest
from hal_402_device_mgr.state_machine_402 import StateMachine402


class TestStateMachine402:
    tc = StateMachine402
    patch_mock_rospy = 'hal_402_device_mgr.state_machine_402.rospy'

    @pytest.fixture
    def obj(self, mock_rospy):
        obj = self.tc()
        # Default to online operational
        obj.slave_online = True
        obj.slave_oper = True
        yield obj

    def test_mode_attrs(self):
        modes = dict()
        for mode in [a for a in dir(self.tc) if a.startswith('MODE_')]:
            val = getattr(self.tc, mode)
            assert val not in modes
            assert val >= 0
            assert val < 16
            modes[val] = mode

    # Control word:  16-bit, 0-3,7 for state machine control
    cw_extra_bits_mask = 0xFF70
    # Status word:  16-bits, 0-3,5,6 for state machine status
    sw_extra_bits_mask = 0xFF90

    def test_sw_cw_extra_bits(self):
        cw_extra_bits_sum = 0
        for name, bitnum in self.tc.cw_extra_bits.items():
            cw_extra_bits_sum += 1 << bitnum
        print('cw', hex(cw_extra_bits_sum), hex(self.cw_extra_bits_mask))
        assert cw_extra_bits_sum == self.cw_extra_bits_mask

        sw_extra_bits_sum = 0
        for name, bitnum in self.tc.sw_extra_bits.items():
            sw_extra_bits_sum += 1 << bitnum
        print('sw', hex(sw_extra_bits_sum), hex(self.sw_extra_bits_mask))
        assert sw_extra_bits_sum == self.sw_extra_bits_mask

    def test_set_get_control_mode(self, obj):
        for mode_name in ('MODE_PP', 'MODE_CSP', 'MODE_HM', 'MODE_CST'):
            mode = getattr(self.tc, mode_name, None)
            assert mode is not None
            obj.set_control_mode(mode)
            assert obj.control_mode == mode
            assert obj.get_control_mode() == mode

            obj.set_control_mode(mode_name)
            assert obj.control_mode == mode
            assert obj.get_control_mode() == mode

    def test_states_402(self):
        # Test the `states_402` class attr
        for state, status_data in self.tc.states_402.items():
            # Values should be lists of len 2
            if state == 'START':
                assert status_data is None
                continue

            assert isinstance(status_data, list)
            assert len(status_data) == 2

            # Status words should match bitmask (except start)
            s_mask, s_word = status_data
            assert s_word & s_mask == s_word

    def test_fake_status_word(self, obj):
        for curr_state, control_word, exp_status_word in [
            # Automatic transitions:  control word ignored
            ('START', 0x0000, 0x0010),
            ('NOT READY TO SWITCH ON', 0x0000, 0x0050),
            ('FAULT REACTION ACTIVE', 0x9929, 0x0018),
            ('QUICK STOP ACTIVE', 0xFFFF, 0x8050),  # HOMING COMPLETED
            # Non-exhaustive list of transitions
            ('OPERATION ENABLED', 0x0000, 0x0050),
            ('OPERATION ENABLED', 0x0002, 0x0017),
            ('SWITCHED ON', 0x0002, 0x0050),
            ('SWITCH ON DISABLED', 0x0006, 0x0031),
            ('SWITCHED ON', 0x0006, 0x0031),
            ('OPERATION ENABLED', 0x0006, 0x0031),
            ('READY TO SWITCH ON', 0x0007, 0x0033),
            ('SWITCHED ON', 0x000F, 0x0037),
            ('FAULT', 0x0080, 0x0050),
            # Hold state
            ('OPERATION ENABLED', 0x000F, 0x0037),
            ('SWITCHED ON', 0x0007, 0x0033),
            ('SWITCH ON DISABLED', 0x0000, 0x0050),
            ('SWITCHED ON', 0x0006, 0x0031),
            # Nonsensical control word
            ('OPERATION ENABLED', 0x0006, 0x0031),
            ('SWITCH ON DISABLED', 0x000F, 0x0050),
            ('FAULT', 0x000F, 0x0018),
        ]:
            obj.curr_state = curr_state
            next_state, status_word = obj.fake_status_word(control_word)
            print(
                f'state/next:  {curr_state}/{next_state};  control/status '
                f'words:  0x{control_word:04X}/0x{status_word:04X}'
            )
            assert exp_status_word == status_word

    def test_add_status_word_flags(self):
        flags = dict(VOLTAGE_ENABLED=True, WARNING=False, REMOTE=True)
        assert self.tc.add_status_word_flags(0, **flags) == 0x0210

    def test_get_control_word_flags(self):
        test = dict(
            OPERATION_MODE_SPECIFIC_1=False,  # HM=HOMING_START
            OPERATION_MODE_SPECIFIC_2=False,
            OPERATION_MODE_SPECIFIC_3=False,
            HALT=False,
            NA_1=False,
            NA_2=False,
            MANUFACTURER_SPECIFIC_1=False,
            MANUFACTURER_SPECIFIC_2=False,
            MANUFACTURER_SPECIFIC_3=False,
            MANUFACTURER_SPECIFIC_4=False,
            MANUFACTURER_SPECIFIC_5=False,
        )
        assert self.tc.get_control_word_flags(0x0000) == test
        test.update(dict(OPERATION_MODE_SPECIFIC_1=True, HALT=True, NA_1=True))
        assert self.tc.get_control_word_flags(0x0310) == test

    def test_update_state_operational(self, obj):
        # Also test `obj.operational`, `obj.slave_online_changed`,
        # `obj.slave_oper_changed` properties
        obj.slave_online = False
        obj.slave_oper = False
        for (
            online_in,
            online_exp,
            prev_online_exp,
            oper_in,
            oper_exp,
            prev_oper_exp,
            operational_exp,
        ) in (
            # Drive offline
            (False, False, False, False, False, False, False),
            # Drive online but not slave_oper
            (True, True, False, False, False, False, False),
            # Drive offline again
            (False, False, True, False, False, False, False),
            # Drive online but not slave_oper again
            (True, True, False, False, False, False, False),
            # Drive online operational
            (True, True, True, True, True, False, True),
            # Drive online operational again
            (True, True, True, True, True, True, True),
            # Drive online, not operational
            (True, True, True, False, False, True, False),
        ):
            print(
                online_in,
                online_exp,
                prev_online_exp,
                oper_in,
                oper_exp,
                prev_oper_exp,
                operational_exp,
            )

            obj.update_state(online_in, oper_in, 0x0000)
            assert obj.slave_online is online_exp
            assert obj.prev_slave_online is prev_online_exp
            online_changed_exp = online_exp is not prev_online_exp
            assert obj.slave_online_changed is online_changed_exp
            assert obj.slave_oper is oper_exp
            assert obj.prev_slave_oper is prev_oper_exp
            oper_changed_exp = oper_exp is not prev_oper_exp
            assert obj.slave_oper_changed is oper_changed_exp
            assert obj.operational is operational_exp

        obj.update_state(False, False, 0x0000)
        assert obj.slave_online is False
        assert obj.slave_oper is False
        assert obj.operational is False

    def test_update_state(self, obj):
        # Also test related `get_status_flag` and 'drive_state_changed'
        sw = self.tc.states_402['READY TO SWITCH ON'][1]
        sw += self.sw_extra_bits_mask  # Set all extra status word bits
        obj.update_state(True, True, sw)
        assert obj.curr_state == 'READY TO SWITCH ON'
        assert obj.prev_state == 'START'
        assert obj.drive_state_changed()
        for flag in self.tc.sw_extra_bits:
            assert obj.curr_state_flags[flag]
            assert obj.get_status_flag(flag)

        # Bogus status word:  state unchanged; warning message
        obj.update_state(True, True, 0x42)
        self.mock_rospy.logwarn.assert_called()
        self.mock_rospy.logwarn.reset_mock()
        assert obj.curr_state == 'READY TO SWITCH ON'
        assert obj.prev_state == 'READY TO SWITCH ON'
        assert not obj.drive_state_changed()
        for flag in self.tc.sw_extra_bits:
            assert not obj.curr_state_flags[flag]
            assert not obj.get_status_flag(flag)

        obj.update_state(True, True, 0x23 + self.sw_extra_bits_mask)
        assert obj.curr_state == 'SWITCHED ON'
        assert obj.prev_state == 'READY TO SWITCH ON'
        assert obj.drive_state_changed()
        for flag in self.tc.sw_extra_bits:
            assert obj.curr_state_flags[flag]
            assert obj.get_status_flag(flag)

        obj.curr_state = 'START'
        obj.update_state(True, True, 0x0000)
        assert obj.curr_state == 'NOT READY TO SWITCH ON'

    def test_goal_paths(self):
        # Test the goal path dicts
        for goal_state, goal_path in self.tc.goal_paths.items():
            print("goal_state:", goal_state)
            assert goal_state in self.tc.states_402
            assert isinstance(goal_path, dict)
            assert len(goal_path) == len(self.tc.states_402)

            for state_src, state_data in goal_path.items():
                assert state_src in self.tc.states_402

                assert isinstance(state_data, list)
                assert len(state_data) == 2
                state_dst, trans = state_data

                assert state_dst in self.tc.states_402
                if trans is not None:
                    assert trans in self.tc.transitions

    def test_set_get_goal_state(self, obj):
        with pytest.raises(KeyError):
            obj.set_goal_state('bogus')

        for goal_state in self.tc.goal_paths:
            obj.set_goal_state(goal_state)
            assert obj.goal_state == goal_state
            assert obj.get_goal_state() == goal_state
            assert obj.goal_path is self.tc.goal_paths[goal_state]

    def test_get_next(self, obj):
        # Goal state SWITCH ON DISABLED
        assert obj.goal_state == 'SWITCH ON DISABLED'
        assert obj.curr_state == 'START'
        assert obj.get_next_state() == 'NOT READY TO SWITCH ON'
        assert obj.get_next_transition() == 'TRANSITION_0'

        obj.curr_state = 'OPERATION ENABLED'
        assert obj.get_next_state() == 'QUICK STOP ACTIVE'
        assert obj.get_next_transition() == 'TRANSITION_11'

        obj.curr_state = 'SWITCHED ON'
        assert obj.get_next_state() == 'SWITCH ON DISABLED'
        assert obj.get_next_transition() == 'TRANSITION_10'

        # Goal state OPERATION ENABLED
        obj.set_goal_state('OPERATION ENABLED')
        assert obj.get_next_state() == 'OPERATION ENABLED'
        assert obj.get_next_transition() == 'TRANSITION_4'

    def test_is_goal_state_reached(self, obj):
        # Goal state SWITCH ON DISABLED
        assert obj.goal_state == 'SWITCH ON DISABLED'
        assert obj.curr_state == 'START'
        assert not obj.is_goal_state_reached()

        obj.curr_state = 'SWITCH ON DISABLED'
        assert obj.is_goal_state_reached()

        # Not reached when not online operational
        obj.slave_oper = False
        assert not obj.operational
        assert not obj.is_goal_state_reached()
        obj.slave_oper = True
        assert obj.operational
        assert obj.is_goal_state_reached()

        obj.curr_state = 'FAULT'
        assert not obj.is_goal_state_reached()

        # Goal state FAULT
        obj.set_goal_state('FAULT')
        assert obj.is_goal_state_reached()

        obj.curr_state = 'SWITCH ON DISABLED'  # Alternate goal state
        assert obj.is_goal_state_reached()

        obj.curr_state = 'READY TO SWITCH ON'
        assert not obj.is_goal_state_reached()

        # Goal state OPERATION ENABLED:
        obj.set_goal_state('OPERATION ENABLED')
        assert not obj.is_goal_state_reached()

        obj.curr_state = 'SWITCHED ON'
        assert not obj.is_goal_state_reached()

        obj.curr_state = 'OPERATION ENABLED'
        assert obj.is_goal_state_reached()

    def test_transitions(self):
        # Test `transitions` class attr
        trans = self.tc.transitions

        for name, control_word in trans.items():
            assert isinstance(name, str)
            if control_word is not None:
                print(name, hex(control_word))
                # Be sure these bits and cw_extra_bits don't conflict
                assert not control_word & self.cw_extra_bits_mask

    automatic_transition_states = {
        'START',
        'NOT READY TO SWITCH ON',
        'FAULT REACTION ACTIVE',
    }

    def test_get_hold_state_control_word(self, obj):
        obj.set_goal_state('SWITCHED ON')
        assert len(self.tc.hold_state_control_word) == len(self.tc.states_402)
        for state in self.tc.states_402:
            assert state in self.tc.states_402
            obj.curr_state = state
            if state in self.automatic_transition_states:
                with pytest.raises(ValueError):
                    obj.get_hold_state_control_word()
            else:
                res = obj.get_hold_state_control_word()
                assert res == self.tc.hold_state_control_word[state]

    def test_get_transition_control_word(self, obj):
        for goal_state, status_word, expected in (
            # ** Goal state SWITCHED ON
            # Transition 1 -> SWITCH ON DISABLED (Automatic)
            ('SWITCHED ON', 0x00, None),
            # Transition 2 -> READY TO SWITCH ON
            ('SWITCHED ON', 0x40, 0x0006),
            # Transition 3 -> SWITCHED ON
            ('SWITCHED ON', 0x21, 0x0007),
            # Transition 15 -> SWITCH ON DISABLED
            ('SWITCHED ON', 0x08, 0x0080),
            # Transition 14 -> FAULT   (Automatic)
            ('SWITCHED ON', 0x0F, None),
            # Transition 12 -> SWITCH ON DISABLED
            ('SWITCHED ON', 0x07, 0x0000),
            # Hold state SWITCHED ON
            ('SWITCHED ON', 0x23, None),
            # Hold state OPERATION ENABLED
            ('OPERATION ENABLED', 0x27, None),
            # ** Goal state OPERATION ENABLED
            #  Transition 4 SWITCHED ON -> OPERATION ENABLED
            ('OPERATION ENABLED', 0x23, 0x000F),
            #  Hold state OPERATION ENABLED
            ('OPERATION ENABLED', 0x27, None),
        ):
            obj.set_goal_state(goal_state)
            obj.update_state(True, True, status_word)
            result = obj.get_transition_control_word()
            if expected is None:
                print(goal_state, hex(status_word), expected, result)
            else:
                print(goal_state, hex(status_word), hex(expected), hex(result))
            assert result == expected

    def test_add_control_word_flags(self):
        cw = 0
        for flags, flag_sum in (
            (dict(), 0x0000),
            (dict(HALT=True), 0x0100),
            (dict(HALT=False), 0x0000),
            (dict(NA_1=False, NA_2=True), 0x0400),
            (dict(NA_1=True, NA_2=False), 0x0200),
        ):
            assert self.tc.add_control_word_flags(cw, **flags) == cw + flag_sum

    def test_get_control_word(self, obj):
        obj.set_goal_state('SWITCHED ON')
        for status_word, kwargs, expected in (
            # Transition 1 -> SWITCH ON DISABLED (Automatic transition)
            (0x00, dict(), 0x0000),
            # Transition 2 -> READY TO SWITCH ON
            (0x40, dict(), 0x0006),
            # Transition 3 -> SWITCHED ON  (With flag)
            (0x21, dict(HALT=True), 0x0107),
            # Transition 15 -> SWITCH ON DISABLED
            (0x08, dict(NA_1=False, NA_2=True), 0x0480),
            # Transition 14 -> FAULT   (Automatic transition)
            (0x0F, dict(NA_1=True, NA_2=False), 0x0200),
            # Transition 12 -> SWITCH ON DISABLED
            (0x07, dict(OPERATION_MODE_SPECIFIC_1=True), 0x0010),
            # Hold state SWITCHED ON
            (0x23, dict(OPERATION_MODE_SPECIFIC_1=False), 0x0007),
            # Transition 11 -> QUICK STOP ACTIVE
            (0x27, dict(), 0x0002),
        ):
            obj.update_state(True, True, status_word)
            result = obj.get_control_word(**kwargs)
            print(hex(status_word), kwargs, hex(expected), hex(result))
            assert result == expected

            # Not online operational, status word is 0
            obj.slave_oper = False
            assert not obj.operational
            assert obj.get_control_word(**kwargs) == 0x0000
            # ...and back to normal
            obj.slave_oper = True
            assert obj.operational
            assert obj.get_control_word(**kwargs) == expected
