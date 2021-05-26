import pytest
from hal_402_device_mgr.hal_402_drive import Drive402


class TestDrive402Base:
    """Base class for other test classes to inherit from"""

    tc = Drive402
    halcomp_name = 'mock_hal_402_mgr'
    patch_mock_rospy = 'hal_402_device_mgr.hal_402_drive.rospy'
    ros_params_yaml = 'ros_params.yaml'
    drive_name = 'drive_1'
    drive_type = 'inovance_is620n'
    init_fixture_obj = True
    sim_mode = False  # Default overridden by `sim` fixture

    @pytest.fixture
    def sim(self):
        # Add this to test function BEFORE `obj`
        self.sim_mode = True

    obj_pin_start_vals = {
        # HAL_U32
        'error-code': 0x0000,
        'aux-error-code': 0x0000,
        'status-word': 0x0000,
        'status-word-sim': 0x0000,
        'control-word': 0x0000,
        'control-word-fb': 0x0000,
        'drive-mode-cmd': 0x0000,
        'drive-mode-fb': 0x0000,
        # HAL_BIT; pretend slave starts up already online
        'slave-online': True,
        'slave-oper': True,
        'slave-state-init': False,
        'slave-state-preop': False,
        'slave-state-safeop': False,
        'slave-state-op': True,
    }

    @pytest.fixture
    def obj(self, mock_halcomp, mock_rospy):
        print("Fixture 'obj':  new instance")
        obj = self.tc(
            self.drive_name,
            'inovance_is620n',
            0,
            mock_halcomp,
            sim=self.sim_mode,
        )
        if self.init_fixture_obj:
            print("Fixture 'obj':  running init()")
            obj.init()
            self.mock_halcomp.reset_mock()
            print("Fixture 'obj':  initializing pin internal values")
            for pname, pin in obj.pins.pin_dict.items():
                # Set value & ensure pin.changed is False, even after read/write
                pin.val = pin.old_val = self.obj_pin_start_vals[pname]
                pin.hal_pin.reset_mock()
        obj.sm402.slave_online = True
        obj.sm402.slave_oper = True
        self.mock_rospy.loginfo.reset_mock()
        self.mock_rospy.logwarn.reset_mock()
        print("Fixture 'obj':  done")
        yield obj

    def get_pin(self, pname):
        return self.mock_halcomp.get_pin(f'{self.drive_name}.{pname}')

    def get_pin_val(self, pname):
        return self.mock_halcomp.get_pin_val(f'{self.drive_name}.{pname}')

    def set_pin_val(self, pname, val):
        self.mock_halcomp.set_pin_val(f'{self.drive_name}.{pname}', val)


class TestDrive402Init(TestDrive402Base):
    init_fixture_obj = False

    ########################################
    # Init

    def test___init__(self, mock_halcomp):
        obj = self.tc(
            'drive_6',
            'inovance_is620n',
            5,
            mock_halcomp,
            transition_timeout=2.0,
            sim=False,
        )

        assert obj.drive_name == 'drive_6'
        assert obj.drive_type == 'inovance_is620n'
        assert obj.slave_number == 5
        assert obj.comp is mock_halcomp
        assert obj.transition_timeout == 2.0
        assert obj.sim is False

    def test_compname(self, obj):
        assert obj.compname == self.halcomp_name

    def test_setup_pins(self, obj):
        obj.setup_pins()
        assert hasattr(obj, 'pins')

        for pname in [
            'error-code',
            'aux-error-code',
            'status-word',
            'control-word',
        ]:
            assert pname in obj.pins.pin_dict
            full_pname = f'{self.drive_name}.{pname}'
            assert self.mock_halcomp.get_pin(full_pname) is not None

    def test_read_device_error_list(self, obj, ros_params):
        err_list = obj.read_device_error_list()
        assert err_list is ros_params['device_fault_code_list'][self.drive_type]

        obj.DEVICE_FAULT_CODE_PARAM = 'bogus'
        err_list = obj.read_device_error_list()
        assert err_list == dict()
        self.mock_rospy.logerr.assert_called()

    def test_create_topics(self, obj):
        obj.create_topics()
        assert hasattr(obj, 'topics')
        for topic in ['error']:
            assert topic in obj.topics

    def test_init(self, obj):
        obj.init()
        # Make sure major pieces were initialized
        for attr in ['pins', 'devices_error_list', 'topics']:
            assert hasattr(obj, attr)


class TestDrive402(TestDrive402Base):
    init_fixture_obj = True

    def set_state(self, obj, state):
        obj.sm402.curr_state = state

    ########################################
    # Main logic and external interface

    def test_get_set_goal_state(self, obj):
        obj.set_goal_state('OPERATION ENABLED')
        assert obj.sm402.goal_state == 'OPERATION ENABLED'
        assert obj.get_goal_state() == 'OPERATION ENABLED'
        obj.set_goal_state('SWITCH ON DISABLED')
        assert obj.sm402.goal_state == 'SWITCH ON DISABLED'
        assert obj.get_goal_state() == 'SWITCH ON DISABLED'

    def test_read_halpins(self, obj):
        pin_vals = {
            'error-code': 0x0024,
            'aux-error-code': 0x00240024,
            'status-word': 0x40,
            'status-word-sim': 0x42,
            'control-word': 0x0006,  # (Output pin)
            'control-word-fb': 0x0021,
            'drive-mode-cmd': 0x08,
            'drive-mode-fb': 0x02,
            'slave-online': False,
            'slave-oper': False,
            'slave-state-init': True,
            'slave-state-preop': True,
            'slave-state-safeop': True,
            'slave-state-op': False,
        }
        # Sanity check above list is updated
        assert set(pin_vals.keys()) == set(self.tc.pin_specs.keys())

        for pname, val in pin_vals.items():
            # Set HAL pins
            self.set_pin_val(pname, val)
        for pname in pin_vals:
            # Sanity check
            # - expected start value
            assert (
                obj.pins.pin_dict[pname].get() == self.obj_pin_start_vals[pname]
            )
            # - final value should change
            assert obj.pins.pin_dict[pname].get() != pin_vals[pname]
        # Read input HAL pin values (do NOT write output HAL pins)
        obj.read_halpins()
        for pname, val in pin_vals.items():
            pin_obj = obj.pins.pin_dict[pname]
            # Verify pin values in both directions
            if pin_obj.pdir == pin_obj.HAL_IN:
                assert pin_obj.get() == val
                assert self.get_pin_val(pname) == val
            else:
                assert pin_obj.get() == 0
                assert self.get_pin_val(pname) == val

    def test_update_state_machine(self, obj):
        # Also test obj.operational
        assert obj.sm402.curr_state == 'START'  # Sanity

        obj.pins.slave_online.set(True)
        obj.pins.slave_oper.set(False)
        obj.pins.status_word.set(0x21)  # READY TO SWITCH ON
        obj.update_state_machine()
        assert obj.sm402.slave_online is True
        assert obj.sm402.slave_oper is False
        assert not obj.operational
        assert obj.sm402.curr_state == 'START'  # Unchanged

        obj.pins.slave_oper.set(True)
        obj.update_state_machine()
        assert obj.operational
        assert obj.sm402.curr_state == 'READY TO SWITCH ON'

    def test_is_goal_state_reached(self, obj):
        assert obj.sm402.goal_state == 'SWITCH ON DISABLED'
        assert not obj.is_goal_state_reached()
        obj.sm402.curr_state = 'SWITCH ON DISABLED'
        assert obj.is_goal_state_reached()

        # Three possible states to reach goal
        obj.set_goal_state('FAULT')
        obj.sm402.curr_state = 'SWITCH ON DISABLED'
        assert obj.is_goal_state_reached()
        obj.sm402.curr_state = 'FAULT'
        assert obj.is_goal_state_reached()
        obj.sm402.curr_state = 'QUICK STOP ACTIVE'
        # Special case:  when 605Ah <=3, auto transition to SWITCH ON DISABLED
        assert not obj.is_goal_state_reached()
        obj.sm402.curr_state = 'OPERATION ENABLED'
        assert not obj.is_goal_state_reached()

    def test_sim_fake_next_inputs(self, sim, obj):
        assert obj.sim
        print('Setting up')
        self.set_pin_val('drive-mode-cmd', 0x08)  # CSP
        self.set_pin_val('control-word-fb', 0x0F)  # OPERATION ENABLED
        obj.sm402.curr_state = 'SWITCHED ON'

        print('Running sim_fake_next_inputs()')
        obj.sim_fake_next_inputs()

        print('Running checks')
        # control-mode-cmd should be echoed on control-mode-fb;
        assert self.get_pin_val('drive-mode-fb') == 0x08
        # Next state should be READY TO SWITCH ON, 0x21
        assert self.get_pin_val('status-word-sim') == 0x37

        assert obj.prev_fake_inputs == (0x08, 'OPERATION ENABLED', 0x37)

    def test_set_control_flags(self, obj):
        assert len(obj.control_flags) == 0
        obj.set_control_flags(VOLTAGE_ENABLED=True)
        assert len(obj.control_flags) == 1
        obj.set_control_flags()
        assert len(obj.control_flags) == 0
        obj.set_control_flags(WARNING=False, REMOTE=True)
        assert len(obj.control_flags) == 2

    def test_get_status_flag(self, obj):
        # 0x0000 state + all extra bits
        obj.sm402.update_state(True, True, 0xFF90)
        for bit in obj.sm402.sw_extra_bits:
            assert obj.get_status_flag(bit)

        # 0x0000 state + no extra bits
        obj.sm402.update_state(True, True, 0x0000)
        for bit in obj.sm402.sw_extra_bits:
            assert not obj.get_status_flag(bit)

    def test_effect_next_transition(self, sim, obj):

        # Helper function
        def helper(online, oper, cw, cw_changed, next_state):
            print(
                f"\ntest inputs:  online/oper {online}/{oper}"
                f" 0x{cw:0X} {cw_changed} '{next_state}'"
            )
            # - set up bogus drive_mode_cmd to test pin change
            obj.pins.drive_mode_cmd.set(-1)
            # - do transition
            print("Calling effect_next_transition() & checking results")
            obj.effect_next_transition()
            # - check drive-mode-cmd, control-word pins
            assert obj.pins.drive_mode_cmd.get() == obj.MODE_DEFAULT
            assert obj.pins.control_word.get() == cw
            assert obj.pins.control_word.changed is cw_changed
            print("Updating for next cycle")
            # - write control word to HAL pin
            obj.pins.control_word.write()
            self.set_pin_val(
                'control-word-fb', self.get_pin_val('control-word')
            )
            # - set status-word HAL pin from previous control-word
            obj.sim_fake_next_inputs()
            # - update state machine with sim status word
            self.set_pin_val('status-word', self.get_pin_val('status-word-sim'))
            obj.pins.status_word.read()
            # - (sim) status after state machine update
            obj.sm402.update_state(online, oper, obj.pins.status_word.get())
            assert obj.operational is (online and oper)
            assert obj.sm402.curr_state == next_state

        # Initial goal state: SWITCH ON DISABLED
        obj.sm402.set_goal_state('SWITCH ON DISABLED')
        # - transition 0 (automatic)
        helper(True, True, 0x0000, False, 'NOT READY TO SWITCH ON')
        # - transition 1 (automatic)
        helper(True, True, 0x0000, False, 'SWITCH ON DISABLED')
        # - reached goal state; hold
        helper(True, True, 0x0000, False, 'SWITCH ON DISABLED')

        # Next goal state:  SWITCHED ON
        obj.sm402.set_goal_state('SWITCHED ON')
        # - drive not online operational; no change
        helper(True, False, 0x0006, True, 'SWITCH ON DISABLED')
        # - recover
        helper(True, True, 0x0000, True, 'SWITCH ON DISABLED')
        # - transition 2
        helper(True, True, 0x0006, True, 'READY TO SWITCH ON')
        # - transition 3
        helper(True, True, 0x0007, True, 'SWITCHED ON')
        # - reached goal state; hold
        helper(True, True, 0x0007, False, 'SWITCHED ON')
        # - reached goal state; hold; bits set
        obj.set_control_flags(HALT=True)
        helper(True, True, 0x0107, True, 'SWITCHED ON')
        obj.set_control_flags()

        # Next goal state:  OPERATION ENABLED
        obj.sm402.set_goal_state('OPERATION ENABLED')
        # - transition 4
        helper(True, True, 0x000F, True, 'OPERATION ENABLED')
        # - reached goal state; hold
        helper(True, True, 0x000F, False, 'OPERATION ENABLED')

        # New goal:  FAULT
        obj.sm402.set_goal_state('FAULT')
        # - transition 11
        helper(True, True, 0x0002, True, 'QUICK STOP ACTIVE')
        # - transition 12 (automatic)
        helper(True, True, 0x0000, True, 'SWITCH ON DISABLED')
        # - reached goal state; hold
        helper(True, True, 0x0000, False, 'SWITCH ON DISABLED')

    def test_write_halpins(self, obj):
        obj.write_halpins()
        for pname in self.mock_halcomp.pin_names():
            if self.mock_halcomp.get_pin_dir(pname) == obj.pins.HAL_IN:
                self.mock_halcomp.get_pin(pname).set.assert_not_called()
            else:
                self.mock_halcomp.get_pin(pname).set.assert_called()

    def test_set_get_control_mode(self, obj):
        mode_hm = obj.normalize_control_mode('MODE_HM')
        obj.set_control_mode('MODE_HM')
        assert obj.sm402.get_control_mode() == mode_hm
        assert obj.get_control_mode() == mode_hm

    def test_normalize_control_mode(self, obj):
        for mode in ('MODE_HM', 'MODE_CSP', 'MODE_CST'):
            mode_num = getattr(obj.sm402, mode)
            print('mode, mode_num:', mode, mode_num)
            assert obj.normalize_control_mode(mode) == mode_num
            assert obj.normalize_control_mode(mode_num) == mode_num

    ########################################
    # ROS topics

    def test_log_error_state(self, obj):
        # Setup:  Drive offline, not operational
        obj.sm402.slave_online = False
        obj.sm402.slave_oper = False
        assert obj.state != 'FAULT'

        # Simulate offline state
        obj.sm402.update_state(False, False, 0x0000)
        assert not obj.sm402.slave_online_changed
        assert not obj.sm402.slave_oper_changed
        obj.log_error_state()
        self.mock_rospy.logwarn.assert_not_called()
        self.mock_rospy.loginfo.assert_not_called()

        # Drive comes online, not operational
        obj.sm402.update_state(True, False, 0x0000)
        assert obj.sm402.slave_online_changed
        assert not obj.sm402.slave_oper_changed
        obj.log_error_state()
        self.mock_rospy.logwarn.assert_not_called()
        self.mock_rospy.loginfo.assert_called()
        self.mock_rospy.loginfo.reset_mock()

        # Drive comes online, operational
        obj.sm402.update_state(True, True, 0x0000)
        assert not obj.sm402.slave_online_changed
        assert obj.sm402.slave_oper_changed
        obj.log_error_state()
        self.mock_rospy.logwarn.assert_not_called()
        self.mock_rospy.loginfo.assert_called()
        self.mock_rospy.loginfo.reset_mock()

        # Drive remains online, operational
        obj.sm402.update_state(True, True, 0x0000)
        assert not obj.sm402.slave_online_changed
        assert not obj.sm402.slave_oper_changed
        obj.log_error_state()
        self.mock_rospy.logwarn.assert_not_called()
        self.mock_rospy.loginfo.assert_not_called()

        # Simulate FAULT state
        assert obj.state != 'FAULT'
        obj.sm402.set_goal_state('FAULT')
        self.set_state(obj, 'FAULT')

        # Look for warning
        print('prev, curr:', obj.sm402.prev_state, obj.sm402.curr_state)
        assert obj.sm402.drive_state_changed()
        obj.log_error_state()
        self.mock_rospy.logwarn.assert_called()
        self.mock_rospy.logwarn.reset_mock()

        # Don't try to recover
        obj.sm402.update_state(True, True, 0x08)
        print('prev, curr:', obj.sm402.prev_state, obj.sm402.curr_state)
        assert not obj.sm402.drive_state_changed()
        obj.log_error_state()
        self.mock_rospy.logwarn.assert_not_called()

        # Recover from fault
        obj.sm402.update_state(True, True, 0x40)
        print('prev, curr:', obj.sm402.prev_state, obj.sm402.curr_state)
        assert obj.sm402.drive_state_changed()
        obj.log_error_state()
        self.mock_rospy.loginfo.assert_called()
        self.mock_rospy.loginfo.reset_mock()

        # Drive goes online, not operational
        obj.sm402.update_state(True, False, 0x08)
        assert not obj.sm402.slave_online_changed
        assert obj.sm402.slave_oper_changed
        obj.log_error_state()
        self.mock_rospy.logwarn.assert_called()
        self.mock_rospy.logwarn.reset_mock()
        self.mock_rospy.loginfo.assert_not_called()

        # Drive goes offline, not operational
        obj.sm402.update_state(False, False, 0x08)
        assert obj.sm402.slave_online_changed
        assert not obj.sm402.slave_oper_changed
        obj.log_error_state()
        self.mock_rospy.logwarn.assert_called()
        self.mock_rospy.logwarn.reset_mock()
        self.mock_rospy.loginfo.assert_not_called()

        # Drive remains offline, not operational
        obj.sm402.update_state(False, False, 0x08)
        assert not obj.sm402.slave_online_changed
        assert not obj.sm402.slave_oper_changed
        obj.log_error_state()
        self.mock_rospy.logwarn.assert_not_called()
        self.mock_rospy.loginfo.assert_not_called()

    def test_error_code_hex(self, obj):
        assert obj.error_code_hex(0x0040) == '0x0040'
        assert obj.error_code_hex(0x00FF) == '0x00FF'
        assert obj.error_code_hex(0x0000) == ''

    def test_get_error_info(self, obj, ros_params):
        err_info = ros_params['device_fault_code_list']['inovance_is620n'][
            '0x0101'
        ]
        assert (
            obj.get_error_info(0x0101)['description'] == err_info['description']
        )
        assert obj.get_error_info(0x0000)['description'] == "No error"
        assert (
            obj.get_error_info(0x0001)['description']
            == Drive402.GENERIC_ERROR_DESCRIPTION
        )

    def test_publish_error(self, obj):
        # No change; returns without action
        obj.publish_error()
        obj.topics['error'].publish.assert_not_called()

        # New error
        obj.pins.error_code.set(0x0101)
        assert obj.pins.error_code.changed
        obj.publish_error()
        obj.topics['error'].publish.assert_called()
        self.mock_rospy.logerr.assert_called()
        obj.topics['error'].publish.reset_mock()

        # Error recovered
        obj.pins.error_code.read()
        obj.pins.error_code.set(0x0000)
        obj.publish_error()
        obj.topics['error'].publish.assert_called()
        self.mock_rospy.loginfo.assert_called()

    def test_publish_status(self, obj):
        # Set FAULT state for log_error_state
        obj.sm402.set_goal_state('FAULT')
        self.set_state(obj, 'FAULT')
        assert obj.sm402.drive_state_changed()

        # Set error_code.changed for publish_error
        obj.pins.error_code.set(0x0101)
        assert obj.pins.error_code.changed

        obj.publish_status()
        # log_error_state ran
        self.mock_rospy.logwarn.assert_called()
        # publish_error ran
        obj.topics['error'].publish.assert_called()

    ########################################
    # Update

    def test_read_write_state(self, sim, obj, mocker):
        mock_methods = dict(
            read_halpins=None,
            update_state_machine=None,
            effect_next_transition=None,
            publish_status=None,
            write_halpins=None,
            sim_fake_next_inputs=None,
        )
        for mname in mock_methods:
            method = mock_methods[mname] = mocker.MagicMock()
            setattr(obj, mname, method)

        obj.read_state()
        obj.write_state()

        for mname in mock_methods:
            print(mname)
            mock_methods[mname].assert_called()
