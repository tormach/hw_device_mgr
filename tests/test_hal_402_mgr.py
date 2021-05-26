import pytest
import yaml
from hal_402_device_mgr.hal_402_mgr import Hal402Mgr


class TestHal402MgrBase:
    tc = Hal402Mgr
    patch_mock_rospy = (
        'hal_402_device_mgr.hal_402_mgr.rospy',
        'hal_402_device_mgr.hal_402_drive.rospy',
    )
    ros_params_yaml = 'ros_params.yaml'

    init_fixture_obj = True
    init_pins = True
    sim = True
    drives_start_operational = True

    @pytest.fixture
    def start_offline(self):
        # Add this to test function BEFORE `obj`
        self.drives_start_operational = False

    @pytest.fixture
    def obj(self, mock_hal, mock_rospy):
        obj = self.tc()
        if self.init_fixture_obj:
            obj.init()
        if not self.sim:
            obj.sim = False
        if self.init_fixture_obj and self.init_pins:
            for pin in obj.pins.pin_dict.values():
                # Set value & ensure pin.changed is False, even after update()
                pin.val = 0
                pin.old_val = 0
                self.comp.set_pin_val(pin.name, 0)
            for drive in obj.drives:
                pin_start_vals = {k: 0 for k in drive.pins.pin_dict}
                if self.drives_start_operational:
                    drive.sm402.slave_online = True
                    drive.sm402.slave_oper = True
                    pin_start_vals.update(
                        {
                            'slave-online': True,
                            'slave-oper': True,
                            'slave-state-op': True,
                        }
                    )
                    drive.sm402.curr_state_flags['VOLTAGE_ENABLED'] = True
                for pname, pin in drive.pins.pin_dict.items():
                    # Set value & ensure pin.changed is False, even after update()
                    val = pin_start_vals[pname]
                    pin.val = pin.old_val = val
                    self.comp.set_pin_val(pin.name, val)
        self.obj = obj
        yield obj

    @property
    def comp(self):
        return self.components.get_instance().mock_obj

    def drive(self, num):
        return self.obj.drives[num - 1]

    def sm402(self, num):
        return self.drive(num - 1).sm402

    @property
    def MODE_DEFAULT(self):
        return self.drive(1).MODE_DEFAULT

    @property
    def MODE_CSP(self):
        return self.sm402(1).MODE_CSP

    @property
    def MODE_HM(self):
        return self.sm402(1).MODE_HM


# FIXME state_cmd start; safety_input 0; enabling_input 0; (fault) state_cmd 1


class TestHal402MgrInit(TestHal402MgrBase):
    init_fixture_obj = False

    def test_init_ros(self, obj):
        obj.init_ros()
        self.mock_rospy.init_node.assert_called_with('hal_402_mgr')
        self.mock_rospy.on_shutdown.assert_called_with(obj.call_cleanup)
        assert obj.update_rate == 10
        self.mock_rospy.Rate.assert_called_with(10)
        assert hasattr(obj, 'rate')

    def test_hal_comp_init(self, obj):
        # Check HAL component creation, pin creation, pin values
        obj.hal_comp_init()
        assert obj.comp is self.comp
        assert hasattr(obj, 'pins')
        assert hasattr(obj.pins, 'state_cmd')

    def test_hal_comp_ready(self, obj):
        obj.hal_comp_init()
        obj.hal_comp_ready()
        self.comp.ready.assert_called_once()
        assert self.comp.is_ready is True

    def test_create_drives(self, obj):
        obj.hal_comp_init()
        obj.create_drives()
        print('drives:', obj.drives)
        assert isinstance(obj.drives, list)
        assert len(obj.drives) == 6
        drive_inst = -1
        for drive in obj.drives:
            assert type(drive).__name__ == 'Drive402'
            # Check for increasing numbers
            assert drive.slave_number > drive_inst
            drive_inst = drive.slave_number

        # If drive config param is absent
        obj.ros_param_base = 'bogus'
        self.mock_rospy.logerr.reset_mock()
        obj.create_drives()
        assert obj.drives == []
        self.mock_rospy.logerr.assert_called()

    def test_init(self, obj):
        obj.init()
        # Check that main components were initialized
        assert hasattr(obj, 'rate')
        assert hasattr(obj, 'comp')
        assert hasattr(obj, 'drives')
        assert obj.drives[0].sm402.goal_state == 'SWITCH ON DISABLED'
        self.comp.ready.assert_called_once()


class TestHal402Mgr(TestHal402MgrBase):
    ####################################################
    # Drive helpers

    def test_read_write_drives_state(self, obj, mocker):
        # hal_402_drive.{read,write}_drives_state() too complex; mock
        for i in range(len(obj.drives)):
            obj.drives[i] = mocker.MagicMock()
        obj.read_drives_state()
        obj.write_drives_state()
        for drive in obj.drives:
            drive.read_state.assert_called()
            drive.write_state.assert_called()

    def test_set_drive_goal_state(self, obj):
        obj.set_drive_goal_state('SWITCHED ON')
        for drive in obj.drives:
            assert drive.sm402.goal_state == 'SWITCHED ON'
            assert drive.control_flags == {}

    def test_set_drive_control_mode(self, obj):
        obj.set_drive_control_mode(self.MODE_HM)
        for drive in obj.drives:
            assert drive.sm402.get_control_mode() == self.MODE_HM

    def test_set_drive_control_flags(self, obj):
        obj.set_drive_control_flags(HALT=True, NA_1=False, NA_2=True)
        for drive in obj.drives:
            assert drive.control_flags == dict(HALT=True, NA_1=False, NA_2=True)
        obj.set_drive_control_flags()
        for drive in obj.drives:
            assert drive.control_flags == dict()

    def test_all_drives_mode(self, obj):
        assert obj.all_drives_mode(self.MODE_DEFAULT)
        assert not obj.all_drives_mode(self.MODE_HM)

        self.sm402(1).set_control_mode(self.MODE_HM)
        assert not obj.all_drives_mode(self.MODE_DEFAULT)

        for i in range(1, 7):
            self.sm402(i).set_control_mode(self.MODE_HM)
        obj.set_drive_control_mode(self.MODE_HM)
        assert obj.all_drives_mode(self.MODE_HM)
        assert not obj.all_drives_mode(self.MODE_DEFAULT)

    def test_all_drives_status_flags(self, start_offline, obj):
        assert obj.all_drives_status_flags(WARNING=False, REMOTE=False)
        for drive in obj.drives:
            drive.sm402.update_state(True, True, 0x23 + 0xFF90)
        assert obj.all_drives_status_flags(WARNING=True, REMOTE=True)
        self.sm402(1).update_state(True, True, 0)
        assert not obj.all_drives_status_flags(WARNING=True, REMOTE=True)

    def test_drives_operational(self, start_offline, obj):
        # Also test all_drives_operational()
        for drive in obj.drives:
            assert not drive.operational  # Sanity
        assert set(obj.drives_operational()) == set()
        assert set(obj.drives_operational(negate=True)) == set(obj.drives)
        assert not obj.all_drives_operational()

        online = obj.drives[:3]
        offline = obj.drives[3:]
        for drive in online:
            drive.sm402.slave_online = True
            drive.sm402.slave_oper = True
            assert drive.operational
        assert set(obj.drives_operational()) == set(online)
        assert set(obj.drives_operational(negate=True)) == set(offline)
        assert not obj.all_drives_operational()

        for drive in offline:
            drive.sm402.slave_online = True
            drive.sm402.slave_oper = True
            assert drive.operational
        assert set(obj.drives_operational()) == set(obj.drives)
        assert set(obj.drives_operational(negate=True)) == set()
        assert obj.all_drives_operational()

    def test_drives_in_state(self, obj):
        d012 = obj.drives[:3]
        d345 = obj.drives[3:]
        assert len(obj.drives_in_state('START')) == 6
        assert obj.all_drives_in_state('START')
        assert obj.any_drives_in_state('START')
        assert len(obj.drives_in_state('OPERATION ENABLED')) == 0
        assert not obj.all_drives_in_state('OPERATION ENABLED')
        assert not obj.any_drives_in_state('OPERATION ENABLED')

        for drive in d012:
            drive.sm402.curr_state = 'OPERATION ENABLED'
        assert len(obj.drives_in_state('START')) == 3
        assert not obj.all_drives_in_state('START')
        assert obj.any_drives_in_state('START')
        assert len(obj.drives_in_state('OPERATION ENABLED')) == 3
        assert not obj.all_drives_in_state('OPERATION ENABLED')
        assert obj.any_drives_in_state('OPERATION ENABLED')

        for drive in d345:
            drive.sm402.curr_state = 'OPERATION ENABLED'
        assert len(obj.drives_in_state('START')) == 0
        assert not obj.all_drives_in_state('START')
        assert not obj.any_drives_in_state('START')
        assert len(obj.drives_in_state('OPERATION ENABLED')) == 6
        assert obj.all_drives_in_state('OPERATION ENABLED')
        assert obj.any_drives_in_state('OPERATION ENABLED')

    ####################################################
    # Drive state FSM

    @pytest.fixture
    def e(self, mocker):
        return mocker.MagicMock(dst='bogus_complete')

    #
    # Helpers
    #
    def test_fsm_command_from_event(self, e):
        for cmd, src_dst_map in self.tc.fsm_next_state_map.items():
            for src, dst in src_dst_map.items():
                e.dst = src
                assert self.tc.fsm_command_from_event(e) == cmd
                if dst is None:
                    continue
                assert dst in src_dst_map  # dst should be next src
                e.dst = dst
                assert self.tc.fsm_command_from_event(e) == cmd

    def test_fsm_check_command(self, obj, e):
        obj.command = 'start'

        # Command can't run:  already running
        e.configure_mock(src='start_3', dst='start_4')
        assert not obj.fsm_check_command(e)

        # Command can run
        e.configure_mock(src='start_complete', dst='stop_command')
        assert obj.fsm_check_command(e)
        assert obj.command == 'stop'

    def test_fsm_check_drive_goal_state(self, obj, e):
        for drive in obj.drives:
            drive.sm402.curr_state = 'OPERATION ENABLED'
        obj.set_drive_goal_state('FAULT')
        assert not obj.fsm_check_drive_goal_state(e, 'OPERATION ENABLED')
        obj.set_drive_goal_state('OPERATION ENABLED')
        assert obj.fsm_check_drive_goal_state(e, 'OPERATION ENABLED')

    def test_fsm_set_drive_goal_state(self, obj, e):
        obj.fsm_set_drive_goal_state(e, 'OPERATION ENABLED')
        for drive in obj.drives:
            assert drive.sm402.goal_state == 'OPERATION ENABLED'

    def test_fsm_check_drive_control_mode(self, obj, e):
        obj.set_drive_control_mode(self.MODE_CSP)
        assert obj.fsm_check_drive_control_mode(e, self.MODE_CSP)
        assert not obj.fsm_check_drive_control_mode(e, self.MODE_HM)

    def test_fsm_set_drive_control_mode(self, obj, e):
        obj.fsm_set_drive_control_mode(e, self.MODE_HM)
        assert obj.all_drives_mode(self.MODE_HM)

    @pytest.fixture
    def on_before_command_helper(self, obj, e):
        def helper(cmd, test_cases):
            print(f'on_before_command_helper:  {cmd}')
            method = getattr(obj, f'on_before_{cmd}_command')
            for src, res in test_cases:
                obj.command = ''
                print(f'Checking src {src}_complete; res {res}')
                e.configure_mock(src=f'{src}_complete', dst=f'{cmd}_command')
                assert method(e) is res
                if res:
                    assert obj.command == cmd

        return helper

    @pytest.fixture
    def on_enter_transition_helper(self, obj, e):
        def helper(transition, state):
            method = getattr(self.obj, f'on_enter_{transition}')
            method(e)
            assert self.sm402(1).goal_state == state
            assert self.drive(1).control_flags == {}

        return helper

    @pytest.fixture
    def on_enter_control_mode_helper(self, obj, e):
        def helper(transition, mode):
            if mode is None:
                mode = getattr(self.sm402(1), obj.default_control_mode)
            mode = self.drive(1).normalize_control_mode(mode)
            method = getattr(self.obj, f'on_enter_{transition}')
            method(e)
            assert self.sm402(1).get_control_mode() == mode

        return helper

    @pytest.fixture
    def on_before_transition_helper(self, obj, e):
        def helper(transition, state):
            method = getattr(self.obj, f'on_before_{transition}')
            obj.set_drive_goal_state(state)
            for drive in obj.drives:
                drive.sm402.curr_state = state
            assert method(e)
            for drive in obj.drives:
                drive.sm402.curr_state = 'START'
            assert not method(e)

        return helper

    def all_drive_pin_values(self, obj, pname, val):
        for drive in obj.drives:
            print(f'{pname}:  {drive.pins.pin_dict[pname].val:X} {val:X}')
            if drive.pins.pin_dict[pname].val != val:
                return False
        return True

    @pytest.fixture
    def on_before_control_mode_helper(self, obj, e):
        def helper(transition, mode):
            if mode is None:
                mode = getattr(self.sm402(1), obj.default_control_mode)
            mode = self.drive(1).normalize_control_mode(mode)

            method_name = f'on_before_{transition}'
            method = getattr(obj, method_name)
            print(f"method={method_name}; mode={mode}")

            for drive in obj.drives:
                drive.sm402.set_control_mode(mode)
            assert obj.all_drives_mode(mode)  # First branch
            assert method(e)

            self.drive(1).sm402.set_control_mode((mode + 1) % 2)
            assert not obj.all_drives_mode(mode)  # Second branch
            assert not method(e)

        return helper

    @pytest.fixture
    def on_enter_complete_helper(self, obj, e):
        def helper(command):
            e.configure_mock(src=f'{command}_4', dst=f'{command}_complete')
            method_name = f'on_enter_{command}_complete'
            method = getattr(obj, method_name)
            obj.pins.state_cmd.set(-1)
            method(e)
            assert obj.pins.state_cmd.get() == obj.cmd_name_to_int_map[command]

        return helper

    #
    # Fault command
    #
    def test_on_before_fault_command(self, on_before_command_helper):
        test_cases = (
            ('stopped', True),
            ('start', True),
            ('home', True),
            ('fault', False),
        )
        on_before_command_helper('fault', test_cases)

    def test_on_enter_fault_1(self, on_enter_transition_helper):
        on_enter_transition_helper('fault_1', 'FAULT')

    def test_on_before_fault_complete(self, on_before_transition_helper):
        on_before_transition_helper('fault_complete', 'FAULT')

    def test_on_enter_fault_complete(self, on_enter_complete_helper):
        on_enter_complete_helper('fault')

    #
    # Start command
    #
    def test_on_before_start_command(self, on_before_command_helper):
        test_cases = (
            ('stop', True),
            ('start', False),
            ('home', True),
            ('fault', True),
        )
        on_before_command_helper('start', test_cases)

    def test_on_enter_start_1(self, on_enter_transition_helper):
        on_enter_transition_helper('start_1', 'SWITCHED ON')

    def test_on_before_start_2(self, on_before_transition_helper):
        on_before_transition_helper('start_2', 'SWITCHED ON')

    def test_on_enter_start_2(self, on_enter_control_mode_helper):
        on_enter_control_mode_helper('start_2', None)

    def test_on_before_start_3(self, on_before_control_mode_helper):
        on_before_control_mode_helper('start_3', None)

    def test_on_enter_start_3(self, on_enter_transition_helper):
        on_enter_transition_helper('start_3', 'OPERATION ENABLED')

    def test_on_before_start_complete(self, on_before_transition_helper):
        on_before_transition_helper('start_complete', 'OPERATION ENABLED')

    def test_on_enter_start_complete(self, on_enter_complete_helper):
        on_enter_complete_helper('start')

    #
    # Stop command
    #
    def test_on_before_stop_command(self, on_before_command_helper):
        test_cases = (
            ('stop', False),
            ('start', True),
            ('home', True),
            ('fault', True),
        )
        on_before_command_helper('stop', test_cases)

    def test_on_enter_stop_1(self, on_enter_transition_helper):
        on_enter_transition_helper('stop_1', 'SWITCH ON DISABLED')

    def test_on_before_stop_complete(self, on_before_transition_helper):
        on_before_transition_helper('stop_complete', 'SWITCH ON DISABLED')

    def test_on_enter_stop_complete(self, on_enter_complete_helper):
        on_enter_complete_helper('stop')

    #
    # Homed state
    #
    def test_on_before_home_command(self, on_before_command_helper):
        test_cases = (
            ('stop', True),
            ('start', False),
            ('home', False),
            ('fault', False),
        )
        on_before_command_helper('home', test_cases)

    def test_on_enter_home_1(self, on_enter_control_mode_helper):
        on_enter_control_mode_helper('home_1', 'MODE_HM')

    def test_on_before_home_2(self, on_before_control_mode_helper):
        on_before_control_mode_helper('home_2', 'MODE_HM')

    def test_on_enter_home_2(self, on_enter_transition_helper):
        on_enter_transition_helper('home_2', 'OPERATION ENABLED')

    def test_on_before_home_3(self, on_before_transition_helper):
        on_before_transition_helper('home_3', 'OPERATION ENABLED')

    def test_on_enter_home_3(self, obj, e):
        obj.on_enter_home_3(e)
        for drive in obj.drives:
            # MODE_HM:  OPERATION_MODE_SPECIFIC_1 = HOMING_START
            assert drive.control_flags['OPERATION_MODE_SPECIFIC_1'] is True

    def test_on_before_home_complete(self, obj, e):
        for drive in obj.drives:
            drive.sm402.curr_state_flags['HOMING_COMPLETED'] = True
        assert obj.on_before_home_complete(e)
        self.sm402(1).curr_state_flags['HOMING_COMPLETED'] = False
        assert not obj.on_before_home_complete(e)
        self.sm402(1).curr_state_flags.pop('HOMING_COMPLETED')
        assert not obj.on_before_home_complete(e)

    def test_on_enter_home_complete(self, on_enter_complete_helper):
        on_enter_complete_helper('home')

    ####################################################
    # Execution

    def test_call_cleanup(self, obj):
        # Only writes log messages
        obj.call_cleanup()

    def test_run(self, obj, mocker):
        # Mock obj.update both to sidestep more complex mocking and to
        # shutdown after a few iterations
        mock_update = mocker.MagicMock()
        obj.update = mock_update

        # Run ends with rospy.is_shutdown()
        self.mock_rospy.is_shutdown_behavior = 'shutdown'
        obj.run()
        assert len(mock_update.mock_calls) == 2
        assert len(obj.rate.sleep.mock_calls) == 2
        self.mock_rospy.reset()
        mock_update.reset_mock()

        # Run ends with rospy.ROSInterruptException
        self.mock_rospy.is_shutdown_behavior = 'raise'
        obj.run()
        assert getattr(self, 'raised_ros_interrupt_exception', False) is True
        assert len(mock_update.mock_calls) == 2
        obj.rate.sleep.assert_called()


class TestHal402MgrUpdate(TestHal402MgrBase):
    sim = False  # Need to run drive.sim_fake_next_inputs() ourselves

    # Start with slaves offline
    drives_start_operational = False

    def drive_pin_val(self, drive, pname, new_val=None):
        pin = drive.pins.pin_dict[pname]
        if new_val is None:
            return self.comp.get_pin_val(pin.name)
        else:
            self.comp.set_pin_val(pin.name, new_val)

    def expand_drives(self, data):
        if not isinstance(data, dict):
            return {f'drive_{i}': data for i in range(1, 7)}
        if 'all' in data:
            data = data.copy()
            data_all = data.pop('all')
            data_tmp = {f'drive_{i}': data_all for i in range(1, 7)}
            for key in data:
                data_tmp[key] = data[key]
            return data_tmp
        return data

    #
    # Header
    #
    def print_iter_intro(self, desc):
        print()
        print('*' * 80)
        print('    ', desc)
        print('*' * 80)

    #
    # Object checks
    #
    def check_fsm_state(self, exp):
        val = self.obj.state
        print(f'  fsm_state: exp "{exp}"; actual "{val}"')
        if val != exp:
            print('MISMATCH')
            return False
        return True

    def check_drive_state(self, exp):
        exp = self.expand_drives(exp)
        passing = True
        for drive in self.obj.drives:
            name = drive.drive_name
            val = drive.sm402.curr_state
            print(f'  drive_state {name}: exp "{exp[name]}"; actual "{val}"')
            if val != exp[name]:
                print('MISMATCH')
                passing = False
        return passing

    def check_slave_online(self, exp):
        exp = self.expand_drives(exp)
        passing = True
        for drive in self.obj.drives:
            name = drive.drive_name
            val = drive.sm402.slave_online
            print(f'  slave_online {name}: exp "{exp[name]}"; actual "{val}"')
            if val != exp[name]:
                print('MISMATCH')
                passing = False
        return passing

    def check_slave_oper(self, exp):
        exp = self.expand_drives(exp)
        passing = True
        for drive in self.obj.drives:
            name = drive.drive_name
            val = drive.sm402.slave_oper
            print(f'  slave_oper {name}: exp "{exp[name]}"; actual "{val}"')
            if val != exp[name]:
                print('MISMATCH')
                passing = False
        return passing

    def check_pins(self, expected):
        passing = True
        for pname, exp in expected.items():
            val = self.comp.get_pin_val(pname)
            print(f'  pin {pname}:  exp {exp}; actual {val}')
            if val != exp:
                print('MISMATCH')
                passing = False
        return passing

    pin_value_masks = {
        'control-word': 0x008F,
        'status-word': 0x006F,
    }

    def check_drive_pins(self, all_expected):
        passing = True
        for drive in self.obj.drives:
            name = drive.drive_name
            exp = dict()
            exp.update(all_expected.get('all', dict()))
            exp.update(all_expected.get(drive.drive_name, dict()))
            for pname, exp in exp.items():
                if exp is None:
                    print(f'  drive_pin {name}.{pname}:  (ignoring)')
                    continue
                val = self.drive_pin_val(drive, pname)
                val &= self.pin_value_masks.get(pname, 0xFFFF)
                print(
                    f'  drive_pin {name}.{pname}:'
                    f' exp 0x{exp:X}; actual 0x{val:X}'
                )
                if val != exp:
                    print('MISMATCH')
                    passing = False
        return passing

    def check_goal_state(self, exp):
        passing = True
        for drive in self.obj.drives:
            name = drive.drive_name
            val = drive.sm402.goal_state
            print(f'  goal_state {name}:  exp {exp}; actual {val}')
            if val != exp:
                print('MISMATCH')
                passing = False
        return passing

    def check_goal_state_reached(self, exp):
        exp = self.expand_drives(exp)
        passing = True
        for drive in self.obj.drives:
            name = drive.drive_name
            val = drive.is_goal_state_reached()
            print(
                f'  goal_state_reached {name}:  exp {exp[name]}; actual {val}'
            )
            if val != exp[name]:
                print('MISMATCH')
                passing = False
        return passing

    def check_control_word_flags(self, exp):
        passing = True
        for name, val in exp.items():
            for drive in self.obj.drives:
                mask = 1 << drive.sm402.cw_extra_bits[name]
                cw = self.drive_pin_val(drive, 'control-word')
                flag = bool(mask & cw)
                dname = drive.drive_name
                print(
                    f'  control_word_flags {dname}:  {name} '
                    f'exp {val}; actual {flag}'
                )
                if flag != val:
                    print('MISMATCH')
                    passing = False
        return passing

    def check_status_word_flags(self, exp):
        passing = True
        for name, val in exp.items():
            for drive in self.obj.drives:
                mask = 1 << drive.sm402.sw_extra_bits[name]
                sw = self.drive_pin_val(drive, 'status-word')
                flag = bool(mask & sw)
                dname = drive.drive_name
                print(
                    f'  status_word_flags {dname}:  {name} '
                    f'exp {val}; actual {flag}'
                )
                if flag != val:
                    print('MISMATCH')
                    passing = False
        return passing

    def check_obj(self, when, spec):
        print(f'*** {when} state checks:')
        passing = True
        for key, val in spec.items():
            if not getattr(self, f'check_{key}')(val):
                passing = False

        assert passing

    #
    # Changes
    #
    def change_drive_pins(self, all_changes):
        for drive in self.obj.drives:
            name = drive.drive_name
            changes = dict()
            changes.update(all_changes.get('all', dict()))
            changes.update(all_changes.get(name, dict()))
            for pname, val in changes.items():
                self.drive_pin_val(drive, pname, val)
                print(f'  pin {name}.{pname}: set to {val:X}')

    def change_pins(self, changes):
        for pname, val in changes.items():
            self.comp.set_pin_val(pname, val)
            print(f'  pin {pname}: set to {val:X}')

    def effect_changes(self, changes):
        print('*** Effecting changes:')
        if changes is None:
            return
        for key, val in changes.items():
            getattr(self, f'change_{key}')(val)

    #
    # Update
    #
    def update_obj(self):
        print('*** Running update()')
        self.obj.update()
        for drive in self.obj.drives:
            print(f'*** Faking {drive.drive_name} next state')
            mode = self.drive_pin_val(drive, 'drive-mode-cmd')
            self.drive_pin_val(drive, 'drive-mode-fb', mode)
            cw = self.drive_pin_val(drive, 'control-word')
            self.drive_pin_val(drive, 'control-word-fb', cw)
            drive.sim_fake_next_inputs()
            sw = self.drive_pin_val(drive, 'status-word-sim')
            self.drive_pin_val(drive, 'status-word', sw)
            name = drive.drive_name
            print(f'{name}:  mode={mode}; cw=0x{cw:04X}; sw=0x{sw:04X}')

    #
    # Debug
    #
    def debug(self, debug):
        if not debug:
            return

        print('*** debugging:')
        for drive in self.obj.drives:
            for pname, pin in drive.pins.pin_dict.items():
                name = drive.drive_name
                hal_val = self.drive_pin_val(drive, pname)
                print(
                    f'  dbg {name}.{pname}:  HAL {hal_val} val {pin.val} '
                    f'old {pin.old_val}'
                )

    #
    # Main function
    #
    def test_update(self, obj, fpath):
        assert obj.sim is False

        with open(fpath('mgr_tests.yaml')) as f:
            test_config = yaml.safe_load(f)

        default_before_checks = dict()
        for test in test_config['tests']:
            self.print_iter_intro(test['desc'])
            self.check_obj('Before', test.get('before', default_before_checks))
            self.effect_changes(test.get('changes', None))
            self.update_obj()
            self.debug(test.get('debug', False))
            self.check_obj('After', test['after'])
            default_before_checks = test['after']
