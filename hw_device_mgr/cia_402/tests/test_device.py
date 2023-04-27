from .base_test_class import BaseCiA402TestClass
from ...cia_301.tests.test_device import TestCiA301Device as _TestCiA301Device


class TestCiA402Device(_TestCiA301Device, BaseCiA402TestClass):
    # expected_mro is the same as CiA301Device, because this test class also
    # tests non-CiA402Device classes to ensure a realistic mix won't break, and
    # to simplify tests & test cases

    def test_cw_to_str(self, cia402_cls):
        if not cia402_cls:
            return
        tests = {
            0x0000: "SWITCH ON DISABLED flags: (none)",
            0x0002: "QUICK STOP ACTIVE flags: (none)",
            0x0006: "READY TO SWITCH ON flags: (none)",
            0x0007: "SWITCHED ON flags: (none)",
            0x000F: "OPERATION ENABLED flags: (none)",
            0x001F: "OPERATION ENABLED flags: OPERATION_MODE_SPECIFIC_1",
            0x0080: "CLEAR FAULT flags: (none)",
            0x002F: "OPERATION ENABLED flags: OPERATION_MODE_SPECIFIC_2",
            0x030F: "OPERATION ENABLED flags: HALT,NA_1",
        }
        for cw, cw_str in tests.items():
            expected = f"0x{cw:04X} {cw_str}"
            print(f'expected:  0x{cw:04X} : "{expected}",')
            actual = cia402_cls.cw_to_str(cw)
            print(f"actual:  {actual}")
            assert actual == expected

    def test_sw_to_str(self, cia402_cls):
        if not cia402_cls:
            return
        tests = {
            0x0010: "NOT READY TO SWITCH ON flags: VOLTAGE_ENABLED",
            0x0050: "SWITCH ON DISABLED flags: VOLTAGE_ENABLED",
            0x0450: "SWITCH ON DISABLED flags: VOLTAGE_ENABLED,TARGET_REACHED",
            0x0031: "READY TO SWITCH ON flags: VOLTAGE_ENABLED",
            0x0431: "READY TO SWITCH ON flags: VOLTAGE_ENABLED,TARGET_REACHED",
            0x0033: "SWITCHED ON flags: VOLTAGE_ENABLED",
            0x0037: "OPERATION ENABLED flags: VOLTAGE_ENABLED",
            0x9037: "OPERATION ENABLED flags: VOLTAGE_ENABLED,OPERATION_MODE_SPECIFIC_1,MANUFACTURER_SPECIFIC_3",
            0x0017: "QUICK STOP ACTIVE flags: VOLTAGE_ENABLED",
        }
        for sw, sw_str in tests.items():
            expected = f"0x{sw:04X} {sw_str}"
            print(f'expected:  0x{sw:04X} : "{expected}",')
            actual = cia402_cls.sw_to_str(sw)
            print(f"actual:  {actual}")
            assert actual == expected

    def read_update_write_conv_test_data(self):
        if not self.is_402_device:
            return
        uint16 = self.device_class.data_type_class.uint16
        for data in (self.test_data, self.ovr_data):
            for intf, intf_data in data.items():
                # Format status_word, control_word for readability, e.g. 0x000F
                for key in ("status_word", "control_word"):
                    if key in intf_data:
                        intf_data[key] = uint16(intf_data[key])

    def setup_test(self, test_case):
        super().setup_test(test_case)
        # Add SV660N feedback_out home_found exception
        mno = self.missing_not_ok["feedback_out"]
        mno_home_found = mno.setdefault("home_found", set())
        mno_home_found.add((0x00100000, 0x000C010D))

    def test_read_update_write(self, obj):
        if hasattr(obj, "MODE_CSP"):
            # CiA 402 device
            self.read_update_write_package = self.read_update_write_402_package
            self.read_update_write_yaml = self.read_update_write_402_yaml
            self.is_402_device = True
        else:
            self.is_402_device = False
        super().test_read_update_write(obj)

    def test_hm_timeout(self, obj, mock_time):
        # Only applies to CiA402 devices
        if not hasattr(obj, "MODE_CSP"):
            return
        self.is_402_device = True

        # Read test case YAML
        self.read_update_write_package = self.read_update_write_402_package
        self.read_update_write_yaml = "hm_timeout.cases.yaml"
        test_cases = self.load_test_cases()

        # Timeouts for test
        start_time = 10000
        self.now = start_time
        obj.feedback_in.update(oper=True, online=True)  # Normal timeout cond.
        initial_timeout = obj.goal_reached_timeout
        assert obj.home_timeout > initial_timeout  # Sanity

        # Test case 0:  from CSP enabled, command HM mode
        self.read_update_write_loop(test_cases[0])
        assert obj.command_out.get("control_mode") == obj.MODE_HM

        # Test case 1:  HOMING_START command to drive
        self.read_update_write_loop(test_cases[1])
        assert obj.command_out.get("control_word") == 0x001F  # HOMING_START
        assert not obj.feedback_out.get("fault")  # No timeout

        # Test case 2:  Homing in progress (no time increment))
        self.read_update_write_loop(test_cases[2])
        assert not obj.feedback_out.get("fault")  # No timeout

        # Test case 2:  Homing in progress (small time increment)
        self.now = start_time + initial_timeout + 1
        self.read_update_write_loop(test_cases[2])
        assert not obj.feedback_out.get("fault")  # No timeout

        # Test case 3:  Homing timeout exceeded
        self.now = start_time + obj.home_timeout + 1
        self.read_update_write_loop(test_cases[3])
        assert obj.feedback_out.get("fault")  # Timeout

    def test_pp_timeout(self, obj, mock_time):
        # Only applies to CiA402 devices
        if not hasattr(obj, "MODE_CSP"):
            return
        self.is_402_device = True

        # Read test case YAML
        self.read_update_write_package = self.read_update_write_402_package
        self.read_update_write_yaml = "pp_timeout.cases.yaml"
        test_cases = self.load_test_cases()

        # Timeouts for test
        start_time = 10000
        self.now = start_time
        obj.feedback_in.update(oper=True, online=True)  # Normal timeout cond.
        initial_timeout = obj.goal_reached_timeout
        assert obj.move_timeout > initial_timeout + 1  # Sanity

        # Test case 0:  Command PP move
        self.read_update_write_loop(test_cases[0])
        assert obj.command_out.get("control_mode") == obj.MODE_PP

        # Test case 1:  NEW_SETPOINT command to drive
        self.read_update_write_loop(test_cases[1])
        assert not obj.feedback_out.get("fault")  # No timeout

        # Test case 2:  SETPOINT_ACKNOWLEDGE feedback (no time increment)
        self.read_update_write_loop(test_cases[2])
        assert not obj.feedback_out.get("fault")  # No timeout

        # Test case 3:  Move in progress (small time increment)
        self.now = start_time + initial_timeout + 1
        self.read_update_write_loop(test_cases[3])
        assert not obj.feedback_out.get("fault")  # No timeout

        # Test case 3:  Move in progress (small time increment)
        self.now = start_time + initial_timeout + 1
        self.read_update_write_loop(test_cases[3])
        assert not obj.feedback_out.get("fault")  # No timeout

        # Test case 4:  Move timeout exceeded
        self.now = start_time + obj.move_timeout + 1
        self.read_update_write_loop(test_cases[4])
        assert obj.feedback_out.get("fault")  # Timeout
