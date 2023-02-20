from .base_test_class import BaseCiA402TestClass
from ...cia_301.tests.test_device import TestCiA301Device as _TestCiA301Device


class TestCiA402Device(_TestCiA301Device, BaseCiA402TestClass):

    # expected_mro is the same as CiA301Device, because this test class also
    # tests non-CiA402Device classes to ensure a realistic mix won't break, and
    # to simplify tests & test cases

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

        # Test case 2:  Move in progress (no time increment)
        self.read_update_write_loop(test_cases[2])
        assert not obj.feedback_out.get("fault")  # No timeout

        # Test case 2:  Move in progress (small time increment)
        self.now = start_time + initial_timeout + 1
        self.read_update_write_loop(test_cases[2])
        assert not obj.feedback_out.get("fault")  # No timeout

        # Test case 3:  Move timeout exceeded
        self.now = start_time + obj.move_timeout + 1
        self.read_update_write_loop(test_cases[3])
        assert obj.feedback_out.get("fault")  # Timeout
