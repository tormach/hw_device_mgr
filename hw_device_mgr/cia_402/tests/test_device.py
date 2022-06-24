from .base_test_class import BaseCiA402TestClass
from ...cia_301.tests.test_device import TestCiA301Device as _TestCiA301Device


class TestCiA402Device(_TestCiA301Device, BaseCiA402TestClass):

    # expected_mro is the same as CiA301Device, because this test class also
    # tests non-CiA402Device classes to ensure a realistic mix won't break, and
    # to simplify tests & test cases

    def read_update_write_conv_test_data(self):
        if "error_code" in self.obj.feedback_out.get():
            # Account for some devices that inherit from ErrorDevice
            self.test_data["feedback_in"].setdefault("error_code", 0x00000000)
            self.test_data["feedback_out"].setdefault("error_code", 0x00000000)
            self.test_data["feedback_out"].setdefault("description", "No error")
            self.test_data["feedback_out"].setdefault("advice", "No error")

        if not self.is_402_device:
            return
        uint16 = self.device_class.data_type_class.uint16
        for data in (self.test_data, self.ovr_data):
            for intf, intf_data in data.items():
                # Translate control_mode, e.g. MODE_CSP -> 8
                cm = intf_data.get("control_mode", None)
                if cm is not None and intf == "command_out":
                    cm = self.obj.control_mode_int(cm)
                    intf_data["control_mode"] = cm
                cmf = intf_data.get("control_mode_fb", None)
                if cmf is not None and intf in ("feedback_in", "sim_feedback"):
                    cmf = self.obj.control_mode_int(cmf)
                    intf_data["control_mode_fb"] = cmf

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
