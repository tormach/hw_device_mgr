from .base_test_class import BaseLCEC402MgrTestClass
from ...mgr.tests.test_mgr import TestHWDeviceMgr as _TestHWDeviceMgr
from ...hal.tests.test_device import TestHALDevice as _TestHALDevice


class TestHALHWDeviceMgr(
    BaseLCEC402MgrTestClass, _TestHWDeviceMgr, _TestHALDevice
):
    expected_mro = [
        "BogusLCEC402HWDeviceMgr",
        "HALHWDeviceMgr",
        "HWDeviceMgr",
        "FysomGlobalMixin",
        "HALCompDevice",
        "HALPinDevice",
        "Device",
        "ABC",
        "HALMixin",
        "object",
    ]

    def override_interface_param(self, interface, key, val):
        match = self.drive_key_re.match(key)
        if match:
            index, key = match.groups()
            dev = self.obj.devices[int(index)]
            pname = dev.pin_name(interface, key)
            self.set_pin(pname, val)
        else:
            super().override_interface_param(interface, key, val)

    def copy_sim_feedback(self):
        super().copy_sim_feedback()
        for dev in self.obj.devices:
            super().copy_sim_feedback(obj=dev)

    def post_read_actions(self, obj=None):
        if obj is None:
            super().post_read_actions()
            print("  feedback_in pin values:")
            obj = self.obj

        for name in obj.feedback_in.get():
            pname = obj.pin_name("feedback_in", name)
            val = self.get_pin(pname)
            print(f"    {pname} = {val}")
            assert val == obj.feedback_in.get(pname)
        print()

    def check_halpin_values(self):
        super().check_halpin_values()
        for dev in self.obj.devices:
            super().check_halpin_values(obj=dev)
