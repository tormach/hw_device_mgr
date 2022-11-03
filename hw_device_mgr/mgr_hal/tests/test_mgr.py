from .base_test_class import BaseHALMgrTestClass
from ...mgr.tests.test_mgr import TestHWDeviceMgr as _TestHWDeviceMgr
from ...hal.tests.test_device import TestHALDevice as _TestHALDevice


class TestHALHWDeviceMgr(BaseHALMgrTestClass, _TestHWDeviceMgr, _TestHALDevice):
    expected_mro = [
        "HALHWDeviceMgrTestCategory",
        "HALSimHWDeviceMgr",
        "HALHWDeviceMgr",
        "HALCompDevice",  # HAL comp (this should be tested, too!)
        _TestHWDeviceMgr.expected_mro[1],  # SimHWDeviceMgr
        *_TestHALDevice.expected_mro[:2],  # HALPinSimDevice...HALPinDevice
        *_TestHWDeviceMgr.expected_mro[2:],  # HWDeviceMgr...ABC
        _TestHALDevice.expected_mro[-1],  # HalMixin (skip CiA301, etc.)
    ]

    def override_interface_param(self, interface, ovr_data):
        for key, val in ovr_data.items():
            match = self.test_case_key_re.match(key)
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
            obj = self.obj
            super().post_read_actions()
            for dev in obj.devices:
                self.post_read_actions(dev)

        print(f"  feedback_in pin values:  {obj}")
        for name in obj.pins["feedback_in"]:
            pname = obj.pin_name("feedback_in", name)
            val = self.get_pin(pname)
            print(f"    {pname} = {val}")
            assert val == obj.feedback_in.get(name)
        print()

    def munge_interface_data(self, interface):
        # `HALDevice.set_command()` takes no arguments, and reads from HAL pins
        # instead.
        data = super().munge_interface_data(interface)
        if interface != "command_in":
            return data  # Do the usual thing for other interfaces
        # For `command_in`, write HAL pins
        print("      (Setting command_in HAL pins)")
        for key, val in data.items():
            self.set_pin(key, val)
        return dict()

    def post_write_actions(self):
        super().post_write_actions()
        for dev in self.obj.devices:
            for iface in dev.pins:
                if dev.pin_interfaces[iface][0] == dev.HAL_OUT:
                    self.check_halpin_values(iface, dev)
