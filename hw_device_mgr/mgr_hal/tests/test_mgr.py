from .base_test_class import BaseHALMgrTestClass
from ...mgr.tests.test_mgr import TestHWDeviceMgr as _TestHWDeviceMgr
from ...hal.tests.test_device import TestHALDevice as _TestHALDevice


class TestHALHWDeviceMgr(BaseHALMgrTestClass, _TestHWDeviceMgr, _TestHALDevice):
    expected_mro = [
        "HALHWDeviceMgrTestCategory",
        "HALSimHWDeviceMgr",
        "HALHWDeviceMgr",
        *_TestHWDeviceMgr.expected_mro[1:4],  # SimHWDeviceMgr..FysomGlobalMixin
        "HALCompDevice",  # HAL comp (this should be tested, too!)
        *_TestHALDevice.expected_mro[:2],  # HALPinSim...HALPin
        *_TestHWDeviceMgr.expected_mro[4:],  # SimDevice...ABC
        _TestHALDevice.expected_mro[-1],  # HalMixin
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
