from ...mgr import HALHWDeviceMgr
from ....lcec.tests.bogus_devices.device_402 import (
    BogusLCEC402Device,
    BogusLCEC402Servo,
    BogusLCEC402Servo2,
)


class BogusLCEC402HWDeviceMgr(HALHWDeviceMgr):
    data_type_class = BogusLCEC402Device.data_type_class
    device_base_class = BogusLCEC402Device
    device_classes = (BogusLCEC402Servo, BogusLCEC402Servo2)

    name = "bogus_lcec_hw_device_mgr"
