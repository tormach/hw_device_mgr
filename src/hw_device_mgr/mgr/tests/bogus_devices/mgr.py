from ...mgr import HWDeviceMgr
from ....ethercat.tests.bogus_devices.device_402 import (
    BogusEtherCAT402Device,
    BogusEtherCAT402Servo,
    BogusOtherCAT402Servo,
)


class BogusHWDeviceMgr(HWDeviceMgr):
    device_base_class = BogusEtherCAT402Device
    device_classes = (
        BogusEtherCAT402Servo,
        BogusOtherCAT402Servo,
    )

    name = "bogus_hw_device_mgr"
