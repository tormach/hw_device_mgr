from ...mgr import ROSSimHWDeviceMgr
from ....mgr.tests.bogus_devices.mgr import (
    HwMgrTestDevices,
    HwMgrTestElmoGold420,
    HwMgrTestElmoGold520,
    HwMgrTestInovanceIS620N,
    HwMgrTestInovanceSV660N,
    HwMgrTestEVEXCRE,
)
from ....tests.interface import DebugInterface


class ROSHWDeviceMgrTestCategory(ROSSimHWDeviceMgr):
    interface_class = DebugInterface
    category = "test_ros_hw_device_mgr"
    device_base_class = HwMgrTestDevices
    device_classes = (
        HwMgrTestElmoGold420,
        HwMgrTestElmoGold520,
        HwMgrTestInovanceIS620N,
        HwMgrTestInovanceSV660N,
        HwMgrTestEVEXCRE,
    )


class ROSHWDeviceMgrTest(ROSHWDeviceMgrTestCategory):
    name = "test_ros_hw_device_mgr"
