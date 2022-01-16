from ...mgr import SimROSEtherCATHWDeviceMgr
from ....devices.tests.test_devices_402 import (
    DevicesForTest,
    ElmoGold420ForTest,
    ElmoGold520ForTest,
    InovanceIS620NForTest,
    InovanceSV660NForTest,
)


class BogusHALROSHWDeviceMgr(SimROSEtherCATHWDeviceMgr):
    name = "bogus_hal_ros_hw_device_mgr"
    data_type_class = DevicesForTest.data_type_class
    device_base_class = DevicesForTest
    device_classes = (
        ElmoGold420ForTest,
        ElmoGold520ForTest,
        InovanceIS620NForTest,
        InovanceSV660NForTest,
    )
