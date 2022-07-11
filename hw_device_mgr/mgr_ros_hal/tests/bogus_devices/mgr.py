from ...mgr import ROSHALSimHWDeviceMgr
from ....mgr_hal.tests.bogus_devices.mgr import HALHWDeviceMgrTestCategory
from ....tests.interface import DebugInterface


class ROSHWDeviceMgrTestCategory(ROSHALSimHWDeviceMgr):
    interface_class = DebugInterface
    category = "test_ros_hal_hw_device_mgr"
    data_type_class = HALHWDeviceMgrTestCategory.data_type_class
    device_base_class = HALHWDeviceMgrTestCategory.device_base_class
    device_classes = HALHWDeviceMgrTestCategory.device_classes


class ROSHWDeviceMgrTest(ROSHWDeviceMgrTestCategory):
    name = "test_ros_hal_hw_device_mgr"
