from ..mgr_hal.mgr import HALHWDeviceMgr
from ..mgr_ros.mgr import ROSHWDeviceMgr
from .devices import ManagedDevices


class ROSHALHWDeviceMgr(ROSHWDeviceMgr, HALHWDeviceMgr):

    data_type_class = ManagedDevices.data_type_class
    device_base_class = ManagedDevices
    device_classes = ManagedDevices.get_model()
