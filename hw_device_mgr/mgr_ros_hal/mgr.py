from ..mgr_ros.mgr import ROSHWDeviceMgr, ROSSimHWDeviceMgr
from ..mgr_hal.mgr import HALHWDeviceMgr, HALSimHWDeviceMgr

# from .devices import ManagedDevices, ManagedEtherCATSimDevices


class ROSHALHWDeviceMgr(ROSHWDeviceMgr, HALHWDeviceMgr):
    pass
    # data_type_class = ManagedDevices.data_type_class
    # device_base_class = ManagedDevices
    # device_classes = ManagedDevices.get_model()


class ROSHALSimHWDeviceMgr(
    ROSHALHWDeviceMgr, ROSSimHWDeviceMgr, HALSimHWDeviceMgr
):
    pass
    # device_base_class = ManagedEtherCATSimDevices
    # device_classes = ManagedEtherCATSimDevices.get_model()
