from ..mgr_hal.mgr import HALHWDeviceMgr
from ..mgr_ros.mgr import ROSHWDeviceMgr, SimROSHWDeviceMgr
from .devices import (
    ManagedDevices,
    SimManagedLCECDevices,
    SimManagedEtherCATDevices,
)


class ROSHALHWDeviceMgr(ROSHWDeviceMgr, HALHWDeviceMgr):
    name = "ros_hal_hw_device_mgr"
    data_type_class = ManagedDevices.data_type_class
    device_base_class = ManagedDevices
    device_classes = ManagedDevices.get_model()


class SimROSLCECHWDeviceMgr(ROSHALHWDeviceMgr, SimROSHWDeviceMgr):

    name = "sim_ros_lcec_hw_device_mgr"
    device_base_class = SimManagedLCECDevices
    device_classes = SimManagedLCECDevices.get_model()


class SimROSEtherCATHWDeviceMgr(ROSHALHWDeviceMgr, SimROSHWDeviceMgr):

    name = "sim_ros_ethercat_hw_device_mgr"
    device_base_class = SimManagedEtherCATDevices
    device_classes = SimManagedEtherCATDevices.get_model()
