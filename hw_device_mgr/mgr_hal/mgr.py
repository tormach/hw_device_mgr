from ..mgr.mgr import HWDeviceMgr, SimHWDeviceMgr
from ..hal.device import HALCompDevice, HALPinDevice, HALDataType


class HALHWDeviceMgr(HWDeviceMgr, HALCompDevice, HALPinDevice):

    name = "hal_hw_device_mgr"
    hal_comp_name = "hw_device_mgr"
    data_type_class = HALDataType
    device_base_class = HALPinDevice
    device_classes = (HALPinDevice,)

    def init_devices(self, **kwargs):
        super().init_devices(**kwargs)
        self.hal_comp_ready()

    def init_device_instances(self, **kwargs):
        super().init_device_instances(comp=self.comp, **kwargs)


class SimHALHWDeviceMgr(HALHWDeviceMgr, SimHWDeviceMgr):
    name = "sim_hal_hw_device_mgr"
