from ..mgr.mgr import HWDeviceMgr, SimHWDeviceMgr
from ..hal.device import HALCompDevice, HALPinDevice, HALPinSimDevice


class HALHWDeviceMgr(HWDeviceMgr, HALCompDevice):
    """Hardware device manager with HAL pins."""

    hal_comp_name = "hw_device_mgr"
    data_type_class = HALCompDevice.data_type_class
    device_base_class = HALPinDevice

    def init_devices(self, **kwargs):
        super().init_devices(**kwargs)
        self.hal_comp_ready()

    def init_device_instances(self, **kwargs):
        super().init_device_instances(comp=self.comp, **kwargs)


class HALSimHWDeviceMgr(HALHWDeviceMgr, SimHWDeviceMgr, HALPinSimDevice):
    """Hardware device manager with HAL pins."""

    device_base_class = HALPinSimDevice
