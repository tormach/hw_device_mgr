from ..mgr.mgr import HWDeviceMgr, SimHWDeviceMgr
from ..hal.device import HALCompDevice, HALPinDevice, HALPinSimDevice


class HALHWDeviceMgr(HWDeviceMgr, HALCompDevice):
    """Hardware device manager with HAL pins."""

    hal_comp_name = "hw_device_mgr"
    data_type_class = HALCompDevice.data_type_class
    device_base_class = HALPinDevice

    def init_devices(self, /, device_init_kwargs=dict(), **kwargs):
        device_init_kwargs["comp"] = self.comp
        super().init_devices(device_init_kwargs=device_init_kwargs, **kwargs)


class HALSimHWDeviceMgr(HALHWDeviceMgr, SimHWDeviceMgr, HALPinSimDevice):
    """Hardware device manager with HAL pins."""

    device_base_class = HALPinSimDevice
