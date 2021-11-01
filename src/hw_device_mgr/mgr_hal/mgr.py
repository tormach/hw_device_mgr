from ..mgr.mgr import HWDeviceMgr
from ..hal.device import HALCompDevice, HALPinDevice, HALDataType


class HALHWDeviceMgr(HWDeviceMgr, HALCompDevice, HALPinDevice):
    data_type_class = HALDataType
    device_base_class = HALPinDevice
    device_classes = (HALPinDevice,)

    def init_devices(self, **kwargs):
        super().init_devices(**kwargs)
        self.hal_comp_ready()

    def init_device_instances(self, **kwargs):
        super().init_device_instances(comp=self.comp, **kwargs)
