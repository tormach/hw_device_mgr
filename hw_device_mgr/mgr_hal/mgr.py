from ..mgr.mgr import HWDeviceMgr, SimHWDeviceMgr
from ..hal.device import HALCompDevice, HALPinDevice, HALPinSimDevice


class HALHWDeviceMgr(HALCompDevice, HWDeviceMgr):
    """Hardware device manager with HAL pins."""

    # Inherit from `HALCompDevice` first to ensure `HALCompDevice.init()` sets
    # `self.comp` first, `HWDeviceMgr.init()` calls `HALPinDevice.init()` then
    # `HWDeviceMgr.init_devices()`, then returns to `HALCompDevice.init()` to
    # call `comp.ready()`.

    hal_comp_name = "hw_device_mgr"
    device_base_class = HALPinDevice

    pin_interfaces = dict(
        feedback_in=(HALPinDevice.HAL_IN, ""),
        feedback_out=(HALPinDevice.HAL_OUT, ""),
        command_in=(HALPinDevice.HAL_IN, ""),
        command_out=(HALPinDevice.HAL_OUT, ""),
    )

    def init_devices(self, /, device_init_kwargs=dict(), **kwargs):
        device_init_kwargs["comp"] = self.comp
        super().init_devices(device_init_kwargs=device_init_kwargs, **kwargs)

    def set_command(self):
        # Read command from HAL pins
        # FIXME this needs to be better formalized
        pins = self.pins["command_in"]
        vals = {p: pins[p].get() for p in pins.keys()}
        super().set_command(**vals)


class HALSimHWDeviceMgr(HALHWDeviceMgr, SimHWDeviceMgr, HALPinSimDevice):
    """Hardware device manager with HAL pins."""

    device_base_class = HALPinSimDevice
