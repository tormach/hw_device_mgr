from ...device import SimDevice
from ..interface import DebugInterface


class BogusDevice(SimDevice):
    """Bogus-Bus device class."""

    interface_class = DebugInterface
    category = "bogus_bus_device"


class BogusV1Servo(BogusDevice):
    name = "bogo_v1_servo"
    test_category = "bogus_v1_servo"
    model_id = 0xB0905000


class BogusV2Servo(BogusDevice):
    name = "bogo_v2_servo"
    test_category = "bogus_v2_servo"
    model_id = 0xB0905001


class BogusV1IO(BogusDevice):
    name = "bogo_v1_io"
    test_category = "bogus_v1_io"
    model_id = 0xB0901000


class BogusV1JBox(BogusDevice):
    name = "bogo_v1_jbox"
    test_category = "bogus_v1_jbox"
    model_id = 0xB0908000
