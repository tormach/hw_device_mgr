from ...device import HALPinSimDevice
from ....cia_301.device import CiA301SimDevice
from ....tests.interface import DebugInterface


class BogusHALDevice(HALPinSimDevice, CiA301SimDevice):
    interface_class = DebugInterface
    category = "bogus_hal_device"
    vendor_id = 0xB090C0


class BogusHALV1Servo(BogusHALDevice):
    name = "bogo_hal_v1_servo"
    test_category = "bogus_v1_servo"
    product_code = 0xB0905050


class BogusHALV2Servo(BogusHALDevice):
    name = "bogo_hal_v2_servo"
    test_category = "bogus_v2_servo"
    product_code = 0xB0905051


class BogusHALV1IO(BogusHALDevice):
    name = "bogo_hal_v1_io"
    test_category = "bogus_v1_io"
    product_code = 0xB0901050
