from ...device import CiA301SimDevice
from ....tests.interface import DebugInterface


# Categories
class BogusCiA301Device(CiA301SimDevice):
    """Bogo Co CANopen 301 device class."""

    interface_class = DebugInterface
    category = "bogus_cia301_device"
    vendor_id = 0xB090C0


class BogusCiA301V1Servo(BogusCiA301Device):
    name = "bogo_cia301_v1_servo"
    test_category = "bogus_v1_servo"
    product_code = 0xB0905010


class BogusCiA301V2Servo(BogusCiA301Device):
    name = "bogo_cia301_v2_servo"
    test_category = "bogus_v2_servo"
    product_code = 0xB0905011


class BogusCiA301V1IO(BogusCiA301Device):
    name = "bogo_cia301_v1_io"
    test_category = "bogus_v1_io"
    product_code = 0xB0901010
