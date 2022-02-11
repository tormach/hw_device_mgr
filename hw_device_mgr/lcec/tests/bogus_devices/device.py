from ...device import LCECDevice
from ....ethercat.tests.bogus_devices.device import (
    BogusEtherCATDevice,
    BogusEtherCATServo,
    BogusOtherCATServo,
    BogusEtherCATIO,
)
from .config import BogusLCECConfig


class BogusLCECDevice(LCECDevice, BogusEtherCATDevice):
    config_class = BogusLCECConfig
    category = "bogus_lcec_devices"


class BogusLCECServo(BogusLCECDevice, BogusEtherCATServo):
    name = "bogo_lcec_servo"
    product_code = 0xB0905060


class BogusLCECServo2(BogusLCECDevice, BogusOtherCATServo):
    name = "bogo_lcec_servo_2"
    product_code = 0xB0905061


class BogusLCECIO(BogusLCECDevice, BogusEtherCATIO):
    name = "bogo_lcec_io"
    product_code = 0xB0901060
