from ...device import LCECDevice
from ....ethercat.tests.bogus_devices.device_402 import (
    BogusEtherCAT402Device,
    BogusEtherCAT402Servo,
    BogusOtherCAT402Servo,
)
from .config import BogusLCECConfig


class BogusLCEC402Device(LCECDevice, BogusEtherCAT402Device):
    config_class = BogusLCECConfig
    category = "bogus_lcec_402_devices"


class BogusLCEC402Servo(BogusLCEC402Device, BogusEtherCAT402Servo):
    name = "bogo_lcec_402_servo"
    product_code = 0xB0905062


class BogusLCEC402Servo2(BogusLCEC402Device, BogusOtherCAT402Servo):
    name = "bogo_lcec_402_servo_2"
    product_code = 0xB0905063
