from ...device import CiA301Device
from .config import BogusCiA301Config


class BogusCiA301Device(CiA301Device):
    config_class = BogusCiA301Config
    category = "bogus_cia301_devices"


class BogusCiA301Servo(BogusCiA301Device):
    name = "bogo_cia301_servo"
    vendor_id = 0xB090C0
    product_code = 0xB0905010
    model_key = "bogus_servo"  # For test fixtures


class BogusCiA301Servo2(BogusCiA301Device):
    name = "bogo_cia301_servo_2"
    vendor_id = 0xB090C0
    product_code = 0xB0905011
    model_key = "bogus_servo_2"  # For test fixtures


class BogusCiA301IO(BogusCiA301Device):
    name = "bogo_cia301_io"
    vendor_id = 0xB090C0
    product_code = 0xB0901010
    model_key = "bogus_io"  # For test fixtures
