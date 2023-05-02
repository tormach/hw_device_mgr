from ...device import CiA402SimDevice
from ....cia_301.device import CiA301SimDevice
from ....tests.interface import DebugInterface


class BogusCiA402Device(CiA301SimDevice):
    """Bogo Co CANopen 402 (except IO) device class."""

    interface_class = DebugInterface
    category = "bogus_cia402_devices"
    vendor_id = 0xB090C0
    device_error_package = "hw_device_mgr.devices.device_err"
    device_error_yaml = "unpopulated.yaml"


class BogusCiA402V1Servo(BogusCiA402Device, CiA402SimDevice):
    name = "bogo_cia402_v1_servo"
    test_category = "bogus_v1_servo"
    product_code = 0xB0905020


class BogusCiA402V2Servo(BogusCiA402Device, CiA402SimDevice):
    name = "bogo_cia402_v2_servo"
    test_category = "bogus_v2_servo"
    product_code = 0xB0905021


class BogusCiA402V1IO(BogusCiA402Device):
    name = "bogo_cia402_v1_io"
    test_category = "bogus_v1_io"
    product_code = 0xB0901020


class BogusCiA402V1JBox(BogusCiA402Device):
    name = "bogo_cia402_v1_jbox"
    test_category = "bogus_v1_jbox"
    product_code = 0xB0908020
