from ....ethercat.tests.relocatable_esi_device import RelocatableESIDevice
from ...device import LCECSimDevice
from ....cia_402.device import CiA402SimDevice
from ....tests.interface import DebugInterface


class BogusLCECDevice(LCECSimDevice, RelocatableESIDevice):
    interface_class = DebugInterface
    category = "bogus_lcec_devices"
    vendor_id = 0xB090C0
    xml_description_package = "hw_device_mgr.devices.device_xml"
    device_error_package = "hw_device_mgr.devices.device_err"


class BogusLCECV1Servo(BogusLCECDevice, CiA402SimDevice):
    name = "bogo_lcec_v1_servo"
    test_category = "bogus_v1_servo"
    product_code = 0xB0905060
    xml_description_fname = "BogusServo.xml"
    device_error_yaml = "bogus_v1_v2_error_servo.yaml"


class BogusLCECV2Servo(BogusLCECDevice, CiA402SimDevice):
    name = "bogo_lcec_v2_servo"
    test_category = "bogus_v2_servo"
    product_code = 0xB0905061
    xml_description_fname = "BogusServo.xml"
    device_error_yaml = "bogus_v1_v2_error_servo.yaml"


class BogusLCECV1IO(BogusLCECDevice):
    name = "bogo_lcec_v1_io"
    test_category = "bogus_v1_io"
    product_code = 0xB0901060
    xml_description_fname = "BogusIO.xml"
    device_error_yaml = "bogus_v1_error_io.yaml"


class BogusLCECV1JBox(BogusLCECDevice):
    name = "bogo_lcec_v1_jbox"
    test_category = "bogus_v1_jbox"
    product_code = 0xB0908060
    xml_description_fname = "BogusJunction.xml"
    device_error_yaml = "unpopulated.yaml"
