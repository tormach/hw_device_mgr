from ..relocatable_esi_device import RelocatableESIDevice
from ....cia_402.device import CiA402SimDevice
from ....tests.interface import DebugInterface


class BogusEtherCATDevice(RelocatableESIDevice):
    interface_class = DebugInterface
    category = "bogus_ethercat_devices"
    vendor_id = 0xB090C0
    xml_description_package = "hw_device_mgr.devices.device_xml"
    device_error_package = "hw_device_mgr.devices.device_err"


class BogusEtherCATServo(BogusEtherCATDevice, CiA402SimDevice):
    name = "bogo_ethercat_servo"
    test_category = "bogus_v1_servo"
    product_code = 0xB0905030
    xml_description_fname = "BogusServo.xml"
    device_error_yaml = "bogus_v1_v2_error_servo.yaml"


class BogusOtherCATServo(BogusEtherCATDevice, CiA402SimDevice):
    name = "bogo_Othercat_servo"
    test_category = "bogus_v2_servo"
    product_code = 0xB0905031
    xml_description_fname = "BogusServo.xml"
    device_error_yaml = "bogus_v1_v2_error_servo.yaml"


class BogusEtherCATIO(BogusEtherCATDevice):
    name = "bogo_ethercat_io"
    test_category = "bogus_v1_io"
    product_code = 0xB0901030
    xml_description_fname = "BogusIO.xml"
    device_error_yaml = "bogus_v1_error_io.yaml"


class BogusEtherCATJBox(BogusEtherCATDevice):
    name = "bogo_ethercat_jbox"
    test_category = "bogus_v1_jbox"
    product_code = 0xB0908030
    xml_description_fname = "BogusJunction.xml"
    device_error_yaml = "unpopulated.yaml"
