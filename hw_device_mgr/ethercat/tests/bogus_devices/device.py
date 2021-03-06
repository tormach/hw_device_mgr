from ..relocatable_esi_device import RelocatableESIDevice
from ....cia_402.device import CiA402SimDevice


class BogusEtherCATDevice(RelocatableESIDevice):
    category = "bogus_ethercat_devices"
    vendor_id = 0xB090C0


class BogusEtherCATServo(BogusEtherCATDevice, CiA402SimDevice):
    name = "bogo_ethercat_servo"
    test_category = "bogus_v1_servo"
    product_code = 0xB0905030
    xml_description_fname = "BogusServo.xml"


class BogusOtherCATServo(BogusEtherCATDevice, CiA402SimDevice):
    name = "bogo_Othercat_servo"
    test_category = "bogus_v2_servo"
    product_code = 0xB0905031
    xml_description_fname = "BogusServo.xml"


class BogusEtherCATIO(BogusEtherCATDevice):
    name = "bogo_ethercat_io"
    test_category = "bogus_v1_io"
    product_code = 0xB0901030
    xml_description_fname = "BogusIO.xml"
