from ...device import EtherCATSimDevice
from ....cia_301.tests.bogus_devices.device import (
    BogusCiA301DeviceCategory,
    BogusV1CiA301ServoCategory,
    BogusV2CiA301ServoCategory,
    BogusV1CiA301IOCategory,
)


class BogusEtherCATDevice(EtherCATSimDevice, BogusCiA301DeviceCategory):
    category = "bogus_ethercat_devices"


class BogusEtherCATServo(BogusEtherCATDevice, BogusV1CiA301ServoCategory):
    name = "bogo_ethercat_servo"
    product_code = 0xB0905030
    xml_description_fname = "BogusServo.xml"


class BogusOtherCATServo(BogusEtherCATDevice, BogusV2CiA301ServoCategory):
    name = "bogo_Othercat_servo"
    product_code = 0xB0905031
    xml_description_fname = "BogusServo.xml"


class BogusEtherCATIO(BogusEtherCATDevice, BogusV1CiA301IOCategory):
    name = "bogo_ethercat_io"
    product_code = 0xB0901030
    xml_description_fname = "BogusIO.xml"
