from ...device import LCECSimDevice
from ....cia_301.tests.bogus_devices.device import (
    BogusCiA301DeviceCategory,
    BogusV1CiA301ServoCategory,
    BogusV2CiA301ServoCategory,
    BogusV1CiA301IOCategory,
)


class BogusLCECDevice(LCECSimDevice, BogusCiA301DeviceCategory):
    category = "bogus_lcec_devices"


class BogusV1LCECServo(BogusLCECDevice, BogusV1CiA301ServoCategory):
    name = "bogo_v1_lcec_servo"
    product_code = 0xB0905060
    xml_description_fname = "BogusServo.xml"


class BogusV2LCECServo(BogusLCECDevice, BogusV2CiA301ServoCategory):
    name = "bogo_v2_lcec_servo"
    product_code = 0xB0905061
    xml_description_fname = "BogusServo.xml"


class BogusV1LCECIO(BogusLCECDevice, BogusV1CiA301IOCategory):
    name = "bogo_v1_lcec_io"
    product_code = 0xB0901060
    xml_description_fname = "BogusIO.xml"
