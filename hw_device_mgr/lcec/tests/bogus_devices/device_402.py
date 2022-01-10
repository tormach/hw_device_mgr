from ...device import LCECSimDevice
from ....cia_402.tests.bogus_devices.device import (
    BogusCiA402DeviceCategory,
    BogusV1CiA402ServoCategory,
    BogusV2CiA402ServoCategory,
)


class BogusLCEC402Device(LCECSimDevice, BogusCiA402DeviceCategory):
    category = "bogus_lcec_402_devices"
    xml_description_fname = "BogusServo.xml"
    vendor_id = 0xB090C0


class BogusV1LCEC402Servo(BogusLCEC402Device, BogusV1CiA402ServoCategory):
    name = "bogo_v1_lcec_402_servo"
    product_code = 0xB0905062


class BogusV2LCEC402Servo(BogusLCEC402Device, BogusV2CiA402ServoCategory):
    name = "bogo_v2_lcec_402_servo"
    product_code = 0xB0905063
