from ...device import CiA301SimDevice
from ....tests.bogus_devices.device import (
    BogusDevice,
    BogusV1ServoDevice,
    BogusV2ServoDevice,
    BogusV1IODevice,
)


# Define a few abstract device categories for sim models to inherit
# from
class BogusCiA301DeviceCategory(CiA301SimDevice, BogusDevice):
    vendor_id = 0xB090C0


class BogusV1CiA301ServoCategory(BogusCiA301DeviceCategory, BogusV1ServoDevice):
    pass


class BogusV2CiA301ServoCategory(BogusCiA301DeviceCategory, BogusV2ServoDevice):
    pass


class BogusV1CiA301IOCategory(BogusCiA301DeviceCategory, BogusV1IODevice):
    pass


# Define a few concrete (sim) device models in a category
class BogusCiA301Device(BogusCiA301DeviceCategory):
    category = "bogus_cia301_devices"


class BogusV1CiA301Servo(BogusCiA301Device, BogusV1CiA301ServoCategory):
    name = "bogo_v1_cia301_servo"
    product_code = 0xB0905010


class BogusV2CiA301Servo(BogusCiA301Device, BogusV2CiA301ServoCategory):
    name = "bogo_v2_cia301_servo"
    product_code = 0xB0905011


class BogusV1CiA301IO(BogusCiA301Device, BogusV1CiA301IOCategory):
    name = "bogo_v1_cia301_io"
    product_code = 0xB0901010
