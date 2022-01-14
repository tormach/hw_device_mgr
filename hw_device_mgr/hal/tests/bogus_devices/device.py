from ...device import HALPinDevice
from ....cia_301.tests.bogus_devices.device import (
    BogusCiA301DeviceCategory,
    BogusV1CiA301ServoCategory,
    BogusV2CiA301ServoCategory,
    BogusV1CiA301IOCategory,
)


class BogusHALDevice(HALPinDevice, BogusCiA301DeviceCategory):
    category = "bogus_hal_device"
    vendor_id = 0xB090C0


class BogusV1HALServo(BogusHALDevice, BogusV1CiA301ServoCategory):
    name = "bogo_v1_hal_servo"
    product_code = 0xB0905050


class BogusV2HALServo(BogusHALDevice, BogusV2CiA301ServoCategory):
    name = "bogo_v2_hal_servo"
    product_code = 0xB0905051


class BogusV1HALIO(BogusHALDevice, BogusV1CiA301IOCategory):
    name = "bogo_v1_hal_io"
    product_code = 0xB0901050
