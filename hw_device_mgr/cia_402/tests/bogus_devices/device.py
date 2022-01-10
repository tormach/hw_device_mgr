from ...device import CiA402SimDevice
from ....cia_301.tests.bogus_devices.device import (
    BogusCiA301DeviceCategory,
    BogusV1CiA301ServoCategory,
    BogusV2CiA301ServoCategory,
)


# Define a few abstract device categories for sim models to inherit
# from
class BogusCiA402DeviceCategory(CiA402SimDevice, BogusCiA301DeviceCategory):
    pass


class BogusV1CiA402ServoCategory(
    BogusCiA402DeviceCategory, BogusV1CiA301ServoCategory
):
    pass


class BogusV2CiA402ServoCategory(
    BogusCiA402DeviceCategory, BogusV2CiA301ServoCategory
):
    pass


# Define a few concrete (sim) device models in a category
class BogusCiA402Device(BogusCiA402DeviceCategory):
    category = "bogus_cia402_devices"


class BogusV1CiA402Servo(BogusCiA402Device, BogusV1CiA402ServoCategory):
    name = "bogo_v2_cia402_servo"
    product_code = 0xB0905020


class BogusV2CiA402Servo(BogusCiA402Device, BogusV2CiA402ServoCategory):
    name = "bogo_v2_cia402_servo"
    product_code = 0xB0905021
