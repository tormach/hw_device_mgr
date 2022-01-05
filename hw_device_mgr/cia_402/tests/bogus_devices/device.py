from ....cia_301.tests.bogus_devices.device import (
    BogusCiA301Device,
    BogusCiA301Servo,
    BogusCiA301Servo2,
)
from ...device import CiA402Device


class BogusCiA402Device(CiA402Device, BogusCiA301Device):
    category = "bogus_cia402_devices"


class BogusCiA402Servo(BogusCiA402Device, BogusCiA301Servo):
    name = "bogo_cia402_servo"
    product_code = 0xB0905020


class BogusCiA402Servo2(BogusCiA402Device, BogusCiA301Servo2):
    name = "bogo_cia402_servo_2"
    product_code = 0xB09050201
