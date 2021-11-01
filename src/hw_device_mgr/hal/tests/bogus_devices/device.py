from ....cia_301.tests.bogus_devices.device import (
    BogusCiA301Device,
    BogusCiA301Servo,
    BogusCiA301Servo2,
    BogusCiA301IO,
)
from ...device import HALPinDevice


class BogusHALDevice(HALPinDevice, BogusCiA301Device):
    category = "bogus_hal_device"


class BogusHALServo(BogusHALDevice, BogusCiA301Servo):
    name = "bogo_hal_servo"
    product_code = 0xB0905050


class BogusHALServo2(BogusHALDevice, BogusCiA301Servo2):
    name = "bogo_hal_servo_2"
    product_code = 0xB0905051


class BogusHALIO(BogusHALDevice, BogusCiA301IO):
    name = "bogo_hal_io"
    product_code = 0xB0901050
