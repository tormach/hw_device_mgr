from ...device import ErrorDevice
from ....tests.bogus_devices.device import BogusDevice, BogusServo, BogusIO


class BogusErrorDevice(ErrorDevice, BogusDevice):
    category = "bogus_error_devices"

    @classmethod
    def scan_devices(cls, **kwargs):
        return list()


class BogusErrorServo(BogusErrorDevice, BogusServo):
    name = "bogus_error_servo"
    model_id = 0xB0905001


class BogusErrorIO(BogusErrorDevice, BogusIO):
    name = "bogus_error_io"
    model_id = 0xB0901001
