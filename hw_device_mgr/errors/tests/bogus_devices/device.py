from ...device import ErrorDevice
from ....tests.bogus_devices.device import (
    BogusDevice,
    BogusV1ServoDevice,
    BogusV2ServoDevice,
    BogusV1IODevice,
)


class BogusErrorDevice(ErrorDevice, BogusDevice):
    category = "bogus_error_devices"

    @classmethod
    def scan_devices(cls, **kwargs):
        return list()


class BogusV1ErrorServo(BogusErrorDevice, BogusV1ServoDevice):
    name = "bogus_v1_error_servo"
    model_id = 0xB0905041


class BogusV2ErrorServo(BogusErrorDevice, BogusV2ServoDevice):
    name = "bogus_v2_error_servo"
    model_id = 0xB0905042


class BogusV1ErrorIO(BogusErrorDevice, BogusV1IODevice):
    name = "bogus_v1_error_io"
    model_id = 0xB0901041
