from ...device import ErrorSimDevice
from ....tests.interface import DebugInterface


class BogusErrorDevice(ErrorSimDevice):
    interface_class = DebugInterface
    category = "bogus_error_devices"
    device_error_package = "hw_device_mgr.devices.device_err"

    @classmethod
    def scan_devices(cls, **kwargs):
        return list()


class BogusErrorV1Servo(BogusErrorDevice):
    name = "bogus_v1_error_servo"
    test_category = "bogus_v1_servo"
    model_id = 0xB0905041
    device_error_yaml = "bogus_v1_v2_error_servo.yaml"


class BogusErrorV2Servo(BogusErrorDevice):
    name = "bogus_v2_error_servo"
    test_category = "bogus_v2_servo"
    model_id = 0xB0905042
    device_error_yaml = "bogus_v1_v2_error_servo.yaml"


class BogusErrorV1IO(BogusErrorDevice):
    name = "bogus_v1_error_io"
    test_category = "bogus_v1_io"
    model_id = 0xB0901041
    device_error_yaml = "bogus_v1_error_io.yaml"


class BogusErrorV1JBox(BogusErrorDevice):
    name = "bogus_v1_error_jbox"
    test_category = "bogus_v1_jbox"
    model_id = 0xB0908041
    device_error_yaml = "unpopulated.yaml"
