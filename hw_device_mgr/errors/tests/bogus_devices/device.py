from ...device import ErrorSimDevice


class BogusErrorDevice(ErrorSimDevice):
    category = "bogus_error_devices"

    @classmethod
    def scan_devices(cls, **kwargs):
        return list()


class BogusErrorV1Servo(BogusErrorDevice):
    name = "bogus_v1_error_servo"
    test_category = "bogus_v1_servo"
    model_id = 0xB0905041


class BogusErrorV2Servo(BogusErrorDevice):
    name = "bogus_v2_error_servo"
    test_category = "bogus_v2_servo"
    model_id = 0xB0905042


class BogusErrorV1IO(BogusErrorDevice):
    name = "bogus_v1_error_io"
    test_category = "bogus_v1_io"
    model_id = 0xB0901041
