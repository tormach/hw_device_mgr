from ...device import SimDevice


class BogusDevice(SimDevice):
    """Class for devices compatible with Bogus-Bus."""

    category = "bogus_device"

    @classmethod
    def load_test_data(cls, data):
        """Side-load bus scan data."""
        cls._device_data = data


# Create several categories to group drives into
class ServoDevice(SimDevice):
    category = "servo_devices"


class IODevice(SimDevice):
    category = "io_devices"


class BogusV1ServoDevice(ServoDevice, BogusDevice):
    category = "bogus_v1_servo"


class BogusV2ServoDevice(ServoDevice, BogusDevice):
    category = "bogus_v2_servo"


class BogusV1IODevice(IODevice, BogusDevice):
    category = "bogus_v1_io"


# Concrete devices in a separate category
class BogusLowEndDevice(BogusDevice):
    category = "bogus_low_end"


class BogusV1Servo(BogusLowEndDevice, BogusV1ServoDevice):
    name = "bogo_v1_servo"
    model_id = 0xB0905000


class BogusV2Servo(BogusLowEndDevice, BogusV2ServoDevice):
    name = "bogo_v2_servo"
    model_id = 0xB0905001


class BogusV1IO(BogusLowEndDevice, BogusV1IODevice):
    name = "bogo_v1_io"
    model_id = 0xB0901000
