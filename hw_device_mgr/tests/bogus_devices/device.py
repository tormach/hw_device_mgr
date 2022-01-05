from ...device import Device


class BogusDevice(Device):
    """Class for devices compatible with Bogus-Bus"""

    category = "bogus_device"

    @classmethod
    def load_test_data(cls, data):
        """Side-load bus scan data"""
        cls._device_data = data

    @classmethod
    def scan_devices(cls, **kwargs):
        res = list()
        for data in cls._device_data:
            dev_type = cls.get_model(data["name"])
            dev = dev_type.get_device(address=data["address"], **kwargs)
            print(f"scan_devices:  found {dev}")
            res.append(dev)
        return res


class BogusServo(BogusDevice):
    name = "bogo_servo"
    model_id = 0xB0905000


class BogusIO(BogusDevice):
    name = "bogo_io"
    model_id = 0xB0901000
