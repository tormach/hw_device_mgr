import pytest
from pathlib import Path
import os
import yaml
from .bogus_devices.data_types import BogusDataType
from .bogus_devices.device import BogusDevice, BogusServo, BogusIO


class BaseTestClass:
    """Base test class providing fixtures for use with `bogus_devices`"""

    # Device scan data; for test fixture
    device_data_yaml = "tests/bogus_devices/device.yaml"

    # Data types
    # Classes under test in this module
    data_type_class = BogusDataType
    device_class = BogusDevice
    device_model_classes = BogusServo, BogusIO

    # Device class has `category` attribute
    tc_is_category = True

    # Sim mode by default
    sim = True

    @classmethod
    def load_yaml(cls, fname):
        p = Path(__file__).parent.parent.joinpath(fname)
        with p.open() as f:
            return yaml.safe_load(f)

    @classmethod
    def munge_device_data(cls, data):
        """Massage device test data for readability"""
        uint32 = cls.data_type_class.uint32
        data["model_id"] = uint32(data["model_id"])
        return data

    @classmethod
    def load_device_data_yaml(cls):
        return cls.load_yaml(cls.device_data_yaml)

    @pytest.fixture
    def all_device_data(self):
        # All device data in a list
        dev_data = self.load_device_data_yaml()
        yield [self.munge_device_data(dev) for dev in dev_data]

    @pytest.fixture
    def device_cls(self, all_device_data):
        """Fixture for configured Device class"""
        # Sideload device data into test class
        self.device_class.load_test_data(all_device_data)
        self.device_class.clear_devices()
        yield self.device_class

    def pytest_generate_tests(self, metafunc):
        # Dynamic test parametrization
        # - device_data:  iterate through `device_data_yaml` list
        if "device_data" not in metafunc.fixturenames:
            return

        device_data = self.load_device_data_yaml()
        names = "device_data"
        vals, ids = (list(), list())
        for dev in device_data:
            self.munge_device_data(dev)
            ids.append(f"{dev['name']}@{dev['address']}")
            vals.append(dev)
        metafunc.parametrize(names, vals, ids=ids, scope="class")

    @pytest.fixture
    def fpath(self):
        """Fixture that returns test directory"""

        def func(base_name=None):
            cwd = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
            if base_name is None:
                return cwd
            else:
                return os.path.join(cwd, base_name)

        return func
