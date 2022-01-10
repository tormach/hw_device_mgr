import pytest
from pathlib import Path
import os
import yaml
from .bogus_devices.data_types import BogusDataType
from .bogus_devices.device import (
    BogusLowEndDevice,
    BogusV1Servo,
    BogusV2Servo,
    BogusV1IO,
)


class BaseTestClass:
    """Base test class providing fixtures for use with `bogus_devices`."""

    # Device scan data; for test fixture
    device_data_yaml = "tests/bogus_devices/sim_devices.yaml"

    # Data types
    # Classes under test in this module
    data_type_class = BogusDataType
    device_class = BogusLowEndDevice
    device_model_classes = BogusV1IO, BogusV2Servo, BogusV1Servo

    # Device class has `category` attribute
    tc_is_category = True

    # Sim mode by default
    sim = True

    @classmethod
    def load_yaml(cls, fname, return_path=False):
        p = Path(__file__).parent.parent.joinpath(fname)
        with p.open() as f:
            data = yaml.safe_load(f)
        return (p, data) if return_path else data

    @classmethod
    def munge_device_data(cls, device_data):
        """Massage device test data for usability."""
        # Locate device model class
        for dev in device_data:
            device_cls = cls.device_class.device_category_class(dev["category"])
            if device_cls is None:
                continue

            # Set sparse keys
            updates = dict(
                model_id=device_cls.device_type_key(),
                name=device_cls.name,
                address=dev["position"],
            )
            dev.update(updates)

        return device_data

    def init_sim(self):
        if getattr(self, "_sim_initialized", False):
            return
        self.device_class.clear_devices()
        self.dev_data_path, dev_data = self.load_yaml(
            self.device_data_yaml, True
        )
        dev_data = self.munge_device_data(dev_data)
        self.device_class.init_sim(device_data=dev_data)
        self._sim_initialized = True

    @classmethod
    def load_device_data_yaml(cls):
        return cls.load_yaml(cls.device_data_yaml)

    @pytest.fixture
    def device_cls(self):
        """Fixture for configured Device class."""
        # Sideload device data into test class
        self.init_sim()
        yield self.device_class

    @pytest.fixture
    def all_device_data(self, device_cls):
        # All device data in a dict
        yield device_cls._device_data

    def pytest_generate_tests(self, metafunc):
        # Dynamic test parametrization
        # - device_data:  iterate through `device_data_yaml` list
        if "device_data" not in metafunc.fixturenames:
            return

        path, device_data = self.load_yaml(self.device_data_yaml, True)
        device_data = self.munge_device_data(device_data)
        vals, ids = (list(), list())
        for dev in device_data:
            ids.append(f"{dev['name']}@{dev['address']}")
            vals.append(dev)
        metafunc.parametrize("device_data", vals, ids=ids, scope="class")

    @pytest.fixture
    def fpath(self):
        """Fixture that returns test directory."""
        # This line resolves black & pep257 conflicts.  :P

        def func(base_name=None):
            cwd = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
            if base_name is None:
                return cwd
            else:
                return os.path.join(cwd, base_name)

        return func
