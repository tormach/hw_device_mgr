import pytest
from ..config_io import ConfigIO
from .bogus_devices.data_types import BogusDataType
from .bogus_devices.device import (
    BogusDevice,
    BogusV1Servo,
    BogusV2Servo,
    BogusV1IO,
)


class BaseTestClass(ConfigIO):
    """Base test class providing fixtures for use with `bogus_devices`."""

    # Device scan data; for test fixture
    sim_device_data_package = "hw_device_mgr.tests"
    sim_device_data_yaml = "sim_devices.yaml"

    # Data types
    # Classes under test in this module
    data_type_class = BogusDataType
    device_class = BogusDevice
    device_model_classes = BogusV1IO, BogusV2Servo, BogusV1Servo

    # Sim mode by default
    sim = True

    # Length of device address tuple
    address_tuple_length = 2

    @classmethod
    def test_category_class(cls, test_category):
        for dmc in cls.device_model_classes:
            assert dmc.name
            if dmc.test_category == test_category:
                return dmc
        raise ValueError(
            f"{cls}:  No device in test category class '{test_category}'"
        )

    @classmethod
    def munge_test_address(cls, raw_address):
        """Massage test data address."""
        # Ignore addresses with aliases if not supported by class
        t_len = cls.address_tuple_length
        if len(raw_address) > t_len and raw_address[t_len]:
            return None
        # Pad and trim if needed
        pad = [0] * (cls.address_tuple_length - len(raw_address))
        padded_trimmed = (raw_address + pad)[0:cls.address_tuple_length]
        return tuple(padded_trimmed)

    @classmethod
    def munge_sim_device_data(cls, sim_device_data):
        """Massage device test data for reusability."""
        # We want to reuse the single `sim_devices.yaml` for all classes, but
        # key names and values that go into generating model_id will change.
        # This method adds those keys & values based on the YAML `test_category`
        # key (only used in tests).
        new_sim_device_data = list()
        for dev in sim_device_data:
            # Get device class from test_category key
            device_cls = cls.test_category_class(dev["test_category"])
            assert device_cls
            address = cls.munge_test_address(dev["address"])
            if address is None:
                continue
            new_sim_device_data.append(dev)
            # Set model_id key
            dev["model_id"] = device_cls.device_model_id()
            # Munged address
            dev["address"] = address
            # Set name (for test logging purposes only)
            dev["test_name"] = device_cls.name

        assert new_sim_device_data  # Sanity:  have test cases
        return new_sim_device_data

    @classmethod
    def init_sim(cls, **kwargs):
        kwargs["sim_device_data"] = cls.init_sim_device_data()
        cls.device_class.clear_devices()
        cls.device_class.init_sim(**kwargs)

    @classmethod
    def load_sim_device_data(cls):
        rsrc = cls.sim_device_data_package, cls.sim_device_data_yaml
        dev_data = cls.load_yaml_resource(*rsrc)
        assert dev_data, f"Empty device data in package resource {rsrc}"
        return dev_data

    @classmethod
    def init_sim_device_data(cls):
        # Set up sim devices:  munge data & pass to sim device class
        dev_data = cls.load_sim_device_data()
        return cls.munge_sim_device_data(dev_data)

    @pytest.fixture
    def device_cls(self):
        """Fixture for configured Device class."""
        self.init_sim()
        yield self.device_class

    @pytest.fixture
    def category_cls(self, device_cls):
        """Fixture for Device class category."""
        yield device_cls

    @pytest.fixture
    def all_device_data(self, device_cls):
        # All device data in a dict
        yield device_cls._sim_device_data[self.device_class.category]

    def pytest_generate_tests(self, metafunc):
        # Dynamic test parametrization
        # - sim_device_data:  iterate through `sim_device_data_yaml` list
        if "sim_device_data" not in metafunc.fixturenames:
            return

        data_raw = self.load_sim_device_data()
        sim_device_data = self.munge_sim_device_data(data_raw)
        vals, ids = (list(), list())
        for dev in sim_device_data:
            ids.append(f"{dev['test_name']}@{dev['address']}")
            vals.append(dev)
        metafunc.parametrize("sim_device_data", vals, ids=ids, scope="class")
