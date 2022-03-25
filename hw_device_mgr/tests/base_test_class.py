import pytest
from pathlib import Path
import os
import yaml
from .bogus_devices.data_types import BogusDataType
from .bogus_devices.device import (
    BogusDevice,
    BogusV1Servo,
    BogusV2Servo,
    BogusV1IO,
)


class BaseTestClass:
    """Base test class providing fixtures for use with `bogus_devices`."""

    # Device scan data; for test fixture
    sim_device_data_yaml = "tests/sim_devices.yaml"

    # Data types
    # Classes under test in this module
    data_type_class = BogusDataType
    device_class = BogusDevice
    device_model_classes = BogusV1IO, BogusV2Servo, BogusV1Servo

    # Sim mode by default
    sim = True

    @classmethod
    def load_yaml(cls, fname, return_path=False):
        p = Path(__file__).parent.parent.joinpath(fname)
        with p.open() as f:
            data = yaml.safe_load(f)
        return (p, data) if return_path else data

    @classmethod
    def test_category_class(cls, test_category):
        for dmc in cls.device_model_classes:
            assert dmc.name
            if dmc.test_category == test_category and dmc.name:
                return dmc
        raise ValueError(f"No test category class '{test_category}'")

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
            new_sim_device_data.append(dev)
            # Set model_id key
            dev["model_id"] = device_cls.device_model_id()
            # Set name & address (for test logging purposes only)
            dev["test_name"] = device_cls.name
            dev["test_address"] = dev["position"]

        assert new_sim_device_data  # Sanity:  have test cases
        return new_sim_device_data

    @classmethod
    def init_sim(cls, **kwargs):
        kwargs["sim_device_data"] = cls.init_sim_device_data()
        cls.device_class.clear_devices()
        cls.device_class.init_sim(**kwargs)

    @classmethod
    def init_sim_device_data(cls):
        # Set up sim devices:  munge YAML data & pass to sim device class
        cls.sim_device_data_path, dev_data = cls.load_yaml(
            cls.sim_device_data_yaml, True
        )
        print(f"  Raw sim_device_data from {cls.sim_device_data_path}")
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

        path, sim_device_data = self.load_yaml(self.sim_device_data_yaml, True)
        sim_device_data = self.munge_sim_device_data(sim_device_data)
        vals, ids = (list(), list())
        for dev in sim_device_data:
            ids.append(f"{dev['test_name']}@{dev['test_address']}")
            vals.append(dev)
        metafunc.parametrize("sim_device_data", vals, ids=ids, scope="class")

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
