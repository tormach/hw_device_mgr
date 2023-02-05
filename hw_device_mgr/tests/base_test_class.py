import pytest
from pprint import pformat
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

    #
    # Class methods for initializing sim data
    #

    @classmethod
    def init_sim(cls, **kwargs):
        kwargs["sim_device_data"] = cls.init_sim_device_data()
        cls.device_class.clear_devices()
        cls.device_class.init_sim(**kwargs)

    @classmethod
    def init_sim_device_data(cls):
        # Load and munge sim device data
        dev_data = cls.load_sim_device_data()
        return cls.munge_sim_device_data(dev_data)

    @classmethod
    def load_sim_device_data(cls):
        rsrc = cls.sim_device_data_package, cls.sim_device_data_yaml
        dev_data = cls.load_yaml_resource(*rsrc)
        assert dev_data, f"Empty device data in package resource {rsrc}"
        return dev_data

    @classmethod
    def munge_sim_device_data(cls, sim_device_data):
        """Massage device test data for reusability."""
        # We want to reuse the single `sim_devices.yaml` for all classes, but
        # key names and values that go into generating model_id will change.
        # This method adds those keys & values based on the YAML `test_category`
        # key (only used in tests).
        new_sim_device_data = list()
        for dev in sim_device_data:
            # Fill in missing device data based on `test_category` key
            if "test_category" in dev:
                device_cls = cls.test_category_class(dev["test_category"])
                assert device_cls
                assert device_cls.name
                # Set model_id key
                dev["model_id"] = device_cls.device_model_id()
                # Set name (for test logging purposes only)
                dev["test_name"] = device_cls.name
            # Assert no missing device data
            for key in ("address", "model_id", "test_name"):
                assert key in dev, f"No {key} key in {dev}"
            # Canonicalize address & weed out N/A addresses
            address = cls.munge_test_address(dev["address"])
            if address is None:
                continue
            dev["address"] = address
            # Add sim device data to result
            new_sim_device_data.append(dev)

        assert new_sim_device_data  # Sanity:  have test cases
        return new_sim_device_data

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
        padded_trimmed = (raw_address + pad)[0 : cls.address_tuple_length]
        return tuple(padded_trimmed)

    #
    # Test fixtures
    #

    @pytest.fixture
    def device_cls(self, _sim_device_data, category_cls):
        """Fixture for configured Device class."""
        yield _sim_device_data["device_cls"]

    @pytest.fixture
    def device_extra_fixtures(self):
        # Use this to add extra fixtures to the `device_cls` fixture
        # in subclasses
        pass

    @pytest.fixture
    def category_cls(self, category_extra_fixtures):
        """Fixture for Device class category with sim devices initialized."""
        self.init_sim()
        yield self.device_class

    @pytest.fixture
    def category_extra_fixtures(self):
        # Use this to add extra fixtures to the `category_cls` fixture
        # in subclasses
        pass

    @pytest.fixture
    def all_device_data(self, category_cls):
        # All device data in a dict
        data = category_cls._sim_device_data[self.device_class.category]
        assert data
        yield data

    @pytest.fixture
    def sim_device_data(self, _sim_device_data):
        yield _sim_device_data

    @pytest.fixture
    def mock_time(self, mocker):
        """Fixture to mock `time.time()`, returning test object `now` attr."""

        def side_effect():
            if not hasattr(self, "now"):
                raise RuntimeError("mock_time needs test object 'now' attr")
            print(f"mock time.time():  Returning {self.now}")
            return self.now

        mock_obj = mocker.MagicMock(name="time.time", side_effect=side_effect)
        mocker.patch("time.time", side_effect=side_effect)
        yield mock_obj

    def mock_sim_device_data(self):
        # When _sim_device_data not in fixtures, improvise list of devices, one
        # of each type
        dev_data = list()
        addr = 0
        for dev_cls in self.device_model_classes:
            dev_data.append(
                dict(
                    model_id=dev_cls.device_model_id(),
                    address=[0, addr],
                    test_name=dev_cls.name,
                    device_cls=dev_cls,
                )
            )
            addr += 1
        assert dev_data
        return dev_data

    @classmethod
    def sim_device_data_device_cls(cls, dev_data):
        device_cls = dev_data.get("device_cls", None)
        return device_cls or cls.device_class.get_model(dev_data["model_id"])

    def munge_pytest_generate_tests(self, dev_data, pytest_data, metafunc):
        return  # For subclasses to override

    def pytest_generate_tests(self, metafunc):
        # Dynamic test parametrization
        # - _device_cls:  iterate through device_model_classes
        # - _sim_device_data:  iterate through `sim_device_data_yaml` list; if
        #   present, device_cls will match the data
        #
        # Get raw sim device data from YAML or else generate it
        if "_sim_device_data" not in metafunc.fixturenames:
            dev_data_raw = self.mock_sim_device_data()
        else:
            dev_data_raw = self.load_sim_device_data()
        dev_data = self.dev_data = self.munge_sim_device_data(dev_data_raw)
        # Organize data to easily build pytest metafunc param structure
        pytest_data = dict()
        for dev in dev_data:
            # Fill in `device_cls` key with actual class
            if "test_category" in dev:
                device_cls = self.test_category_class(dev["test_category"])
            else:
                device_cls = self.sim_device_data_device_cls(dev)
            assert device_cls is not None
            dev["device_cls"] = device_cls
            # Set up test instance id and values
            inst_id = f"{device_cls.__name__}@{dev['address']}"
            inst_data = pytest_data[inst_id] = dict()
            # Populate test instance values, per fixture
            inst_data["_sim_device_data"] = dev
            inst_data["_device_cls"] = device_cls
            inst_data["pytest_data"] = pytest_data  # for debugging fixtures
        assert inst_data
        # Allow subclasses to extend this structure:
        # { inst_id: {_sim_device_data: {dev}, _device_cls: cls}, ...}
        self.munge_pytest_generate_tests(dev_data, pytest_data, metafunc)
        # Build pytest metafunc param structure
        names, ids, vals = None, list(), list()
        for inst_id, inst_data in pytest_data.items():
            if not names:  # Construct names from first instance data
                names = [
                    k for k in inst_data.keys() if k in metafunc.fixturenames
                ]
            ids.append(inst_id)
            vals.append([inst_data[name] for name in names])
        # Parametrize it
        if names:
            metafunc.parametrize(names, vals, ids=ids, scope="class")

    def test_fixture(self, device_cls, sim_device_data, category_cls):
        print("device_cls:", device_cls)
        print("sim_device_data:\n", pformat(sim_device_data))
        assert sim_device_data["device_cls"] is device_cls
        assert hasattr(device_cls, "name")
        assert device_cls in self.device_model_classes
        assert issubclass(device_cls, category_cls)
