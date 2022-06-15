import pytest
from ...tests.base_test_class import BaseTestClass
from ..data_types import CiA301DataType
from ..sdo import CiA301SDO
from ..config import CiA301SimConfig
from .bogus_devices.device import (
    BogusCiA301Device,
    BogusCiA301V1Servo,
    BogusCiA301V2Servo,
    BogusCiA301V1IO,
)
from ..command import CiA301SimCommand


class BaseCiA301TestClass(BaseTestClass):
    """Base test class for CiA301 module."""

    #
    # Configuration for test subclasses
    #

    # The device configuration, as in a real system
    device_config_package = "hw_device_mgr.cia_301.tests"
    device_config_yaml = "device_config.yaml"

    # Device model SDOs; for test fixture
    device_sdos_package = "hw_device_mgr.cia_301.tests"
    device_sdos_yaml = "sim_sdo_data.yaml"

    # Device model DCs; for test fixture
    device_dcs_package = "hw_device_mgr.cia_301.tests"
    device_dcs_yaml = "dcs_data.yaml"

    # Classes under test in this module
    data_type_class = CiA301DataType
    sdo_class = CiA301SDO
    command_class = CiA301SimCommand
    config_class = CiA301SimConfig
    device_class = BogusCiA301Device
    device_model_classes = (
        BogusCiA301V1Servo,
        BogusCiA301V2Servo,
        BogusCiA301V1IO,
    )

    # Whether to pass SDO/DC data to device_class.init_sim()
    pass_init_sim_device_description = True

    @classmethod
    def init_sim(cls, **kwargs):
        """Create sim device objects with configured SDOs."""
        if cls.pass_init_sim_device_description:
            # Init sim SDO data
            sdo_data = cls.load_sdo_data()
            print(f"  init_sim() sdo_data from {cls.sdo_data_resource()}")
            kwargs["sdo_data"] = cls.munge_sdo_data(sdo_data)
            # Init DC data
            dcs_data = cls.load_dcs_data()
            print(f"  init_sim() dcs_data from {cls.dcs_data_resource()}")
            kwargs["dcs_data"] = cls.munge_dcs_data(dcs_data)
        # Init sim device data
        super().init_sim(**kwargs)

    @classmethod
    def munge_device_config(cls, device_config):
        """
        Munge raw device config.

        Return a copy of `device_config` with minor processing.

        Optionally, to make the YAML file reusable, each configuration's
        `vendor_id` and `product_code` keys may be replaced with a `category`
        key matching a parent of classes listed; this fixture will re-add those
        keys.
        """
        new_device_config = list()
        for conf in device_config:
            device_cls = cls.test_category_class(conf["test_category"])
            assert device_cls
            new_device_config.append(conf)
            model_id = device_cls.device_model_id()
            conf["vendor_id"], conf["product_code"] = model_id
        assert new_device_config  # Sanity check not empty
        return new_device_config

    @pytest.fixture
    def extra_fixtures(self):
        # Use this to add extra fixtures to the `device_cls` fixture
        # in subclasses
        pass

    @pytest.fixture
    def device_cls(self, device_config, extra_fixtures):
        self.init_sim()
        self.device_class.set_device_config(device_config)
        yield self.device_class

    @pytest.fixture
    def config_cls(self, device_cls, device_config):
        yield self.config_class

    @pytest.fixture
    def command_cls(self, device_cls):
        yield self.command_class

    @classmethod
    def load_device_config(cls):
        """
        Load device configuration from package resource.

        The `importlib.resources` resource is named by
        `device_config_package` and `device_config_yaml` attrs.
        """
        rsrc = (cls.device_config_package, cls.device_config_yaml)
        dev_conf = cls.load_yaml_resource(*rsrc)
        assert dev_conf, f"Empty device config in package resource {rsrc}"
        print(f"  Raw device_config from {rsrc}")
        return dev_conf

    @pytest.fixture
    def device_config(self):
        """
        Device configuration data fixture.

        Load device configuration with `load_device_config()` and munge with
        `mung_device_config()`.

        Device configuration in the same format as non-test
        configuration, described in `Config` classes.
        """
        conf_raw = self.load_device_config()
        dev_conf = self.munge_device_config(conf_raw)
        self.device_config = dev_conf
        yield dev_conf

    @pytest.fixture
    def all_device_data(self, device_cls, request):
        """
        Device data from file named in `sim_device_data_yaml` attr.

        Device data is reformatted for ease of use in a `list` of `dict`:
        `model_key`:   `bogus_devices.device` model class attribute
        `bus`, `position`:  Device address
        `params`:  `dict` of SDO initial values
          `(idx, subidx)`:  Initial value, `DataType` instance

        The same data is available in the test object `dev_data` as a
        `dict` with device address key `(bus, position)`.
        """
        request.instance.sim_device_data = self.command_class.sim_device_data
        yield request.instance.sim_device_data

    @pytest.fixture
    def all_sdo_data(self, device_cls, request):
        """
        SDO data from file named in `device_sdos_yaml` attr.

        SDO data is read from file and reformatted for ease of use in
        a `dict`:

        `model_key`:  # `bogus_devices.device` model class attribute
          `(idx, subidx)`:  # One key (`tuple` of `int`) for each SDO of device
            `index`:  # SDO index
            `subindex`:  # ...and subindex
            `address`:  # `tuple(index, subindex)`
            `data_type`:  # `DataType` subclass
            `name`:  # Human-readable name
            `index_name`:  # Optional, human-readable category, default `""`
            `pdo_mapping`:  # If PDO mappable, `R` or `T`, else `None`
            `ro`:  # `True` or `False`
            `default_value`:  # Default value, `DataType` instance, default `0`
        """
        request.instance.sdo_data = self.command_class.sim_sdo_data
        request.instance.sdo_values = self.command_class.sim_sdo_values
        yield request.instance.sdo_data

    #
    # Dynamic test parameterization
    #

    @pytest.fixture
    def sim_device_data(self, _sim_device_data, device_cls):
        """
        Parametrize test with values from `all_device_data` fixture.

        When combined with the `sdo_data` fixture, `sim_device_data`
        values will match that fixture's device model.

        The `sim_device_data` is also available in the test instance's
        `sim_device_data` attribute.
        """
        self.sim_device_data = _sim_device_data
        model_id = (
            _sim_device_data["vendor_id"],
            _sim_device_data["product_code"],
        )
        _sim_device_data["test_model_id"] = model_id
        self.device_model_cls = device_cls.get_model(model_id)
        assert self.device_model_cls
        yield _sim_device_data

    @pytest.fixture
    def sdo_data(self, _sdo_data, device_cls):
        """
        Parametrize test with values from `all_sdo_data` fixture.

        When combined with the `sim_device_data` fixture, `sdo_data`
        values will match that fixture's device model.

        The `sdo_data` is also available in the test instance's
        `sdo_data` attribute.
        """
        self.sdo_data = _sdo_data
        yield _sdo_data

    @classmethod
    def munge_sdo_data(cls, sdo_data, conv_sdos=False):
        new_sdo_data = dict()
        for test_category, old_sdos in sdo_data.items():
            device_cls = cls.test_category_class(test_category)
            assert device_cls
            sdos = new_sdo_data[device_cls.device_model_id()] = dict()
            for ix, sdo in old_sdos.items():
                if conv_sdos:
                    ix = cls.config_class.sdo_ix(ix)
                    sdos[ix] = cls.sdo_class(**sdo)
                else:
                    sdos[ix] = sdo
        assert new_sdo_data
        assert None not in new_sdo_data.keys()
        return new_sdo_data

    @classmethod
    def munge_sim_device_data(cls, sim_device_data):
        sim_device_data = super().munge_sim_device_data(sim_device_data)
        for dev in sim_device_data:
            # Replace model_id key
            dev["vendor_id"], dev["product_code"] = dev.pop("model_id")
            # For test fixture
            dev["test_address"] = (dev["bus"], dev["position"])
        return sim_device_data

    @classmethod
    def sdo_data_resource(cls):
        return (cls.device_sdos_package, cls.device_sdos_yaml)

    @classmethod
    def load_sdo_data(cls):
        rsrc = cls.sdo_data_resource()
        sdo_data = cls.load_yaml_resource(*rsrc)
        assert sdo_data, f"Empty SDO data in package resource {rsrc}"
        return sdo_data

    @pytest.fixture
    def dcs_data(self, _dcs_data, device_cls):
        """
        Parametrize test with values from `device_dcs_yaml` resource.

        When combined with the `sim_device_data` fixture, `dcs_data`
        values will match that fixture's device model.

        The `dcs_data` is also available in the test instance's
        `dcs_data` attribute.
        """
        self.dcs_data = _dcs_data
        yield _dcs_data

    @classmethod
    def dcs_data_resource(cls):
        return (cls.device_dcs_package, cls.device_dcs_yaml)

    @classmethod
    def load_dcs_data(cls):
        rsrc = cls.dcs_data_resource()
        dcs_data = cls.load_yaml_resource(*rsrc)
        assert dcs_data, f"Empty DC data in package resource {rsrc}"
        return dcs_data

    @classmethod
    def munge_dcs_data(cls, dcs_data):
        new_dcs_data = dict()
        for test_category, dcs in dcs_data.items():
            device_cls = cls.test_category_class(test_category)
            assert device_cls
            new_dcs_data[device_cls.device_model_id()] = dcs
        assert new_dcs_data
        assert None not in new_dcs_data
        return new_dcs_data

    def pytest_generate_tests(self, metafunc):
        # Dynamic parametrization from sim_device_data_yaml:
        # - _sim_device_data:  iterate through `sim_device_data` list
        #   - with `_sdo_data`:  add matching entry in `sdo_data` list
        # - _sdo_data:  iterate through `sdo_data` values
        # - bus:  iterate through `sim_device_data` unique `bus` values
        # *Note all three cases are mutually exclusive
        dev_data = self.munge_sim_device_data(self.load_sim_device_data())
        sdo_data = self.munge_sdo_data(self.load_sdo_data(), conv_sdos=True)
        dcs_data = self.munge_dcs_data(self.load_dcs_data())
        names = list()
        vals, ids = (list(), list())
        if "_sim_device_data" in metafunc.fixturenames:
            names.append("_sim_device_data")
            assert "bus" not in metafunc.fixturenames  # sim_device_data["bus"]
            if "_sdo_data" in metafunc.fixturenames:
                names.append("_sdo_data")
            if "_dcs_data" in metafunc.fixturenames:
                names.append("_dcs_data")
            for dev in dev_data:
                ids.append(f"{dev['test_name']}@{dev['test_address']}")
                dev_vals = [dev]
                device_cls = self.test_category_class(dev["test_category"])
                assert device_cls is not None
                if "_sdo_data" in metafunc.fixturenames:
                    dev_vals.append(sdo_data[device_cls.device_model_id()])
                if "_dcs_data" in metafunc.fixturenames:
                    dev_vals.append(dcs_data[device_cls.device_model_id()])
                if len(dev_vals) == 1:
                    vals.append(dev_vals[0])
                else:
                    vals.append(dev_vals)
        elif "_sdo_data" in metafunc.fixturenames:
            names.append("_sdo_data")
            for model_id, device_sdos in sdo_data.items():
                vals.append(device_sdos)
                dev_cls = self.device_class.get_model(model_id)
                ids.append(dev_cls.test_category)
        elif "_dcs_data" in metafunc.fixturenames:
            names.append("_dcs_data")
            for model_id, device_dcs in dcs_data.items():
                vals.append(device_dcs)
                dev_cls = self.device_class.get_model(model_id)
                ids.append(dev_cls.test_category)
        elif "bus" in metafunc.fixturenames:
            names.append("bus")
            vals = list(d for d in {d["bus"] for d in dev_data})
            ids.extend(f"bus{b}" for b in vals)
        if names:
            metafunc.parametrize(",".join(names), vals, ids=ids, scope="class")
