import pytest
from ...tests.base_test_class import BaseTestClass
from ..data_types import CiA301DataType
from ..sdo import CiA301SDO
from .bogus_devices.config import BogusCiA301Config
from .bogus_devices.device import (
    BogusCiA301Device,
    BogusCiA301Servo,
    BogusCiA301Servo2,
    BogusCiA301IO,
)
from .bogus_devices.command import BogusCiA301Command


class BaseCiA301TestClass(BaseTestClass):
    """Base test class for CiA301 module"""

    #
    # Configuration for test subclasses
    #

    # The global configuration, as in a real system
    global_config_yaml = "cia_301/tests/global_config.yaml"

    # Device scan data; for test fixture
    device_data_yaml = "cia_301/tests/bogus_devices/device.yaml"

    # Device model SDOs; for test fixture
    device_sdos_yaml = "cia_301/tests/bogus_devices/config_sdos.yaml"

    # Classes under test in this module
    data_type_class = CiA301DataType
    sdo_class = CiA301SDO
    command_class = BogusCiA301Command
    config_class = BogusCiA301Config
    device_class = BogusCiA301Device
    device_model_classes = BogusCiA301Servo, BogusCiA301Servo2, BogusCiA301IO

    @classmethod
    def model_key_to_class(cls, model_key):
        """Return class from `device_model_classes` with matching `model_key`
        attribute (else default to `name` attribute); used by
        fixtures
        """
        mk_map = {
            getattr(dmc, "model_key", dmc.name): dmc
            for dmc in cls.device_model_classes
        }
        return mk_map.get(model_key, None)

    #
    # Global config:  Device configuration
    #

    @pytest.fixture
    def global_config(self):
        """Device configuration from file named in `global_config_yaml` attr

        Device configuration in the same format as non-test
        configuration, described in `Config` classes.

        Optionally, to make the YAML file reusable, each
        configuration's `vendor_id` and `product_code` keys may be
        replaced with a `model_key` key matching the attribute of the
        same name from one of the classes listed in the
        `device_model_classes` attribute; this fixture will re-add
        those keys.
        """
        global_conf = self.load_yaml(self.global_config_yaml)
        print(f"Loaded global config from {self.global_config_yaml}")
        for model_conf in global_conf:
            if "model_key" not in model_conf:
                continue
            model_cls = self.model_key_to_class(model_conf["model_key"])
            if model_cls is None:
                continue  # E.g. cia_402 doesn't cover the bogus_io device
            model_conf["vendor_id"] = model_cls.vendor_id
            model_conf["product_code"] = model_cls.product_code
        yield global_conf

    #
    # SDO fixtures:  SDO data from file named by `device_sdos_yaml` attr
    #

    @classmethod
    def sdo_str_to_ix(cls, sdo_str):
        dtc = cls.data_type_class
        idx, subidx = (sdo_str.split("-") + ["00h"])[:2]
        idx = dtc.uint16(int(idx[:4], 16))
        subidx = dtc.uint8(int(subidx[:2], 16))
        return (idx, subidx)

    @classmethod
    def munge_sdo(cls, sdo):
        # Munge an individual SDO's raw test data,
        # e.g. B090C0_B0905010/6040h
        dtc = cls.data_type_class
        attr_munge = dict(
            # attr = (type | 'dt', default)
            index=(dtc.uint16, None),
            subindex=(dtc.uint8, 0),
            ro=(None, False),
            default_value=("dt", 0),
            index_name=(None, ""),
        )
        dt = sdo["data_type"] = dtc.by_shared_name(sdo["data_type"])
        for k, munge in attr_munge.items():
            datatype, default = munge
            val = sdo.get(k, None)
            if val is None and default is None:
                continue
            if val is None:
                val = default
            if datatype == "dt":
                val = dt(val)
            elif datatype is not None:
                val = datatype(val)
            sdo[k] = val
        return sdo

    @classmethod
    def munge_sdos(cls, sdos):
        # Munge a dict of SDOs, e.g. sdo_data/bogo_servo
        for ix_str in list(sdos):
            data = sdos.pop(ix_str)
            if not data.get("sdo", True):
                continue  # Ignore PDO-only objects
            ix = cls.sdo_str_to_ix(ix_str)  # 6040h -> (0x6040, 0x00)
            sdo = cls.munge_sdo(data)  # Munge SDO data
            sdos[ix] = sdo
        return sdos

    @classmethod
    def munge_all_sdo_data(cls):
        raw_data = cls.load_yaml(cls.device_sdos_yaml)
        data = dict()
        for key, d in raw_data.items():
            data[key] = cls.munge_sdos(d)
        return data

    @pytest.fixture
    def all_sdo_data(self):
        """SDO data from file named in `device_sdos_yaml` attr

        SDO data is read from file and reformatted for ease of use in a `dict`:
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
        yield self.munge_all_sdo_data()

    @pytest.fixture
    def sdo_data(self, _sdo_data):
        """Parametrize test with values from `all_sdo_data` fixture

        When combined with the `device_data` fixture, `sdo_data`
        values will match that fixture's device model.

        The `sdo_data` is also available in the test instance's
        `sdo_data` attribute.
        """
        self.sdo_data = _sdo_data
        yield _sdo_data

    #
    # Device fixtures:  Results of `scan_devices()` and SDO initial values
    #

    @classmethod
    def munge_device_data(cls, data, sdo_data):
        """Massage device test data for readability"""
        # Locate device model class and SDO definitions
        model_key = data["model_key"]
        dev_cls = cls.model_key_to_class(model_key)
        sdos = sdo_data[model_key]
        # Redo `params` keys and values and set defaults
        p = data.setdefault("params", dict())
        # - init SDO values provided by device_data
        for sdo_str in list(p.keys()):
            sdo_ix = cls.sdo_str_to_ix(sdo_str)
            p[sdo_ix] = sdos[sdo_ix]["data_type"](p.pop(sdo_str))
        # - init rest of SDO values from sdo_data default values
        for ix, sdo in sdos.items():
            p.setdefault(ix, sdo["default_value"])
        # Set other keys and return
        model_id = data["model_id"] = dev_cls.device_type_key()
        data["vendor_id"], data["product_code"] = model_id
        data["address"] = (data["bus"], data["position"])
        data["name"] = dev_cls.name
        return data

    @pytest.fixture
    def all_device_data(self, all_sdo_data):
        """Device data from file named in `device_data_yaml` attr

        Device data is reformatted for ease of use in a `list` of `dict`:
        `model_key`:   `bogus_devices.device` model class attribute
        `bus`, `position`:  Device address
        `params`:  `dict` of SDO initial values
          `(idx, subidx)`:  Initial value, `DataType` instance

        The same data is available in the test object `dev_data` as a
        `dict` with device address key `(bus, position)`.
        """
        self.dev_data = dict()
        for dev_raw in self.load_device_data_yaml():
            dev = self.munge_device_data(dev_raw, all_sdo_data)
            self.dev_data[dev["address"]] = dev
        yield list(self.dev_data.values())

    @pytest.fixture
    def device_data(self, _device_data, device_cls):
        """Parametrize test with values from `all_device_data` fixture

        When combined with the `sdo_data` fixture, `device_data`
        values will match that fixture's device model.

        The `device_data` is also available in the test instance's
        `device_data` attribute.
        """
        self.device_data = _device_data
        model_id = _device_data["model_id"]
        self.device_model_cls = device_cls.get_model(model_id)
        yield _device_data

    #
    # Fixtures for initializing command, config and device classes
    #

    @pytest.fixture
    def command_cls(self, all_device_data, all_sdo_data):
        """Side-load bus scan data into command class"""
        self.command_class.init(all_device_data, all_sdo_data)
        yield self.command_class

    @pytest.fixture
    def config_cls(self, command_cls, global_config, all_sdo_data):
        self.config_class.set_global_device_configuration(global_config)
        for model_cls in self.device_model_classes:
            sdo_data = {
                model_cls.device_type_key(): all_sdo_data[model_cls.model_key]
            }
            self.config_class.add_device_sdos(sdo_data)
        yield self.config_class

    @pytest.fixture
    def device_cls(self, config_cls):
        self.device_class.clear_devices()
        yield self.device_class

    #
    # Dynamic test parameterization
    #

    def pytest_generate_tests(self, metafunc):
        # Dynamic parametrization from device_data_yaml:
        # - _device_data:  iterate through `device_data` list
        #   - with `_sdo_data`:  add matching entry in `sdo_data` list
        # - _sdo_data:  iterate through `sdo_data` values
        # - bus:  iterate through `device_data` unique `bus` values
        # *Note all three cases are mutually exclusive
        names = list()
        vals, ids = (list(), list())
        if "_device_data" in metafunc.fixturenames:
            names.append("_device_data")
            sdos = self.munge_all_sdo_data()
            if "_sdo_data" in metafunc.fixturenames:
                names.append("_sdo_data")
            for dev in self.load_device_data_yaml():
                self.munge_device_data(dev, sdos)
                ids.append(f"{dev['name']}@{dev['address']}")
                if "_sdo_data" in metafunc.fixturenames:
                    vals.append((dev, sdos[dev["model_key"]]))
                else:
                    vals.append(dev)
        elif "_sdo_data" in metafunc.fixturenames:
            all_sdos = self.munge_all_sdo_data()
            names.append("_sdo_data")
            for key, sdos in all_sdos.items():
                vals.append(sdos)
                ids.append(key)
        elif "bus" in metafunc.fixturenames:
            names.append("bus")
            buses = list({d["bus"] for d in self.load_device_data_yaml()})
            vals.extend(buses)
            ids.extend(f"bus{b}" for b in buses)
        if names:
            metafunc.parametrize(",".join(names), vals, ids=ids, scope="class")
