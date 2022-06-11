from .data_types import CiA301DataType
from .command import CiA301Command, CiA301SimCommand, CiA301CommandException
from .sdo import CiA301SDO
from functools import cached_property


class CiA301Config:
    """
    CiA 301 device configuration interface.

    This class presents a high-level configuration interface to
    CiA 301 devices.  The class can scan the bus for devices and
    their models & positions.  Class instances correspond to a single
    device, and can initialize device parameter values from a
    configuration at init, and can upload and download SDO values
    during operation.

    It uses a low-level `CiA301Command` interface to scan the bus and
    upload/download object dictionary values, and presents a
    high-level interface for manipulating devices.

    This abstract class must be subclassed and `command_class` defined
    in order to read/write dictionary objects from/to devices.
    """

    data_type_class = CiA301DataType
    command_class = CiA301Command
    sdo_class = CiA301SDO

    # Mapping of model_id to a dict of (index, subindex) to SDO object
    _model_sdos = dict()
    # Mapping of model_id to a dict of (index, subindex) to DC object
    _model_dcs = dict()

    def __init__(self, address=None, model_id=None):
        self.address = address
        self.model_id = self.format_model_id(model_id)
        self._config = None

    @classmethod
    def format_model_id(cls, model_id):
        assert None not in model_id
        return tuple(cls.data_type_class.uint32(i) for i in model_id)

    @property
    def vendor_id(self):
        return self.model_id[0]

    @property
    def product_code(self):
        return self.model_id[1]

    @property
    def bus(self):
        return self.address[0]

    @property
    def position(self):
        return self.address[1]

    @classmethod
    def command(cls):
        if not hasattr(cls, "_command_objs"):
            cls._command_objs = dict()
        if cls.__name__ not in cls._command_objs:
            cls._command_objs[cls.__name__] = cls.command_class()
        return cls._command_objs[cls.__name__]

    def __repr__(self):
        cname = self.__class__.__name__
        return f"<{cname} addr {self.address} model {self.model_id}>"

    #
    # Object dictionary
    #

    @classmethod
    def add_device_sdos(cls, sdo_data):
        """Add device model object dictionary descriptions."""
        for model_id, sdos in sdo_data.items():
            sdos_new = dict()
            for ix, sd in sdos.items():
                ix = cls.sdo_ix(ix)
                if ix in sdos_new:
                    raise KeyError(f"Duplicate SDO index {ix}")
                if isinstance(sd, cls.sdo_class):
                    sdos_new[ix] = sd
                else:
                    sdos_new[ix] = cls.sdo_class(**sd)
            cls._model_sdos[model_id] = sdos_new
        assert cls._model_sdos
        assert None not in cls._model_sdos

    @classmethod
    def sdo_ix(cls, ix):
        if isinstance(ix, str):
            ix = cls.sdo_class.parse_idx_str(ix)
        elif isinstance(ix, int):
            ix = (ix, 0)
        dtc = cls.data_type_class
        ix = (dtc.uint16(ix[0]), dtc.uint8(ix[1]))
        return ix

    @cached_property
    def sdos(self):
        assert self.model_id in self._model_sdos
        return self._model_sdos[self.model_id].values()

    def sdo(self, ix):
        if isinstance(ix, self.sdo_class):
            return ix
        ix = self.sdo_ix(ix)
        return self._model_sdos[self.model_id][ix]

    @classmethod
    def add_device_dcs(cls, dcs_data):
        """Add device model distributed clock descriptions."""
        for model_id, dcs in dcs_data.items():
            assert isinstance(dcs, list)
            cls._model_dcs[model_id] = dcs
        assert None not in cls._model_dcs

    def dcs(self):
        """Get list of distributed clocks for this device."""
        return self._model_dcs[self.model_id]

    def dump_param_values(self):
        res = dict()
        for sdo in self.sdos:
            try:
                res[sdo] = self.upload(sdo, stderr_to_devnull=True)
            except CiA301CommandException as e:
                # Objects may not exist, like variable length PDO mappings
                self.logger.debug(f"Upload {sdo} failed:  {e}")
                pass
        return res

    #
    # Param read/write
    #

    def upload(self, sdo, **kwargs):
        # Get SDO object
        sdo = self.sdo(sdo)
        res_raw = self.command().upload(
            address=self.address,
            index=sdo.index,
            subindex=sdo.subindex,
            datatype=sdo.data_type,
            **kwargs,
        )
        return sdo.data_type(res_raw)

    def download(self, sdo, val, dry_run=False, **kwargs):
        # Get SDO object
        sdo = self.sdo(sdo)
        # Check before setting value to avoid unnecessary NVRAM writes
        res_raw = self.command().upload(
            address=self.address,
            index=sdo.index,
            subindex=sdo.subindex,
            datatype=sdo.data_type,
            **kwargs,
        )
        if sdo.data_type(res_raw) == val:
            return  # SDO value already correct
        if dry_run:
            self.logger.info(f"Dry run:  download {val} to {sdo}")
            return
        self.command().download(
            address=self.address,
            index=sdo.index,
            subindex=sdo.subindex,
            value=val,
            datatype=sdo.data_type,
            **kwargs,
        )

    #
    # Device configuration
    #

    _device_config = list()

    @classmethod
    def set_device_config(cls, config):
        """
        Set the device configuration.

        `config` is a `list` of configurations in a `dict`.

        Each configuration matches a specific device model at any
        among a set of specific bus positions; keys for matching a
        device:

        - `vendor_id`, `product_code`:  `int` values for matching device
          model
        - `bus`:  Bus `int` value
        - `positions`:  `list` of matching `int` positions on bus

        Each configuration provides these keys:

        - `sync_manager`:  `dict` of synchronization manager `int` IDs
          to configuration data:
          - `dir`:  `str`, `out` or `in`; all SM types
          - `pdo_mapping`:  PDO mapping SM types only; `dict`:
            - `index`:  Index of PDO mapping object
            - `entries`:  Dictionary objects to be mapped;  `dict`:
              - `index`:  Index of dictionary object
              - `subindex`:  Subindex of dictionary object (default 0)
              - `name`:  Name, a handle for the data object
              - `bits`:  Instead of `name`, break out individual bits,
                 names specified by a `list`

        - `param_values`:  `dict` of `<idx>-<subidx>h`-format object
          dictionary keys to values; values may be a single scalar
          applied to all `positions`, or a `list` of scalars applied
          to corresponding entries in `positions`
        """
        assert config
        cls._device_config.clear()
        cls._device_config.extend(config)

    @classmethod
    def munge_config(cls, config_raw, position):
        config_cooked = config_raw.copy()
        # Convert model ID ints
        model_id = (config_raw["vendor_id"], config_raw["product_code"])
        model_id = cls.format_model_id(model_id)
        config_cooked["vendor_id"], config_cooked["product_code"] = model_id
        # Flatten out param_values key
        config_cooked["param_values"] = dict()
        for ix, val in config_raw.get("param_values", dict()).items():
            ix = cls.sdo_class.parse_idx_str(ix)
            if isinstance(val, list):
                pos_ix = config_raw["positions"].index(position)
                val = val[pos_ix]
            config_cooked["param_values"][ix] = val
        # Return pruned config dict
        return config_cooked

    @property
    def config(self):
        if self._config is None:
            # Find matching config
            for conf in self._device_config:
                if "vendor_id" not in conf:
                    continue  # In tests only
                if self.model_id != (conf["vendor_id"], conf["product_code"]):
                    continue
                if self.bus != conf["bus"]:
                    continue
                if self.position not in conf["positions"]:
                    continue
                break
            else:
                raise KeyError(f"No config for device at {self.address}")
            # Prune & cache config
            self._config = self.munge_config(conf, self.position)

        # Return cached config
        return self._config

    def write_config_param_values(self):
        for sdo, value in self.config["param_values"].items():
            self.download(sdo, value)

    #
    # Scan bus device config factory
    #

    @classmethod
    def scan_bus(cls, bus=0, **kwargs):
        """Return a `list` of configuration objects for each device."""
        res = list()
        for address, model_id in cls.command().scan_bus(bus=bus, **kwargs):
            model_id = cls.format_model_id(model_id)
            config = cls(address=address, model_id=model_id, **kwargs)
            res.append(config)
        return res


class CiA301SimConfig(CiA301Config):
    """Configuration for simulated devices with simulated command."""

    command_class = CiA301SimCommand

    @classmethod
    def init_sim(cls, *, sim_device_data):
        assert sim_device_data
        sdo_data = dict()
        for address, data in sim_device_data.items():
            sdo_data[address] = cls._model_sdos.get(data["model_id"], dict())
        cls.command_class.init_sim(
            sim_device_data=sim_device_data, sdo_data=sdo_data
        )
