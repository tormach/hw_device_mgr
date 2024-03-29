from .data_types import CiA301DataType
from .command import CiA301Command, CiA301SimCommand, CiA301CommandException
from .sdo import CiA301SDO
from .async_params import AsyncParamsQueue
from ..logging import LoggingMixin
from functools import cached_property


class CiA301Config(LoggingMixin):
    """
    CiA 301 device configuration interface.

    This class presents a high-level configuration interface to CiA 301
    devices.  The class can scan the bus for devices and their models &
    positions.  Class instances correspond to a single device, and can
    initialize device parameter values from a configuration at init, and
    can upload and download SDO values during operation.

    It uses a low-level `CiA301Command` interface to scan the bus and
    upload/download object dictionary values, and presents a high-level
    interface for manipulating devices.

    This abstract class must be subclassed and `command_class` defined
    in order to read/write dictionary objects from/to devices.
    """

    def logging_name(self):
        return __name__

    data_type_class = CiA301DataType
    command_class = CiA301Command
    sdo_class = CiA301SDO

    init_params_nv = True

    # Mapping of model_id to a dict of (index, subindex) to SDO object
    _model_sdos = dict()
    # Mapping of model_id to a dict of (index, subindex) to DC object
    _model_dcs = dict()

    def __init__(
        self, address=None, model_id=None, skip_optional_config_values=True
    ):
        if hasattr(address, "address"):
            self.address = address.address  # Config object passed as address
        else:
            self.address = self.canon_address(address)
        self.model_id = self.format_model_id(model_id)
        self.params_queue = AsyncParamsQueue()
        self.skip_optional_config_values = skip_optional_config_values

    @classmethod
    def format_model_id(cls, model_id):
        assert None not in model_id, f"Invalid model ID {model_id} for {cls}"
        return tuple(cls.data_type_class.uint32(i) for i in model_id)

    @cached_property
    def vendor_id(self):
        return self.model_id[0]

    @cached_property
    def product_code(self):
        return self.model_id[1]

    @cached_property
    def bus(self):
        return self.address[0]

    @cached_property
    def position(self):
        return self.address[1]

    @classmethod
    def command(cls):
        if not hasattr(cls, "_command_objs"):
            cls._command_objs = dict()
        if cls.__name__ not in cls._command_objs:
            cls._command_objs[cls.__name__] = cls.command_class()
        return cls._command_objs[cls.__name__]

    def __str__(self):
        cname = self.__class__.__name__
        return f"{cname}:{self.model_id}@{self.address}".replace(" ", "")

    def __repr__(self):
        return f"<{self}>"

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
        assert self.model_id in self._model_sdos
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
        res = sdo.data_type(res_raw)
        self.logger.debug(f"upload SDO {sdo} = {res}")
        return res

    def download(
        self, sdo, val, dry_run=False, force=False, old=None, **kwargs
    ):
        # Get SDO object
        sdo = self.sdo(sdo)
        msg = f"(was {old})" if old is not None else ""
        if val == old:
            return  # SDO value already correct
        if not force:
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
            msg = f"(was {sdo.data_type(res_raw)})"
        if dry_run:
            self.logger.info(f"Dry run:  download {val} to {sdo} {msg}")
            return
        self.logger.info(f"Param download {sdo} = {val} {msg}")
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
        - Other device-specific keys
        - `bus`:  Bus `int` value
        - `positions`:  `list` of matching `int` positions on bus

        Each configuration provides these keys:

        - `sync_manager`:  `dict` of synchronization manager `int` IDs
          to configuration data:
          - `dir`:  `str`, `out` or `in`; all SM types
          - `pdo_mapping`:  PDO mapping SM types only; `dict`:
            - `index`:  Index of PDO mapping object
            - `entries`:  Dictionary objects to be mapped;  `dict`:
              - `index`:  Index of dictionary object, e.g. "6041h" or "1A00-03h"
              - `name`:  Name, a handle for the data object
              - `bits`:  Instead of `name`, break out individual bits,
                 names specified by a `list`

        - `param_values`:  `dict` of `<idx>-<subidx>h`-format object dictionary
          keys to values; values may be a single scalar applied to all
          `positions`, or a `list` of scalars applied to corresponding entries
          in `positions`.  May also be a dict with two keys, `value` and
          `optional`.  If `optional` is set, this value will only be applied if
          the `skip_optional parameter` is `False` on `gen_config()` calls.
        """
        assert config
        cls._device_config.clear()
        cls._device_config.extend(config)

    @classmethod
    def canon_address(cls, address):
        """
        Canonicalize device config address.

        Convert `address` values read from `device_config.yaml` to
        `tuple` of `(bus, position)` (from `list`).
        """
        return None if address is None else tuple(address)

    @classmethod
    def address_variants(cls, address):
        # Only one variant in CANopen addresses
        return [cls.canon_address(address)]

    @classmethod
    def munge_config(cls, config_raw, address, skip_optional=True):
        config_cooked = config_raw.copy()
        # Convert model ID ints
        model_id = (config_raw["vendor_id"], config_raw["product_code"])
        model_id = cls.format_model_id(model_id)
        config_cooked["vendor_id"], config_cooked["product_code"] = model_id
        # Convert addresses from lists to tuples
        addrs = [cls.canon_address(a) for a in config_cooked["addresses"]]
        config_cooked["addresses"] = addrs
        # Find index of address in config
        address = cls.canon_address(address)
        for pos_ix, pos_address in enumerate(addrs):
            if pos_address == address:
                break
            if pos_address in cls.address_variants(address):
                break
        else:
            raise KeyError(f"No address '{address}' in device config '{addrs}'")

        # Flatten out sync_manager pdo_mapping entries; needs deep copy
        sm_conf = config_cooked.get("sync_manager", dict()).copy()
        config_cooked["sync_manager"] = sm_conf
        for sm_idx, sm_idx_conf in list(sm_conf.items()):
            if sm_idx_conf.get("pdo_mapping", {}).get("entries", None) is None:
                continue  # Targeting PDO mapping entries with scale/offset keys
            sm_conf[sm_idx] = sm_idx_conf = sm_idx_conf.copy()
            pm = sm_idx_conf["pdo_mapping"] = sm_idx_conf["pdo_mapping"].copy()
            assert isinstance(pm["entries"], list)
            entries = pm["entries"] = list(pm["entries"])  # Copy list
            for entry_idx, entry in enumerate(entries):
                entries[entry_idx] = entry = entry.copy()
                for key, val in entry.items():
                    if isinstance(val, list) and key != "bits":
                        entry[key] = val[pos_ix]

        # Flatten out param_values key
        config_cooked["param_values"] = dict()
        for ix, val in config_raw.get("param_values", dict()).items():
            ix = cls.sdo_class.parse_idx_str(ix)
            # param_keys value can either be stored directly as a scalar or list
            # (to be applied universally) or as a dict with the key "optional"
            # which specifies that this is a low priority value that can be
            # skipped
            if isinstance(val, dict):
                if "optional" in val:
                    if val["optional"] is True and skip_optional:
                        # Skip this item and don't add it to config_cooked
                        continue
                    else:
                        val = val["value"]

            if isinstance(val, list):
                val = val[pos_ix]
            config_cooked["param_values"][ix] = val

        # Return pruned config dict
        return config_cooked

    @classmethod
    def find_config(cls, model_id, address):
        # Find matching config
        address = cls.canon_address(address)
        for conf in cls._device_config:
            if "vendor_id" not in conf:
                continue  # In tests only
            if model_id != (conf["vendor_id"], conf["product_code"]):
                continue
            if address not in conf["addresses"]:
                continue
            break
        else:
            raise KeyError(f"No config for device at {address}")
        return conf

    @classmethod
    def gen_config(cls, model_id, address, skip_optional=True):
        address = cls.canon_address(address)
        conf = cls.find_config(model_id, address)
        # Prune & return config
        return cls.munge_config(conf, address, skip_optional)

    @cached_property
    def config(self):
        return self.gen_config(
            self.model_id, self.address, self.skip_optional_config_values
        )

    def get_device_params_nv(self):
        """
        Return whether device is in non-volatile params mode.

        Drives with parameter volatile/non-volatile mode must overload
        this.
        """
        return False

    def set_device_params_nv(self, nv=True, dry_run=False):
        """
        Set device params to non-volatile/volatile mode.

        Drives with parameter volatile/non-volatile mode must overload
        this.
        """
        pass

    def initialize_params(self, restart=False, dry_run=False):
        """
        Asynchronously initialize device params.

        The first time this method is called, or if the `restart` arg is
        set, it will enqueue device params to be downloaded to the
        device in a worker thread.  This and following calls (without
        `restart` set) will return `False` until device params have
        finished downloading, and then will return `True`.

        When an offline device comes online, this function should be run
        (with `restart=True` the first time if device was previously
        offline) in a cycle until it returns `True` to ensure the device
        parameters are completely configured.
        """
        if restart:
            # Params haven't been queued up, or need requeuing
            num_params = len(self.config["param_values"])
            self.logger.info(f"Queueing {num_params} param updates")
            self.params_queue.download(
                self, self.config["param_values"], dry_run=dry_run
            )
            return False
        else:
            # Params enqueued; waiting on param processing complete
            complete = self.params_queue.all_cmds_complete()
            return complete

    #
    # Scan bus device config factory
    #

    @classmethod
    def scan_bus(cls, bus=0, skip_optional_config_values=True, **kwargs):
        """Return a `list` of configuration objects for each device."""
        res = list()
        for address, model_id in cls.command().scan_bus(bus=bus, **kwargs):
            model_id = cls.format_model_id(model_id)
            config = cls(
                address=address,
                model_id=model_id,
                skip_optional_config_values=skip_optional_config_values,
                **kwargs,
            )
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
            address = cls.canon_address(address)
            sdo_data[address] = cls._model_sdos.get(data["model_id"], dict())
        cls.command_class.init_sim(
            sim_device_data=sim_device_data, sdo_data=sdo_data
        )
