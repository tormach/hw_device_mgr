import abc
from .data_types import CiA301DataType
from ..logging import Logging

__all__ = ("CiA301Command", "CiA301CommandException")


class CiA301CommandException(RuntimeError):
    pass


class CiA301Command(abc.ABC):
    """
    Abstract class for CiA 301 device command interface.

    Upload/download SDOs and scan bus for device identification.
    """

    data_type_class = CiA301DataType

    logger = Logging.getLogger(__name__)

    @abc.abstractmethod
    def scan_bus(self, bus=0):
        """Scan bus, returning list of addresses and IDs for each device."""

    @abc.abstractmethod
    def upload(
        self, address=None, index=None, subindex=0, datatype=None, **kwargs
    ):
        """Upload a value from a device SDO."""

    @abc.abstractmethod
    def download(
        self,
        address=None,
        index=None,
        subindex=0,
        value=None,
        datatype=None,
        **kwargs,
    ):
        """Download a value to a device SDO."""

    @classmethod
    def decode_address(cls, address):
        master, position, alias = address
        if alias:
            position = 0
        return master, position, alias


class CiA301SimCommand(CiA301Command):
    """Simulated CiA 301 device."""

    # Per-category sim device definitions from sim_devices.yaml:
    # [category] = {sdo data dict}
    sim_device_data = dict()
    # Per-category sim device object dictionary (SDOs, PDOs)
    sim_sdo_data = dict()
    # Per-device param value storage:  [address][ix, subix] = value
    sim_sdo_values = dict()

    @classmethod
    def init_sim(cls, sim_device_data=None, sdo_data=None):
        # Save & index device data
        assert sdo_data
        cls.init_sim_device_data(sim_device_data)
        cls.sim_sdo_data = sdo_data
        cls.init_sim_sdo_values()

    @classmethod
    def init_sim_device_data(cls, sim_device_data):
        cls.sim_device_data.clear()
        cls.sim_device_data.update(sim_device_data)

    @classmethod
    def init_sim_sdo_values(cls):
        for addr, dd in cls.sim_device_data.items():
            sdo_vals = cls.sim_sdo_values[addr] = dict()
            dd_params = dd.get("params", dict())
            for ix, sdo in cls.sim_sdo_data[addr].items():
                default = sdo.default_value
                sdo_vals[ix] = dd_params.get(str(sdo), default)

    @classmethod
    def sdo_str_to_ix(cls, sdo_str):
        dtc = cls.data_type_class
        idx, subidx = (sdo_str.split("-") + ["00h"])[:2]
        idx = dtc.uint16(int(idx[:4], 16))
        subidx = dtc.uint8(int(subidx[:2], 16))
        return (idx, subidx)

    def scan_bus(self, bus=0):
        res = list()
        for dd in self.sim_device_data.values():
            if dd["address"][0] != bus:
                continue
            res.append([dd["address"], dd["model_id"]])
        return res

    def upload(self, address=None, index=None, subindex=0, datatype=None):
        sdo = self.sim_sdo_data[address][index, subindex]
        val = self.sim_sdo_values[address][index, subindex]
        assert datatype is sdo.data_type
        return val or 0

    def download(
        self,
        address=None,
        index=None,
        subindex=0,
        value=None,
        datatype=None,
    ):
        sdo = self.sim_sdo_data[address][index, subindex]
        assert datatype is sdo.data_type
        value = datatype(value)
        self.sim_sdo_values[address][index, subindex] = value
