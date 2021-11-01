import abc
from .data_types import CiA301DataType
from ..logging import Logging

__all__ = ("CiA301Command", "CiA301CommandException")


class CiA301CommandException(RuntimeError):
    pass


class CiA301Command(abc.ABC):
    """Abstract class for CiA 301 device command interface:
    upload/download SDOs and scan bus for device identification
    """

    data_type_class = CiA301DataType

    logger = Logging.getLogger(__name__)

    @abc.abstractmethod
    def scan_bus(self, bus=0):
        """Scan bus, returning list of addresses and IDs for each device"""

    @abc.abstractmethod
    def upload(self, address=None, index=None, subindex=0, datatype=None):
        """Upload a value from a device SDO"""

    @abc.abstractmethod
    def download(
        self,
        address=None,
        index=None,
        subindex=0,
        value=None,
        datatype=None,
    ):
        """Download a value to a device SDO"""
