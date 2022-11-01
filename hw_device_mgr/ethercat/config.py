from .sdo import EtherCATSDO
from ..cia_301.config import CiA301Config, CiA301SimConfig
from .data_types import EtherCATDataType
from .xml_reader import EtherCATXMLReader
from .command import EtherCATCommand, EtherCATSimCommand
from functools import lru_cache, cached_property


class EtherCATConfig(CiA301Config):
    """
    EtherCAT drive configuration interface.

    EtherCAT CoE shares much in common with CiA 301, and this class
    subclasses `CiA301Config`.

    Each EtherCAT device comes with an ESI "EtherCAT Slave
    Information" XML file that describes the device's object
    dictionary, sync managers, etc.
    """

    data_type_class = EtherCATDataType
    sdo_class = EtherCATSDO
    esi_reader_class = EtherCATXMLReader
    command_class = EtherCATCommand

    @cached_property
    def alias(self):
        return self.address[2]

    @classmethod
    def address_aliases(cls, address):
        res = [address[0:2] + (0,)]
        if len(address) > 2 and address[2]:
            res.append((address[0], 0, address[2]))
        return res

    @classmethod
    def address_in_canon_addresses(cls, address, canon_addresses):
        if address in canon_addresses:
            return address
        for canon_address in canon_addresses:
            if address in cls.address_aliases(canon_address):
                return canon_address
        return None

    @classmethod
    def canon_address_in_addresses(cls, canon_address, addresses):
        if canon_address in addresses:
            return canon_address
        for addr in cls.address_aliases(canon_address):
            if addr in addresses:
                return addr
        return None

    @classmethod
    def gen_config(cls, model_id, address):
        # Find matching config, considering device aliases
        for conf in cls._device_config:
            if "vendor_id" not in conf:
                continue  # In tests only
            if model_id != (conf["vendor_id"], conf["product_code"]):
                continue
            conf_addr = cls.canon_address_in_addresses(address, conf["addresses"])
            if conf_addr is None:
                continue
            break  # Found it
        else:
            raise KeyError(f"No config for device at {address}")
        # Prune & return config
        return cls.munge_config(conf, conf_addr)

    #
    # Device ESI
    #

    @classmethod
    @lru_cache
    def read_esi(cls, package, fname, LcId="1033"):
        """
        Read ESI XML and return `EtherCATXMLReader` object.

        ESI may be a package resource from `package` and `fname`;
        otherwise, if `package` is `None`, a file from path `fname`.
        """
        esi_reader_class = cls.esi_reader_class
        if package:
            esi_reader = esi_reader_class.read_from_resource(
                package, fname, LcId=LcId
            )
        else:
            esi_reader = esi_reader_class.read_from_path(fname, LcId=LcId)
        return esi_reader

    @classmethod
    @lru_cache
    def get_device_sdos_from_esi(cls, package, fname, LcId="1033"):
        """
        Read in device SDOs from ESI.

        The `package` and `fname` args are supplied to the `read_esi`
        method.
        """
        esi_reader = cls.read_esi(package, fname, LcId=LcId)
        return esi_reader.parse_sdos()

    @classmethod
    @lru_cache
    def get_device_dcs_from_esi(cls, package, fname, LcId="1033"):
        """
        Read in device distributed clocks from ESI.

        The `package` and `fname` args are supplied to the `read_esi`
        method.
        """
        esi_reader = cls.read_esi(package, fname, LcId=LcId)
        return esi_reader.parse_dc_opmodes()


class EtherCATSimConfig(EtherCATConfig, CiA301SimConfig):
    command_class = EtherCATSimCommand
