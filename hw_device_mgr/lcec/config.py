from ..ethercat.config import EtherCATConfig, EtherCATSimConfig
from .data_types import LCECDataType
from .xml_reader import LCECXMLReader
from .sdo import LCECSDO
from .command import LCECCommand, LCECSimCommand
from lxml import etree


class LCECConfig(EtherCATConfig):
    """Configuration for linuxcnc-ethercat and IgH EtherCAT Master."""

    data_type_class = LCECDataType
    esi_reader_class = LCECXMLReader
    sdo_class = LCECSDO
    command_class = LCECCommand

    @classmethod
    def gen_ethercat_xml(cls, bus_configs=dict()):
        """
        Generate the `ethercat.xml` config file for lcec.

        The `bus_configs` should be a dictionary of
        `master_idx:(appTimePeriod, refClockSyncCycles)`.
        """
        # Convert bus_configs keys to ints (YAML wants str type)
        for key in list(bus_configs):
            bus_configs[str(key)] = bus_configs.pop(key)
        # Scan bus once
        devs = cls.scan_bus()
        # Set up XML top level elements:  <masters><master/>[...]</masters>
        xml = etree.Element("masters")
        masters = dict()
        for dev in devs:
            if dev.bus in masters:
                continue
            bus_conf = bus_configs.get(dev.bus, dict())
            bus_conf["idx"] = str(dev.bus)
            atp = str(bus_conf.get("appTimePeriod", 1000000))
            bus_conf["appTimePeriod"] = atp
            rcsc = str(bus_conf.get("refClockSyncCycles", 1))
            bus_conf["refClockSyncCycles"] = rcsc
            master = etree.Element("master", **bus_conf)
            xml.append(master)
            masters[dev.bus] = (master, bus_conf)
        # Set up <slave/> elements & their child elements
        for dev in devs:
            master, bus_conf = masters[dev.bus]
            slave_xml = etree.Element(
                "slave",
                idx=str(dev.position),
                type="generic",
                vid=str(dev.vendor_id),
                pid=str(dev.product_code),
                configPdos="true",
            )
            master.append(slave_xml)
            config = cls.gen_config(dev.model_id, dev.address)
            # <dcConf/>
            if dev.dcs():
                assign_activate = max(
                    [dc["AssignActivate"] for dc in dev.dcs()]
                )
                s0s_default = int(int(bus_conf["appTimePeriod"]) / 2)
                s0s = config.get("dc_conf", dict()).get(
                    "sync0Shift", s0s_default
                )
                etree.SubElement(
                    slave_xml,
                    "dcConf",
                    assignActivate=hex(assign_activate),
                    sync0Cycle="*1",
                    sync0Shift=str(s0s),
                )
            # <syncManager/>
            for sm_ix, sm_data in config["sync_manager"].items():
                assert "dir" in sm_data
                assert sm_data["dir"] in {"in", "out"}
                sm_xml = etree.Element(
                    "syncManager", idx=sm_ix, dir=sm_data["dir"]
                )
                slave_xml.append(sm_xml)
                if not sm_data.get("pdo_mapping", None):
                    continue
                sdo = dev.sdo(sm_data["pdo_mapping"]["index"])
                pdo_xml = etree.Element("pdo", idx=str(sdo.index))
                sm_xml.append(pdo_xml)
                for entry in sm_data["pdo_mapping"]["entries"]:
                    sdo = dev.sdo(entry["index"])
                    dt = sdo.data_type
                    pdo_entry_xml = etree.Element(
                        "pdoEntry",
                        idx=str(sdo.index),
                        subIdx=str(sdo.subindex),
                        bitLen=str(dt.num_bits),
                    )
                    pdo_xml.append(pdo_entry_xml)
                    if "name" in entry:
                        pdo_entry_xml.set("halPin", entry["name"])
                        pdo_entry_xml.set("halType", dt.hal_type_str()[4:])
                    else:
                        # complexEntry
                        pdo_entry_xml.set("halType", "complex")
                        for bit in entry["bits"]:
                            complex_entry_xml = etree.Element(
                                "complexEntry", bitLen="1"
                            )
                            pdo_entry_xml.append(complex_entry_xml)
                            if bit is None:  # Unused bit
                                continue
                            elif isinstance(bit, dict):  # Dict of attributes
                                for k, v in bit.items():
                                    complex_entry_xml.set(k, str(v))
                            else:  # Pin name; assume 1 bit
                                complex_entry_xml.set("halType", "bit")
                                complex_entry_xml.set("halPin", bit)

        return etree.tostring(xml, pretty_print=True)


class LCECSimConfig(LCECConfig, EtherCATSimConfig):
    command_class = LCECSimCommand
