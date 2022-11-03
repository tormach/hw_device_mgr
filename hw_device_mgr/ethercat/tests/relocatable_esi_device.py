from ..device import EtherCATSimDevice


class RelocatableESIDevice(EtherCATSimDevice):
    """A class whose ESI description file can be moved (for tests)."""

    alt_xml_description = None

    @classmethod
    def read_device_sdos_from_esi(cls, LcId="1033"):
        sdo_data = dict()
        for dev in cls.get_model():
            conf = dev.config_class
            dev_sdo_data = conf.get_device_sdos_from_esi(
                None, dev.alt_xml_description, LcId=LcId
            )
            sdo_data.update(dev_sdo_data)
        return sdo_data

    @classmethod
    def read_device_dcs_from_esi(cls, LcId="1033"):
        dcs_data = dict()
        for dev in cls.get_model():
            conf = dev.config_class
            dev_dcs_data = conf.get_device_dcs_from_esi(
                None, dev.alt_xml_description, LcId=LcId
            )
            dcs_data.update(dev_dcs_data)
        return dcs_data
