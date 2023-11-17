from ..ethercat.device import EtherCATDevice
from ..cia_402.device import CiA402Device


class ElmoGold(EtherCATDevice, CiA402Device):
    """Base class for Elmo Gold EtherCAT Family Devices."""

    vendor_id = 0x0000009A
    xml_description_package = "hw_device_mgr.devices.device_xml"
    # FIXME The original ESI has models that differ only by revision,
    # but the ESI parser doesn't support this yet
    # xml_description_fname = "Elmo_ECAT_00010420_V11.xml"
    xml_description_fname = "Elmo_ECAT_00010420_V11.rev_10420_only.xml"
    device_error_package = "hw_device_mgr.devices.device_err"
    device_error_yaml = "unpopulated.yaml"

    def set_params_volatile(self, nv=False):
        # Params have to be explicitly saved
        if nv:
            for i, char in enumerate("save"):
                self.config.download(f"1010-0{i + 1}h", ord(char))


# FIXME These models differ only by revision, but the CiA 301 class
# doesn't support this yet

# class ElmoGold401(ElmoGold):
#     """Elmo 0x00030924 Gold EtherCAT GCON Boot Rev:0x00000001"""
#     product_code = 0x00030924
#     revision_no = 0x00000001


# class ElmoGold501(ElmoGold):
#     """Elmo 0x00030925 Gold EtherCAT GCON Boot Rev:0x00000001"""
#     product_code = 0x00030925
#     revision_no = 0x00000001


class ElmoGold420(ElmoGold):
    """Elmo 0x00030924 Gold EtherCAT GCON Drive Rev:0x00010420."""

    product_code = 0x00030924
    revision_no = 0x00010420


class ElmoGold520(ElmoGold):
    """Elmo 0x00030925 Gold EtherCAT GCON Drive ID Selector Rev:0x00010420."""

    product_code = 0x00030925
    revision_no = 0x00010420
