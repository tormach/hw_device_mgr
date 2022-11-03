from ..ethercat.device import EtherCATDevice
from ..ethercat.config import EtherCATConfig
from ..cia_402.device import CiA402Device
from ..errors.device import ErrorDevice


class InovanceSV660Config(EtherCATConfig):
    """Inovance SV660 servo drive config."""

    # Device params non-volatile setting in 200E-02h:
    # 0:  params not updated
    # 1:  2000h series changed from serial or EtherCAT saved
    # 2:  6000h series changed from EtherCAT (only) saved
    # 3:  2000h and 6000h series changed from EtherCAT (only) saved

    def get_device_params_nv(self):
        return self.config.upload("200E-02h") == 3

    def set_device_params_nv(self, nv=True, dry_run=False):
        curr_setting = self.get_device_params_nv()
        if curr_setting != nv:
            self.logger.info(
                f"{self} setting params {'non' if nv else ''}volatile mode"
            )
            self.config.download(
                "200E-02h", (0, 3)[nv], force=True, dry_run=dry_run
            )
        else:
            self.logger.info(
                f"{self} params already {'non' if nv else ''}volatile mode"
            )


class InovanceSV660(EtherCATDevice, CiA402Device, ErrorDevice):
    """Inovance SV660 servo drives."""

    vendor_id = 0x00100000
    product_code = 0x000C010D
    xml_description_package = "hw_device_mgr.devices.device_xml"
    xml_description_fname = "SV660_EOE_1Axis_V9.12.xml"
    device_error_package = "hw_device_mgr.devices.device_err"
    device_error_yaml = "inovance_sv660n.yaml"
    config_class = InovanceSV660Config
