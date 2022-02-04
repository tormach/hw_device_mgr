from ..ethercat.device import EtherCATDevice
from ..cia_402.device import CiA402Device


class InovanceSV660(EtherCATDevice, CiA402Device):
    """Inovance SV660 servo drives."""

    vendor_id = 0x00100000
    product_code = 0x000C010D
    xml_description_fname = "SV660_EOE_1Axis_V9.12.xml"

    def set_params_volatile(self, nv=False):
        # 0:  params not updated
        # 1:  2000h series changed from serial or EtherCAT saved
        # 2:  6000h series changed from EtherCAT (only) saved
        # 3:  2000h and 6000h series changed from EtherCAT (only) saved
        self.config.download("200E-02h", (0, 3)[nv])
