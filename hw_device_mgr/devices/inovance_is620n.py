from ..ethercat.device import EtherCATDevice
from ..cia_402.device import CiA402Device


class InovanceIS620N(EtherCATDevice, CiA402Device):
    """Inovance IS620N servo drives"""

    vendor_id = 0x00100000
    product_code = 0x000C0108
    name = "IS620N_ECAT"
    xml_description_fname = "IS620N_v2.6.7.xml"

    def set_params_volatile(self, nv=False):
        # 0:  params not updated
        # 1:  2000h series changed from serial or EtherCAT saved
        # 2:  6000h series changed from EtherCAT (only) saved
        # 3:  2000h and 6000h series changed from EtherCAT (only) saved
        self.config.download("200C-0Eh", (0, 3)[nv])
