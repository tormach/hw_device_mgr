from ...device import EtherCATSimDevice
from ....cia_402.tests.bogus_devices.device import (
    BogusCiA402DeviceCategory,
    BogusV1CiA402ServoCategory,
    BogusV2CiA402ServoCategory,
)


class BogusEtherCAT402Device(EtherCATSimDevice, BogusCiA402DeviceCategory):
    category = "bogus_ethercat_402_devices"
    xml_description_fname = "BogusServo.xml"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.params_volatile = False

    def set_params_volatile(self, nv=False):
        self.params_volatile = not nv


class BogusEtherCAT402Servo(BogusEtherCAT402Device, BogusV1CiA402ServoCategory):
    name = "bogo_ethercat_402_servo"
    product_code = 0xB0905032


class BogusOtherCAT402Servo(BogusEtherCAT402Device, BogusV2CiA402ServoCategory):
    name = "bogo_Othercat_402_servo"
    product_code = 0xB0905033
