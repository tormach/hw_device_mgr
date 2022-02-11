from ...device import EtherCATDevice
from ....cia_301.tests.bogus_devices.device import (
    BogusCiA301Device,
    BogusCiA301Servo,
    BogusCiA301Servo2,
)
from ....cia_402.device import CiA402Device
from .config import BogusEtherCATConfig


class BogusEtherCAT402Device(EtherCATDevice, CiA402Device, BogusCiA301Device):
    config_class = BogusEtherCATConfig
    category = "bogus_ethercat_402_devices"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.params_volatile = False

    def set_params_volatile(self, nv=False):
        self.params_volatile = not nv


class BogusEtherCAT402Servo(BogusEtherCAT402Device, BogusCiA301Servo):
    name = "bogo_ethercat_402_servo"
    product_code = 0xB0905032
    xml_description_fname = "BogusServo.xml"


class BogusOtherCAT402Servo(BogusEtherCAT402Device, BogusCiA301Servo2):
    name = "bogo_Othercat_402_servo"
    product_code = 0xB0905033
    xml_description_fname = "BogusServo.xml"
