from ...device import EtherCATDevice
from ....cia_301.tests.bogus_devices.device import (
    BogusCiA301Device,
    BogusCiA301Servo,
    BogusCiA301Servo2,
    BogusCiA301IO,
)
from .config import BogusEtherCATConfig


class BogusEtherCATDevice(EtherCATDevice, BogusCiA301Device):
    config_class = BogusEtherCATConfig
    category = "bogus_ethercat_devices"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.params_volatile = False

    def set_params_volatile(self, nv=False):
        self.params_volatile = not nv


class BogusEtherCATServo(BogusEtherCATDevice, BogusCiA301Servo):
    name = "bogo_ethercat_servo"
    product_code = 0xB0905030
    xml_description_fname = "BogusServo.xml"


class BogusOtherCATServo(BogusEtherCATDevice, BogusCiA301Servo2):
    name = "bogo_Othercat_servo"
    product_code = 0xB0905031
    xml_description_fname = "BogusServo.xml"


class BogusEtherCATIO(BogusEtherCATDevice, BogusCiA301IO):
    name = "bogo_ethercat_io"
    product_code = 0xB0901030
    xml_description_fname = "BogusIO.xml"
