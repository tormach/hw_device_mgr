from ..ethercat.device import EtherCATSimDevice
from ..cia_402.device import CiA402SimDevice


class BogusDevice(EtherCATSimDevice, CiA402SimDevice):
    """
    Base class for Bogus Device Co Devices.
    """

    category = "bogus_servo"
    vendor_id = 0xB090C0
    xml_description_fname = "BogusServo.xml"


class BogusServo(BogusDevice):
    """
    Bogus Device Co servo drive.
    """

    product_code = 0xB09050FF
    name = "bogus_servo_drive"
