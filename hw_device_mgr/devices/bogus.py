from ..ethercat.device import EtherCATSimDevice
from ..cia_402.device import CiA402SimDevice


class BogusDevice(EtherCATSimDevice, CiA402SimDevice):
    """Base class for Bogus Device Co Devices."""

    category = "bogus_servo"
    vendor_id = 0xB090C0
    xml_description_package = "hw_device_mgr.devices.device_xml"
    xml_description_fname = "BogusServo.xml"


class BogusV1Servo(BogusDevice):
    """Bogus Device Co V1 servo drive."""

    product_code = 0xB0905030


class BogusV2Servo(BogusDevice):
    """Bogus Device Co V2 servo drive."""

    product_code = 0xB0905031
