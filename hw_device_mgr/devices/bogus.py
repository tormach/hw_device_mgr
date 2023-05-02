from ..ethercat.device import EtherCATSimDevice
from ..cia_402.device import CiA402SimDevice


class BogusDevice(EtherCATSimDevice, CiA402SimDevice):
    """Base class for Bogus Device Co Devices."""

    category = "bogus_servo"
    vendor_id = 0xB090C0
    xml_description_package = "hw_device_mgr.devices.device_xml"
    xml_description_fname = "BogusServo.xml"
    device_error_package = "hw_device_mgr.devices.device_err"
    device_error_yaml = "unpopulated.yaml"


class BogusV1Servo(BogusDevice):
    """Bogus Device Co V1 servo drive."""

    product_code = 0xB0905030


class BogusV2Servo(BogusDevice):
    """Bogus Device Co V2 servo drive."""

    product_code = 0xB0905031


class BogusOtherDevice(EtherCATSimDevice):
    """Base class for Bogus Device Co devices other than motors."""

    category = "bogus_other"
    vendor_id = 0xB090C0
    xml_description_package = "hw_device_mgr.devices.device_xml"
    device_error_package = "hw_device_mgr.devices.device_err"
    device_error_yaml = "unpopulated.yaml"


class BogusJBox(BogusOtherDevice):
    """Bogus Device Co Junction Box."""

    product_code = 0xB0908030
    xml_description_fname = "BogusJunction.xml"


class BogusIO(BogusOtherDevice):
    """Bogus Device Co V1 IO Module."""

    product_code = 0xB0901030
    xml_description_fname = "BogusIO.xml"
    device_error_yaml = "bogus_v1_error_io.yaml"
