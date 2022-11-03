from ..ethercat.device import EtherCATDevice


class CU1128(EtherCATDevice):
    """Beckhoff CU1128 EtherCAT Junction Box support."""

    vendor_id = 0x00000002
    product_code = 0x04685432
    xml_description_package = "hw_device_mgr.devices.device_xml"
    xml_description_fname = "Beckhoff CUxxxx.xml"
