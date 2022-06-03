from ..ethercat.device import EtherCATDevice


class ITegvaE7xDevice(EtherCATDevice):
    """Base class for iTegva E7x series IO modules."""

    vendor_id = 0x00000A09
    xml_description_package = "hw_device_mgr.devices.device_xml"
    xml_description_fname = "iTegva_E7x_Series.xml"


# "Access_Bit":  IOs individually accessible via bit PDOs


class ITegvaE7820001BitDevice(ITegvaE7xDevice):
    """E7.820.001 16-ch Dig.In/16-ch Relay Out(Access_Bit)."""

    product_code = 0x00000100


class ITegvaE7820002BitDevice(ITegvaE7xDevice):
    """E7.820.002 32-ch Dig.Input(Access_Bit)."""

    product_code = 0x00000300


class ITegvaE7820003BitDevice(ITegvaE7xDevice):
    """E7.820.003 16-ch Dig.In/16-ch Mosfet Out(Access_Bit)."""

    product_code = 0x00000200  # [sic]


class ITegvaE7820004BitDevice(ITegvaE7xDevice):
    """E7.820.004 32-ch Dig.Output(Access_Bit)."""

    product_code = 0x00000400


# "Access_Byte":  IOs accessible via uint8 PDOs


class ITegvaE7820001ByteDevice(ITegvaE7xDevice):
    """E7.820.001 16-ch Dig.In/16-ch Relay Out(Access_Byte)."""

    product_code = 0x00000101


class ITegvaE7820002ByteDevice(ITegvaE7xDevice):
    """E7.820.002 32-ch Dig.Input(Access_Byte)."""

    product_code = 0x00000301


class ITegvaE7820003ByteDevice(ITegvaE7xDevice):
    """E7.820.003 16-ch Dig.In/16-ch Mosfet Out(Access_Byte)."""

    product_code = 0x00000201  # [sic]


class ITegvaE7820004ByteDevice(ITegvaE7xDevice):
    """E7.820.004 32-ch Dig.Output(Access_Byte)."""

    product_code = 0x00000401
