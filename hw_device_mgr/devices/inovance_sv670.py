from .inovance_sv660 import InovanceSV660Config, InovanceSV660, SimInovanceSV660

class InovanceSV670Config(InovanceSV660Config):
    """Inovance SV670 servo drive config."""
    def maybe_later(self):
        self.logger.info('do we even need this?')

class InovanceSV670(InovanceSV660):
    """Inovance SV670 servo drives."""
    vendor_id = 0x00100000
    product_code = 0x000C011E
    xml_description_package = "hw_device_mgr.devices.device_xml"
    xml_description_fname = "SV670_EOE_1Axis_05003_220801.xml"
    device_error_package = "hw_device_mgr.devices.device_err"
    device_error_yaml = "inovance_sv670n.yaml"
    config_class = InovanceSV670Config
    have_sto = True

class SimInovanceSV670(SimInovanceSV660):
    def maybe_later(self):
        self.logger.info('do we even need this?')
