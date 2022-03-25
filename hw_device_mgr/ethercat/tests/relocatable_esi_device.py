from ..device import EtherCATSimDevice


class RelocatableESIDevice(EtherCATSimDevice):
    """A class whose ESI description file can be moved (for tests)."""

    @classmethod
    def set_device_xml_dir(cls, path):
        # Tests generate customized ESI file in temp directory; provide a hook
        # to point fixtures to it
        cls.xml_base_dir = path

    @classmethod
    def xml_description_path(cls):
        if not hasattr(cls, "xml_base_dir"):
            return super().xml_description_path()
        return cls.xml_base_dir / cls.device_xml_dir / cls.xml_description_fname

    @classmethod
    def orig_xml_description_path(cls):
        return super().xml_description_path()
