from ...ethercat.tests.test_config import (
    TestEtherCATConfig as _TestEtherCATConfig,
)
from .base_test_class import BaseLCECTestClass
from lxml import etree


class TestLCECConfig(BaseLCECTestClass, _TestEtherCATConfig):
    halcomp_name = "lcec_config"
    ethercat_conf_xml_package = "hw_device_mgr.lcec.tests"
    ethercat_conf_xml_resource = "ethercat.conf.xml"

    def test_gen_ethercat_xml(self, config_cls, tmp_path):
        # Read expected conf.xml
        rsrc = (self.ethercat_conf_xml_package, self.ethercat_conf_xml_resource)
        with self.open_resource(*rsrc) as f:
            expected_xml = etree.parse(f)
        etree.strip_tags(expected_xml, etree.Comment)  # Clean out comments
        expected_str = etree.tostring(expected_xml).decode()
        expected_lines = expected_str.splitlines()
        print(f"Comparing lcec conf from {rsrc}")

        # Generate conf.xml
        conf = config_cls.gen_ethercat_xml(dict()).decode()
        assert conf
        conf_lines = conf.splitlines()
        print("Generated ethercat.conf.xml:")
        print(conf)

        # Compare lines
        for ix, expected_line in enumerate(expected_lines):
            print(f"{ix}  expected:  {expected_line}")
            assert ix < len(conf_lines)
            conf_line = conf_lines[ix]
            print(f"{ix}  conf:      {conf_line}")
            assert conf_line == expected_line
        # Compare number of lines
        assert len(conf_lines) == len(expected_lines)
