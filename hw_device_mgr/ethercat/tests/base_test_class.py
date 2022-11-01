from ...cia_402.tests.base_test_class import BaseCiA402TestClass
from ..data_types import EtherCATDataType
from ..sdo import EtherCATSDO
from ..command import EtherCATSimCommand
from ..config import EtherCATSimConfig
from .bogus_devices.device import (
    BogusEtherCATDevice,
    BogusEtherCATServo,
    BogusOtherCATServo,
    BogusEtherCATIO,
)
import re
import pytest


class BaseEtherCATTestClass(BaseCiA402TestClass):

    # Classes under test in this module
    data_type_class = EtherCATDataType
    sdo_class = EtherCATSDO
    command_class = EtherCATSimCommand
    config_class = EtherCATSimConfig
    device_class = BogusEtherCATDevice
    device_model_classes = (
        BogusEtherCATServo,
        BogusOtherCATServo,
        BogusEtherCATIO,
    )
    sdo_model_id_clone = tuple(
        [
            EtherCATSimConfig.format_model_id(mid)
            for mid in (
                (0x00B090C0, 0xB0905030),
                (0x00B090C0, 0xB0905031),
                (0x00B090C0, 0xB0901030),
            )
        ]
    )

    # Length of device address tuple:  Add `alias` field
    address_tuple_length = 3

    @classmethod
    def init_sim_sdo_kwargs(cls, **kwargs):
        """SDO and DC data are generated from ESI."""
        # Send back the result
        return kwargs

    @property
    def model_id_clone_map(self):
        return dict(zip(self.sdo_model_id_clone, self.device_model_classes))

    @pytest.fixture
    def device_xml(self, tmp_path):
        # Subclasses will have different product_code, so customize ESI file
        finished = set()
        re_str = "|".join(rf"{pc[1]:08X}" for pc in self.sdo_model_id_clone)
        re_str = r"#x(" + re_str + r")"
        pat = re.compile(re_str)
        # Map of orig ESI file product code to new ESI file product code
        cm = {
            k[1]: f"{v.device_model_id()[1]:08X}"
            for k, v in self.model_id_clone_map.items()
        }
        for id_orig, cls in self.model_id_clone_map.items():
            if not hasattr(cls, "alt_xml_description"):
                print(f"Using original ESI file for device {cls.name}")
                continue
            esi_orig = (cls.xml_description_package, cls.xml_description_fname)
            print(f"Model {cls.name} ESI resource:  {esi_orig}")
            esi_new = tmp_path / cls.xml_description_fname
            cls.alt_xml_description = esi_new
            if esi_new in finished:
                print(f"   Already written to {esi_new}")
                continue  # Only process each ESI file once
            finished.add(esi_new)
            print(f"   Writing to {esi_new}")
            with self.open_resource(*esi_orig) as f_orig:
                with self.open_path(esi_new, "w") as f_new:
                    for line in f_orig:
                        line = line.decode()
                        line = pat.sub(
                            lambda m: f"#x{cm[int(m.group(1), 16)]}", line
                        )
                        f_new.write(line)
        yield

    @pytest.fixture
    def extra_fixtures(self, device_xml):
        # Use this to add extra fixtures to the `device_cls` fixture
        # in subclasses
        pass
