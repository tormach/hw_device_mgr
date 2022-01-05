import pytest
from ...cia_301.tests.base_test_class import BaseCiA301TestClass
from ..data_types import EtherCATDataType
from ..sdo import EtherCATSDO
from .bogus_devices.command import BogusEtherCATCommand
from .bogus_devices.config import BogusEtherCATConfig
from .bogus_devices.device import (
    BogusEtherCATDevice,
    BogusEtherCATServo,
    BogusOtherCATServo,
    BogusEtherCATIO,
)


class BaseEtherCATTestClass(BaseCiA301TestClass):

    # Classes under test in this module
    data_type_class = EtherCATDataType
    sdo_class = EtherCATSDO
    command_class = BogusEtherCATCommand
    config_class = BogusEtherCATConfig
    device_class = BogusEtherCATDevice
    device_model_classes = (
        BogusEtherCATServo,
        BogusOtherCATServo,
        BogusEtherCATIO,
    )

    @pytest.fixture
    def config_cls(self, command_cls):
        """Configure the EtherCATConfig class

        Munge parsed ESI file with new device IDs for DRY purposes

        Pre-populate all model SDOs"""

        for dmc in self.device_model_classes:
            model_id = dmc.device_type_key()
            # Make the ESI files reusable:  munge the model ID to fit
            # the bogus_device subclass
            esi_reader = self.config_class.esi_reader_class()
            path = dmc.xml_description_path()
            sdos = esi_reader.add_device_descriptions(path)
            if not sdos:
                continue  # Already read
            if model_id not in sdos:
                # HACK for reusing Bogus Devices inc. test data
                assert len(sdos) == 1  # This won't work otherwise
                sdos[model_id] = sdos.pop(list(sdos)[0])
            # Normally this would happen on device instantiation; do
            # it here to make fixtures easier
            self.config_class.add_device_sdos(path)

            if model_id not in self.config_class._model_sdos:
                dev_ids = list(self.config_class._model_sdos)
                print(f"Config class SDO registry:  {dev_ids}")
                raise KeyError(
                    f"Device ID {model_id} not in config SDO registry"
                )

        yield self.config_class
