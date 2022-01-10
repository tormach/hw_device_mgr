from ...cia_301.tests.base_test_class import BaseCiA301TestClass
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


class BaseEtherCATTestClass(BaseCiA301TestClass):

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
    device_model_sdo_clone = (
        (0x00B090C0, 0xB0905030),
        (0x00B090C0, 0xB0905031),
        (0x00B090C0, 0xB0901030),
    )

    def init_sim(self):
        device_class = getattr(self, "device_base_class", self.device_class)
        assert not getattr(self, "_sim_initialized", False)  # Run once only
        device_class.clear_devices()
        self.config_class._device_config.clear()
        self.command_class.sim_device_data.clear()
        path, dev_conf = self.load_yaml(self.device_config_yaml, True)
        print(f"  loaded device_config from {path}")
        device_class.set_device_config(dev_conf)
        path, dev_data = self.load_yaml(self.device_data_yaml, True)
        dev_data = self.munge_device_data(dev_data)
        print(f"  loaded device_data from {path}")
        device_class.add_device_sdos_from_esi()
        if self.device_model_sdo_clone:
            # Reuse ESI SDO data
            sdos = self.config_class._model_sdos
            for clone_id, cls in zip(
                self.device_model_sdo_clone, self.device_model_classes
            ):
                sdos[cls.device_type_key()] = sdos[clone_id]
        device_class.init_sim(device_data=dev_data)
        self._sim_initialized = True
