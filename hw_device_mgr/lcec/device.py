from ..ethercat.device import EtherCATDevice, EtherCATSimDevice
from ..hal.device import HALPinDevice, HALPinSimDevice
from .data_types import LCECDataType
from .config import LCECConfig, LCECSimConfig


class LCECDevice(EtherCATDevice, HALPinDevice):
    data_type_class = LCECDataType
    config_class = LCECConfig


class LCECSimDevice(LCECDevice, EtherCATSimDevice, HALPinSimDevice):
    config_class = LCECSimConfig
