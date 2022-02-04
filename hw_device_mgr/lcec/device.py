from ..ethercat.device import EtherCATDevice, EtherCATSimDevice
from ..hal.device import HALPinDevice, HALPinSimDevice
from .data_types import LCECDataType
from .config import LCECConfig, LCECSimConfig


class LCECDevice(HALPinDevice, EtherCATDevice):
    data_type_class = LCECDataType
    config_class = LCECConfig


class LCECSimDevice(LCECDevice, HALPinSimDevice, EtherCATSimDevice):
    config_class = LCECSimConfig
