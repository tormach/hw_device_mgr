from ..ethercat.device import EtherCATDevice
from ..hal.device import HALPinDevice
from .data_types import LCECDataType
from .config import LCECConfig


class LCECDevice(EtherCATDevice, HALPinDevice):
    data_type_class = LCECDataType
    config_class = LCECConfig
