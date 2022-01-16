# Real devices under LCEC
from ..hal.device import HALPinDevice, HALDataType
from ..lcec.device import LCECDevice, LCECSimDevice
from ..ethercat.device import EtherCATSimDevice
from ..cia_402.device import CiA402Device, CiA402SimDevice
from ..devices.elmo_gold import ElmoGold420, ElmoGold520
from ..devices.inovance_is620n import InovanceIS620N
from ..devices.inovance_sv660 import InovanceSV660
from ..devices.bogus import BogusServo


class ManagedDevices(LCECDevice, CiA402Device, HALPinDevice):
    category = "managed_lcec_devices"
    allow_rereg = True  # Subclasses reuse model_id


class ElmoGold420LCEC(ElmoGold420, ManagedDevices):
    pass


class ElmoGold520LCEC(ElmoGold520, ManagedDevices):
    pass


class InovanceIS620NLCEC(InovanceIS620N, ManagedDevices):
    pass


class InovanceSV660LCEC(InovanceSV660, ManagedDevices):
    pass


class SimManagedLCECDevices(LCECSimDevice, CiA402SimDevice, HALPinDevice):
    category = "sim_lcec_managed_devices"
    allow_rereg = True  # Subclasses reuse model_id


class BogusServoSimLCEC(BogusServo, SimManagedLCECDevices):
    pass


class ElmoGold420SimLCEC(ElmoGold420, SimManagedLCECDevices):
    pass


class ElmoGold520SimLCEC(ElmoGold520, SimManagedLCECDevices):
    pass


class InovanceIS620NSimLCEC(InovanceIS620N, SimManagedLCECDevices):
    pass


class InovanceSV660SimLCEC(InovanceSV660, SimManagedLCECDevices):
    pass


class SimManagedEtherCATDevices(
    EtherCATSimDevice, CiA402SimDevice, HALPinDevice
):
    category = "sim_ethercat_managed_devices"
    allow_rereg = True  # Subclasses reuse model_id
    data_type_class = HALDataType


class BogusServoSimEtherCAT(BogusServo, SimManagedEtherCATDevices):
    pass


class ElmoGold420SimEtherCAT(ElmoGold420, SimManagedEtherCATDevices):
    pass


class ElmoGold520SimEtherCAT(ElmoGold520, SimManagedEtherCATDevices):
    pass


class InovanceIS620NSimEtherCAT(InovanceIS620N, SimManagedEtherCATDevices):
    pass


class InovanceSV660SimEtherCAT(InovanceSV660, SimManagedEtherCATDevices):
    pass
