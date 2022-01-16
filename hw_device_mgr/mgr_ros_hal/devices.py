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


class ElmoGold420LCEC(ElmoGold420, ManagedDevices):
    product_code = 0x20030924


class ElmoGold520LCEC(ElmoGold520, ManagedDevices):
    product_code = 0x20030925


class InovanceIS620NLCEC(InovanceIS620N, ManagedDevices):
    product_code = 0x200C0108


class InovanceSV660LCEC(InovanceSV660, ManagedDevices):
    product_code = 0x200C010D


class SimManagedLCECDevices(LCECSimDevice, CiA402SimDevice, HALPinDevice):
    category = "sim_lcec_managed_devices"


class BogusServoSimLCEC(BogusServo, SimManagedLCECDevices):
    product_code = 0xB09050F2


class ElmoGold420SimLCEC(ElmoGold420, SimManagedLCECDevices):
    product_code = 0x30030924


class ElmoGold520SimLCEC(ElmoGold520, SimManagedLCECDevices):
    product_code = 0x30030925


class InovanceIS620NSimLCEC(InovanceIS620N, SimManagedLCECDevices):
    product_code = 0x300C0108


class InovanceSV660SimLCEC(InovanceSV660, SimManagedLCECDevices):
    product_code = 0x300C010D


class SimManagedEtherCATDevices(
    EtherCATSimDevice, CiA402SimDevice, HALPinDevice
):
    category = "sim_ethercat_managed_devices"
    data_type_class = HALDataType


class BogusServoSimEtherCAT(BogusServo, SimManagedEtherCATDevices):
    product_code = 0xB09050F3


class ElmoGold420SimEtherCAT(ElmoGold420, SimManagedEtherCATDevices):
    product_code = 0x40030924


class ElmoGold520SimEtherCAT(ElmoGold520, SimManagedEtherCATDevices):
    product_code = 0x40030925


class InovanceIS620NSimEtherCAT(InovanceIS620N, SimManagedEtherCATDevices):
    product_code = 0x400C0108


class InovanceSV660SimEtherCAT(InovanceSV660, SimManagedEtherCATDevices):
    product_code = 0x400C010D
