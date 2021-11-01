# Real devices under LCEC
from ..lcec.device import LCECDevice
from ..cia_402.device import CiA402Device
from ..devices.elmo_gold import ElmoGold420, ElmoGold520
from ..devices.inovance_is620n import InovanceIS620N
from ..devices.inovance_sv660 import InovanceSV660


class ManagedDevices(LCECDevice, CiA402Device):
    category = "managed_devices"
    allow_rereg = True  # Subclasses reuse model_id


class ElmoGold420LCEC(ElmoGold420, ManagedDevices):
    pass


class ElmoGold520LCEC(ElmoGold520, ManagedDevices):
    pass


class InovanceIS620NLCEC(InovanceIS620N, ManagedDevices):
    pass


class InovanceSV660LCEC(InovanceSV660, ManagedDevices):
    pass
