from ...mgr import SimHWDeviceMgr
from ....ethercat.device import EtherCATSimDevice
from ....cia_402.device import CiA402SimDevice
from ....devices.tests.devices import (
    ElmoGold420,
    ElmoGold520,
    InovanceIS620N,
    InovanceSV660,
    EVEXCRE,
)
from ....tests.interface import DebugInterface


class HwMgrTestDevices(EtherCATSimDevice, CiA402SimDevice):
    interface_class = DebugInterface
    category = "hw_mgr_test_devices"


class HwMgrTestElmoGold420(HwMgrTestDevices, ElmoGold420):
    name = "elmo_gold_0x30924_0x10420_hw_mgr_test"
    test_category = "elmo_gold_420_test"


class HwMgrTestElmoGold520(HwMgrTestDevices, ElmoGold520):
    name = "elmo_gold_0x30925_0x10420_hw_mgr_test"
    test_category = "elmo_gold_520_test"


class HwMgrTestInovanceIS620N(HwMgrTestDevices, InovanceIS620N):
    name = "IS620N_ECAT_hw_mgr_test"
    test_category = "inovance_is620n_test"


class HwMgrTestInovanceSV660N(HwMgrTestDevices, InovanceSV660):
    name = "SV660_ECAT_hw_mgr_test"
    test_category = "inovance_sv660n_test"


class HwMgrTestEVEXCRE(HwMgrTestDevices, EVEXCRE):
    name = "EVE-XCR-E_hw_mgr_test"
    test_category = "everest_xcr_e_test"


class HWDeviceMgrTestCategory(SimHWDeviceMgr):
    interface_class = DebugInterface
    category = "test_hw_device_mgr"
    device_base_class = HwMgrTestDevices
    device_classes = (
        HwMgrTestElmoGold420,
        HwMgrTestElmoGold520,
        HwMgrTestInovanceIS620N,
        HwMgrTestInovanceSV660N,
        HwMgrTestEVEXCRE,
    )


class HWDeviceMgrTest(HWDeviceMgrTestCategory):
    name = "test_sim_hw_device_mgr"
