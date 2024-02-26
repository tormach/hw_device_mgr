from .base_test_class import BaseCiA402TestClass
from ...cia_301.tests.test_device import TestCiA301Device as _TestCiA301Device


class TestCiA402Device(_TestCiA301Device, BaseCiA402TestClass):
    # expected_mro is the same as CiA301Device, because this test class also
    # tests non-CiA402Device classes to ensure a realistic mix won't break, and
    # to simplify tests & test cases

    def test_cw_to_str(self, cia402_cls):
        if not cia402_cls:
            return
        tests = {
            0x0000: "SWITCH ON DISABLED flags: (none)",
            0x0002: "QUICK STOP ACTIVE flags: (none)",
            0x0006: "READY TO SWITCH ON flags: (none)",
            0x0007: "SWITCHED ON flags: (none)",
            0x000F: "OPERATION ENABLED flags: (none)",
            0x001F: "OPERATION ENABLED flags: OPERATION_MODE_SPECIFIC_1",
            0x0080: "CLEAR FAULT flags: (none)",
            0x002F: "OPERATION ENABLED flags: OPERATION_MODE_SPECIFIC_2",
            0x030F: "OPERATION ENABLED flags: HALT,NA_1",
        }
        for cw, cw_str in tests.items():
            expected = f"0x{cw:04X} {cw_str}"
            print(f'expected:  0x{cw:04X} : "{expected}",')
            actual = cia402_cls.cw_to_str(cw)
            print(f"actual:  {actual}")
            assert actual == expected

    def test_sw_to_str(self, cia402_cls):
        if not cia402_cls:
            return
        tests = {
            0x0010: "NOT READY TO SWITCH ON flags: VOLTAGE_ENABLED",
            0x0050: "SWITCH ON DISABLED flags: VOLTAGE_ENABLED",
            0x0450: "SWITCH ON DISABLED flags: VOLTAGE_ENABLED,TARGET_REACHED",
            0x0031: "READY TO SWITCH ON flags: VOLTAGE_ENABLED",
            0x0431: "READY TO SWITCH ON flags: VOLTAGE_ENABLED,TARGET_REACHED",
            0x0033: "SWITCHED ON flags: VOLTAGE_ENABLED",
            0x0037: "OPERATION ENABLED flags: VOLTAGE_ENABLED",
            0x9037: "OPERATION ENABLED flags: VOLTAGE_ENABLED,OPERATION_MODE_SPECIFIC_1,MANUFACTURER_SPECIFIC_3",
            0x0017: "QUICK STOP ACTIVE flags: VOLTAGE_ENABLED",
        }
        for sw, sw_str in tests.items():
            expected = f"0x{sw:04X} {sw_str}"
            print(f'expected:  0x{sw:04X} : "{expected}",')
            actual = cia402_cls.sw_to_str(sw)
            print(f"actual:  {actual}")
            assert actual == expected
