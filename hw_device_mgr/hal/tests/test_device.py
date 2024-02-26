import pytest
from .base_test_class import BaseHALTestClass
from ...cia_402.tests.test_device import TestCiA402Device as _TestCiA402Device
from pprint import pformat


class TestHALDevice(BaseHALTestClass, _TestCiA402Device):
    halcomp_name = "hal_device"

    expected_mro = [
        "HALPinSimDevice",
        "HALPinDevice",
        *_TestCiA402Device.expected_mro,
        "HALMixin",
    ]

    @pytest.fixture
    def obj(self, sim_device_data, mock_halcomp, device_cls):
        self.obj = self.device_model_cls(address=sim_device_data["address"])
        self.obj.init(comp=mock_halcomp)
        yield self.obj

    def test_pin_interfaces(self, device_cls):
        for intf, data in device_cls.pin_interfaces.items():
            print(f"intf:  {intf};  data:  {data}")
            assert intf in device_cls.interface_names

    def test_init(self, obj):
        super().test_init(obj)

        assert obj.comp is self.halcomp_mockobj
        print(f"pins:\n{pformat(obj.pins)}")
        for intf_name, data in obj.pin_interfaces.items():
            names = set()
            print(f"interface:  {intf_name};  data:  {data}")
            intf = obj.interface(intf_name)
            for name in intf.get():
                if name not in obj.pins[intf_name]:
                    # Only reason is `str` objects don't map to HAL
                    assert intf.get_data_type(name).shared_name == "str"
                    continue
                names.add(name)
            assert names == set(obj.pins[intf_name].keys())
