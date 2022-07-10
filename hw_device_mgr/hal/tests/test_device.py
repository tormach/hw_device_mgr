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
    def obj(self, sim_device_data, mock_halcomp):
        self.obj = self.device_model_cls(
            address=sim_device_data["test_address"]
        )
        self.obj.init(comp=mock_halcomp)
        yield self.obj

    def test_pin_interfaces(self, device_cls):
        for intf, data in device_cls.pin_interfaces.items():
            print(f"intf:  {intf};  data:  {data}")
            assert intf in device_cls.interface_names

    def test_init(self, obj):
        super().test_init(obj)

        assert obj.comp is self.mock_halcomp
        print(f"pins:\n{pformat(obj.pins)}")
        pnames = set()
        for intf, data in obj.pin_interfaces.items():
            print(f"interface:  {intf};  data:  {data}")
            for name in obj.interface(intf).get():
                pname = obj.pin_name(intf, name)
                assert pname in obj.pins
                pnames.add(pname)
        assert pnames == set(obj.pins.keys())

    #########################################
    # Test read()/update()/write() integration
    #

    def override_interface_param(self, interface, ovr_data):
        intf = self.obj.interface(interface)
        intf.update(**ovr_data)
        dt_names = self.obj.merge_dict_attrs(f"{interface}_data_types")
        for key, val in ovr_data.items():
            dt = dt_names.get(key, None)
            if dt is not None:
                val = self.obj.data_type_class.by_shared_name(dt)(val)
            pname = self.obj.pin_name(interface, key)
            self.set_pin(pname, val)

    def copy_sim_feedback(self, obj=None):
        if obj is None:
            obj = self.obj
            print(
                "\n*** Copying HAL pin values from sim_feedback to feedback_in"
            )
        for name in obj.sim_feedback.get():
            sfbpname = obj.pin_name("sim_feedback", name)
            fbpname = obj.pin_name("feedback_in", name)
            sfb_val = self.get_pin(sfbpname)
            self.set_pin(fbpname, sfb_val)
            fb_val = self.get_pin(fbpname)
            assert sfb_val == fb_val

    def pre_read_actions(self):
        self.copy_sim_feedback()

    def check_halpin_values(self, obj=None):
        if obj is None:
            obj = self.obj
            print("\n*** Checking halpin values were written as expected")
        for iface in ("sim_feedback", "command_out"):
            print(f"  {iface}:")
            for name, iface_val in obj.interface(iface).get().items():
                pname = obj.pin_name(iface, name)
                pin_val = self.get_pin(pname)
                print(f"    {pname}={pin_val}, {name}={iface_val}")
                assert pin_val == iface_val

    def post_write_actions(self):
        self.check_halpin_values()
