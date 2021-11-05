from .base_test_class import BaseCiA301TestClass
import pytest


class TestCiA301Config(BaseCiA301TestClass):
    def test_set_global_device_configuration(self, config_cls, global_config):
        config_cls.set_global_device_configuration(global_config)
        assert config_cls._global_config == global_config

    def test_scan_bus(self, bus, config_cls, all_device_data):
        configs = config_cls.scan_bus(bus=bus)
        for i, c in enumerate(configs):
            print(c)
            assert isinstance(c, config_cls)
            dd = all_device_data[i]
            assert c.model_id == dd["model_id"]
            assert c.address == dd["address"]

    @pytest.fixture
    def obj(self, device_data, sdo_data, config_cls):
        self.obj = config_cls(
            address=device_data["address"], model_id=device_data["model_id"]
        )
        yield self.obj

    def test_add_device_sdos(self, obj, config_cls):
        print("model_id:", obj.model_id)
        print("registered models:", list(config_cls._model_sdos))
        assert obj.model_id in config_cls._model_sdos
        obj_sdos = obj._model_sdos[obj.model_id]
        print("obj sdos keys:", sorted(obj_sdos.keys()))
        print("sdo data keys:", sorted(self.sdo_data.keys()))
        assert len(obj_sdos) == len(self.sdo_data)
        print("num SDOs:", len(obj_sdos))
        for ix, sd in self.sdo_data.items():
            print("    index:", ix)
            assert ix in obj_sdos
            sdo = obj_sdos[ix]
            assert isinstance(sdo, config_cls.sdo_class)
            assert sdo.index == sd["index"]
            assert sdo.subindex == sd["subindex"]

    def test_sdo(self, obj, sdo_data):
        for data in self.sdo_data.values():
            ix = (data["index"], data["subindex"])
            print("index:", ix)
            print("data:", data)
            sdo = obj.sdo(ix)
            print("sdo:", repr(sdo))
            assert isinstance(sdo, self.config_class.sdo_class)
            assert ix == (sdo.index, sdo.subindex)
            assert issubclass(sdo.data_type, self.config_class.data_type_class)

    def test_upload_download(self, obj, sdo_data):
        # Simple test:  increment and check SDO value
        for data in sdo_data.values():
            sdo_ix = (data["index"], data["subindex"])
            print(sdo_ix)
            val = obj.upload(sdo_ix)
            obj.download(sdo_ix, val + 1)
            assert obj.upload(sdo_ix) == val + 1

    def test_write_config_param_values(self, obj, sdo_data):
        # Test fixture data:  At least one config param value should
        # be different from default to make this test meaningful.  (IO
        # devices have no config values, so ignore those.)
        something_different = False
        for sdo_ix, conf_val in obj.config["param_values"].items():
            dev_val = obj.upload(sdo_ix)
            print(f"SDO {sdo_ix}:  device={dev_val}, config={conf_val}")
            if dev_val != conf_val:
                something_different = True
            # something_different |= (dev_val != conf_val)
        assert something_different or not obj.config["param_values"]

        obj.write_config_param_values()
        for sdo_ix, val in obj.config["param_values"].items():
            assert obj.upload(sdo_ix) == val
