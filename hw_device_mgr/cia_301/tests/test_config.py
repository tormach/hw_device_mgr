from .base_test_class import BaseCiA301TestClass
import pytest
from pprint import pformat
import time


class TestCiA301Config(BaseCiA301TestClass):
    def test_scan_bus(self, config_cls):
        configs = config_cls.scan_bus(bus=0)
        for i, c in enumerate(configs):
            assert isinstance(c, config_cls)
            dd = self.command_class.sim_device_data[c.address]
            assert c.model_id == dd["model_id"]
            assert c.address == dd["address"]

    @pytest.fixture
    def obj(self, sim_device_data, config_cls):
        self.obj = config_cls(
            address=sim_device_data["address"],
            model_id=sim_device_data["test_model_id"],
        )
        yield self.obj

    def test_add_device_sdos(self, obj, config_cls, sdo_data):
        print("registered models w/SDOs:", list(config_cls._model_sdos))
        print("test obj model_id:", obj.model_id)
        assert obj.model_id in config_cls._model_sdos
        obj_sdos = obj._model_sdos[obj.model_id]
        assert obj_sdos
        print("test obj SDOs:\n", pformat(obj_sdos))
        assert list(sorted(obj_sdos)) == list(sorted(sdo_data))
        for ix, expected_sdo in sdo_data.items():
            assert ix in obj_sdos
            obj_sdo = obj_sdos[ix]
            assert isinstance(obj_sdo, config_cls.sdo_class)
            assert obj_sdo.index == expected_sdo.index
            assert obj_sdo.subindex == expected_sdo.subindex

    def test_sdo(self, obj, config_cls, sdo_data):
        for expected_sdo in sdo_data.values():
            ix = (expected_sdo.index, expected_sdo.subindex)
            print("index:", ix)
            print("sdo:", expected_sdo)
            obj_sdo = obj.sdo(ix)
            print("obj_sdo:", repr(obj_sdo))
            assert isinstance(obj_sdo, config_cls.sdo_class)
            assert ix == (obj_sdo.index, obj_sdo.subindex)
            assert issubclass(
                obj_sdo.data_type_class, config_cls.data_type_class
            )

    def test_upload_download(self, obj, sdo_data):
        # Simple test:  increment and check SDO value
        for sdo in sdo_data.values():
            sdo_ix = (sdo.index, sdo.subindex)
            print(sdo_ix)
            val = obj.upload(sdo_ix)
            obj.download(sdo_ix, val + 1)
            assert obj.upload(sdo_ix) == val + 1

    def test_initialize_params(self, obj, sdo_data, sim_device_data):
        # Test fixture data:  At least one config param value should
        # be different from default to make this test meaningful.  (IO
        # devices have no config values, so ignore those.)
        something_different = False
        for sdo_ix, conf_val in obj.config["param_values"].items():
            dev_val = obj.upload(sdo_ix)
            print(f"SDO {sdo_ix}:  device={dev_val}, config={conf_val}")
            if dev_val != conf_val:
                something_different = True
        assert something_different or not obj.config["param_values"]

        # Sanity checks
        pq = obj.params_queue
        assert pq.all_cmds_complete()

        # Start param init
        print("Starting param init")
        assert obj.initialize_params(restart=True) is False

        # Spin while we wait on the worker
        timeout, incr = 1, 0.01
        for i in range(int(timeout / incr)):
            if obj.initialize_params():
                break
            time.sleep(incr)
        else:
            print("initialize_params() never returned True!")
        print(f"Spun {i} cycles to init params")
        assert obj.initialize_params()
        assert pq.all_cmds_complete()

        for sdo_ix, val in obj.config["param_values"].items():
            assert obj.upload(sdo_ix) == val

    def test_load_optional_params(self, obj):
        # Get the un-pruned config to compare against after munging
        config = obj.find_config(obj.model_id, obj.address)
        raw_params = config.get("param_values", dict())
        optional_param_names = set(
            key
            for (key, val) in raw_params.items()
            if isinstance(val, dict) and val["optional"] is True
        )

        # Get params through the normal munging process
        config_normal = obj.gen_config(obj.model_id, obj.address)
        normal_params = set(config_normal.get("param_values", dict()))

        config_full = obj.gen_config(
            obj.model_id, obj.address, skip_optional=False
        )
        full_params = set(config_full.get("param_values", dict()))

        # Full parameter list should always include all of the non-optional
        # params
        assert full_params.intersection(normal_params) == normal_params
        assert len(full_params) >= len(normal_params)

        # Removing the set of non-optional params from the full list should
        # always give us the list of optional params
        assert len(full_params - normal_params) == len(optional_param_names)

    def test_add_device_dcs(self, obj, config_cls, dcs_data):
        print("model_id:", obj.model_id)
        print("config_cls._model_dcs:", config_cls._model_dcs)
        dcs_data = [dict(dc) for dc in dcs_data]
        print("expected dcs:", dcs_data)
        print("object dcs:", obj.dcs())
        assert len(obj.dcs()) == len(dcs_data)
        for expected_dc in dcs_data:
            assert dict(expected_dc) in obj.dcs()
