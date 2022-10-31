import pytest
from .base_test_class import BaseCiA301TestClass
from pprint import pprint
import random


class TestCiA301Command(BaseCiA301TestClass):
    @pytest.fixture
    def obj(self, command_cls):
        return command_cls()

    def test_scan_bus(self, obj, bus, all_device_data):
        bus_data = obj.scan_bus(bus=bus)
        assert bus_data
        for actual in bus_data:
            expected = all_device_data[actual[0]]
            actual_address, actual_model_id = actual
            print(f"address:  {expected['address']} =? {actual_address}")
            assert expected["address"] == actual_address
            print(f"model_id:  {expected['model_id']} =? {actual_model_id}")
            assert expected["model_id"] == actual_model_id

    def random(self, data_type):
        if data_type in ("float", "double"):
            return (random.random() - 0.5) * 1e10
        if data_type == "str":
            # Random-length (0-9) string of random bytes
            return random.randbytes(random.random(0, 10))

        if data_type.startswith("int"):
            bits = int(data_type[3:], 10)
            stop = 1 << bits
            start = -(stop - 1)
        elif data_type.startswith("uint"):
            bits = int(data_type[4:], 10)
            start = 0
            stop = 1 << bits

        dt = self.data_type_class.by_shared_name(data_type)
        return dt(random.randrange(start, stop))

    def test_upload_download(self, obj, bus, all_device_data, all_sdo_data):
        vals = dict()
        for address, model_id in obj.scan_bus(bus=bus):
            for dev in all_device_data.values():
                if address == dev["address"]:
                    break
            print("device data:")
            pprint(dev)

            sdos = all_sdo_data[address]
            print("device sdos:")
            pprint(sdos)

            # Save initial values
            for ix, sdo in sdos.items():
                dt = sdo.data_type
                val = obj.upload(
                    address=address, index=ix[0], subindex=ix[1], datatype=dt
                )
                vals[address, ix] = val

            # Run through setting and getting random values
            for i in range(10):
                for ix, sdo in sdos.items():
                    dt = sdo.data_type
                    # Upload & check old value
                    val = obj.upload(
                        address=address,
                        index=ix[0],
                        subindex=ix[1],
                        datatype=dt,
                    )
                    assert val == vals[address, ix]
                    # Generate & download new value
                    val = self.random(dt.shared_name)
                    vals[address, ix] = val
                    obj.download(
                        address=address,
                        index=ix[0],
                        subindex=ix[1],
                        datatype=dt,
                        value=val,
                    )
                    # Upload & check new value
                    val = obj.upload(
                        address=address,
                        index=ix[0],
                        subindex=ix[1],
                        datatype=dt,
                    )
                    assert val == vals[address, ix]
