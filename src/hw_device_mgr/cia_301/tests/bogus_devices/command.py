from ...command import CiA301Command
from ...data_types import CiA301DataType


# Concrete class
class BogusCiA301Command(CiA301Command):
    data_type_class = CiA301DataType

    @classmethod
    def init(cls, all_device_data, all_sdo_data):
        # Save & index device data
        cls.all_device_data = all_device_data
        dd = cls.device_data = dict()
        for dev in cls.all_device_data:
            dd[dev["address"]] = dev

        # Save SDO data and init device param storage
        cls.sdo_data = all_sdo_data
        pd = cls.param_data = dict()
        for devix, dev in dd.items():
            devpd = pd[devix] = dict()
            for sdoix, sdo in all_sdo_data[dev["model_key"]].items():
                sdostr = f"{sdo['index']:04X}-{sdo['subindex']:02X}h"
                val = dev.get("params", {}).get(sdostr, sdo["default_value"])
                devpd[sdo["index"], sdo["subindex"]] = [val, sdo]

    def scan_bus(self, bus=0):
        res = list()
        for dev in self.all_device_data:
            if dev["bus"] != bus:
                continue
            res.append([dev["address"], dev["model_id"]])
        return res

    def upload(self, address=None, index=None, subindex=0, datatype=None):
        val, sdo = self.param_data[address][index, subindex]
        if datatype != sdo["data_type"]:
            raise RuntimeError(
                f"Datatype mismatch:  {datatype} != {sdo['data_type']}"
            )
        return val

    def download(
        self,
        address=None,
        index=None,
        subindex=0,
        value=None,
        datatype=None,
    ):
        val, sdo = self.param_data[address][index, subindex]
        if datatype != sdo["data_type"]:
            raise RuntimeError(
                f"Datatype mismatch:  {datatype} != {sdo['data_type']}"
            )
        sdo_str = f"{sdo['index']:04X}-{sdo['subindex']:02X}h"
        value = datatype(value)
        print(
            f"     set dev {address} SDO {sdo_str} ="
            f" [{value}, {datatype.name}]"
        )
        self.param_data[address][index, subindex][0] = value
