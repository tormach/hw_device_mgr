import re
from ..ethercat.command import EtherCATCommand, EtherCATCommandException
from .data_types import LCECDataType
import subprocess


class LCECCommand(EtherCATCommand):
    data_type_class = LCECDataType
    default_bus = 0

    @classmethod
    def _parse_output(cls, resp, kwargs):
        return resp

    def _ethercat(self, *args, log_lev="debug", dry_run=False):
        """Run IgH EtherCAT Master `ethercat` utility

        Run `ethercat` utility with args; return minimally munged
        output
        """
        cmd_args = ["ethercat"] + list(args)
        if dry_run:
            self.logger.info("Would run:  " + " ".join(cmd_args))
            return

        getattr(self.logger, log_lev)(" ".join(cmd_args))
        try:
            resp = subprocess.check_output(cmd_args)
        except subprocess.CalledProcessError as e:
            raise EtherCATCommandException(str(e))

        # Parse and return output
        return resp.rstrip().decode().splitlines()

    _device_location_re = re.compile(r"=== Master ([0-9]), Slave ([0-9]+) ")

    def scan_bus(self, bus=None):
        bus = self.default_bus if bus is None else bus
        devices = list()
        output = self._ethercat("slaves", f"--master={bus}", "--verbose")
        for line in output:
            line = line.strip()
            if line.startswith("==="):
                # === Master 0, Slave 0 ===
                m = self._device_location_re.match(line)
                master, position = m.groups()
                device = [(int(master), int(position))]
                devices.append(device)
            elif line.startswith("Vendor Id:"):
                #  Vendor Id:       0x00100000
                vi = line.split(":", 1)[1].strip()
            elif line.startswith("Product code:"):
                #  Product code:    0x000c0108
                pc = line.split(":", 1)[1].strip()
                model_id = (self.data_type_class.uint32(i) for i in (vi, pc))
                device.append(tuple(model_id))
        return devices

    def upload(self, address=None, index=None, subindex=0, datatype=None):
        index = self.data_type_class.uint16(index)
        subindex = self.data_type_class.uint16(subindex)
        output = self._ethercat(
            "upload",
            f"--master={address[0]}",
            f"--position={address[1]}",
            f"0x{index:04X}",
            f"0x{subindex:02X}",
            f"--type={datatype.igh_type}",
        )
        # FIXME Handle non-int types
        val_hex, val = output[0].split(" ", 1)
        val = int(val, 10)
        return val

    def download(
        self,
        address=None,
        index=None,
        subindex=0,
        value=None,
        datatype=None,
        dry_run=False,
    ):
        self._ethercat(
            "download",
            f"--master={address[0]}",
            f"--position={address[1]}",
            f"0x{index:04X}",
            f"0x{subindex:02X}",
            str(value),
            f"--type={datatype.igh_type}",
            log_lev="info",
            dry_run=dry_run,
        )
