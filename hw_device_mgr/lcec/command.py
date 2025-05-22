import re
from ..ethercat.command import (
    EtherCATCommand,
    EtherCATSimCommand,
    EtherCATCommandException,
)
from .data_types import LCECDataType
import subprocess
import netifaces


class LCECCommand(EtherCATCommand):
    data_type_class = LCECDataType
    default_bus = 0

    @classmethod
    def _parse_output(cls, resp, kwargs):
        return resp

    @classmethod
    def address_to_args(cls, address):
        """Convert address to `ethercat` utility arguments."""
        # IgH `ethercat slaves --position=5 --alias=0` doesn't match slave at
        # pos. 5 if slave at pos. 4 has alias 4; `--alias=-` does the expected
        master, position, alias = cls.decode_address(address)
        return (
            f"--master={master}",
            f"--position={position}",
            f"--alias={alias or '-'}",
        )

    def _ethercat(
        self, *args, log_lev="debug", dry_run=False, stderr_to_devnull=False
    ):
        """
        Run IgH EtherCAT Master `ethercat` utility.

        Run `ethercat` utility with args; return minimally munged
        output.
        """
        cmd_args = ["ethercat"] + list(args)
        if dry_run:
            self.logger.info("Would run:  " + " ".join(cmd_args))
            return

        getattr(self.logger, log_lev)(" ".join(cmd_args))
        stderr = subprocess.DEVNULL if stderr_to_devnull else None
        try:
            resp = subprocess.check_output(cmd_args, stderr=stderr)
        except subprocess.CalledProcessError as e:
            raise EtherCATCommandException(str(e))

        # Parse and return output
        return resp.rstrip().decode().splitlines()

    _device_location_re = re.compile(r"=== Master ([0-9]), Slave ([0-9]+) ")

    def scan_bus(self, bus=None, **kwargs):
        bus = self.default_bus if bus is None else bus
        devices = list()
        output = self._ethercat(
            "slaves", f"--master={bus}", "--verbose", **kwargs
        )
        for line in output:
            line = line.strip()
            if line.startswith("==="):
                # === Master 0, Slave 0 ===
                m = self._device_location_re.match(line)
                master, position = m.groups()
                device = [(int(master), int(position), 0)]
                devices.append(device)
            elif line.startswith("Alias:"):
                #  Alias: 4
                alias = int(line.split(":", 1)[1].strip())
                addr = device[-1][0:2] + (alias,)
                device[0] = addr
            elif line.startswith("Vendor Id:"):
                #  Vendor Id:       0x00100000
                vi = line.split(":", 1)[1].strip()
            elif line.startswith("Product code:"):
                #  Product code:    0x000c0108
                pc = line.split(":", 1)[1].strip()
                model_id = (self.uint32(i) for i in (vi, pc))
                device.append(tuple(model_id))
        return devices

    _master_main_mac_re = re.compile(r"Main: ([0-9a-f:]+) ")

    def master_mac(self, bus=None):
        bus = self.default_bus if bus is None else bus
        for line in self._ethercat("master", f"--master={bus}"):
            m = self._master_main_mac_re.match(line.strip())
            if m:
                return m.group(1)
        else:
            return None

    def master_nic(self, bus=None):
        mac = self.master_mac(bus=bus)
        for intf in netifaces.interfaces():
            addrs = netifaces.ifaddresses(intf)
            link_addrs = addrs.get(netifaces.AF_LINK, list())
            for link_addr in link_addrs:
                if link_addr["addr"] == mac:
                    return intf
        else:
            return None

    def upload(
        self, address=None, index=None, subindex=0, datatype=None, **kwargs
    ):
        output = self._ethercat(
            "upload",
            *self.address_to_args(address),
            f"0x{index:04X}",
            f"0x{subindex:02X}",
            f"--type={datatype.igh_type}",
            **kwargs,
        )
        if datatype.shared_name == "str":
            return output[0]
        else:
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
        **kwargs,
    ):
        self._ethercat(
            "download",
            *self.address_to_args(address),
            f"--type={datatype.igh_type}",
            "--",
            f"0x{index:04X}",
            f"0x{subindex:02X}",
            str(value),
            log_lev="info",
            **kwargs,
        )

    def alias(
        self,
        address=None,
        alias=None,
        **kwargs,
    ):
        self._ethercat(
            "alias",
            *self.address_to_args(address),
            "--",
            str(alias),
            log_lev="info",
            **kwargs,
        )


class LCECSimCommand(LCECCommand, EtherCATSimCommand):
    """Simulated LCEC device."""
