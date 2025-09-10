import abc
from ..cia_301.command import (
    CiA301Command,
    CiA301SimCommand,
    CiA301CommandException,
)


class EtherCATCommandException(CiA301CommandException):
    pass


class EtherCATCommand(CiA301Command):
    @abc.abstractmethod
    def alias(
        self,
        address=None,
        alias=None,
        **kwargs,
    ):
        """Set a device alias."""


class EtherCATSimCommand(EtherCATCommand, CiA301SimCommand):
    def alias(
        self,
        address=None,
        alias=None,
        **kwargs,
    ):
        # Set device alias in sim_device_data for future bus_scan() calls
        for dd in self.sim_device_data.values():
            if dd["address"][0] != address[0]:
                continue
            if address[2]:
                if dd["address"][2] != address[2]:
                    continue
            elif dd["address"][1] != address[1]:
                continue
            oldaddr, newaddr = dd["address"], dd["address"][:2] + (alias,)
            dd["address"] = newaddr
            addr_str = self.format_address(oldaddr)
            newaddr_str = self.format_address(newaddr)
            self.sim_sdo_data[newaddr] = self.sim_sdo_data.pop(oldaddr)
            self.sim_sdo_values[newaddr] = self.sim_sdo_values.pop(oldaddr)
            self.logger.info(f"Set sim device addr {addr_str} -> {newaddr_str}")
            return
        else:
            raise EtherCATCommandException(
                f"Failed to set device alias:  No device at address {address}"
            )
