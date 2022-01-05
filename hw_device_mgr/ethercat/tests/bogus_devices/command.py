from ...command import EtherCATCommand
from ....cia_301.tests.bogus_devices.command import BogusCiA301Command


class BogusEtherCATCommand(EtherCATCommand, BogusCiA301Command):
    pass
