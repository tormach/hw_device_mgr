from ...command import LCECCommand
from ....ethercat.tests.bogus_devices.command import BogusEtherCATCommand


class BogusLCECCommand(LCECCommand, BogusEtherCATCommand):
    pass
