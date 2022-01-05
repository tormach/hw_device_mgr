from ...config import EtherCATConfig
from .command import BogusEtherCATCommand


class BogusEtherCATConfig(EtherCATConfig):
    command_class = BogusEtherCATCommand
