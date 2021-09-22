from ...config import LCECConfig
from .command import BogusLCECCommand


class BogusLCECConfig(LCECConfig):
    command_class = BogusLCECCommand
