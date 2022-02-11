from ...config import CiA301Config
from .command import BogusCiA301Command


class BogusCiA301Config(CiA301Config):
    command_class = BogusCiA301Command
