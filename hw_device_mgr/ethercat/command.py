from ..cia_301.command import (
    CiA301Command,
    CiA301SimCommand,
    CiA301CommandException,
)


class EtherCATCommandException(CiA301CommandException):
    pass


class EtherCATCommand(CiA301Command):
    pass


class EtherCATSimCommand(EtherCATCommand, CiA301SimCommand):
    pass
