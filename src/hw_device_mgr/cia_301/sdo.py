import re
from .data_types import CiA301DataType


class CiA301SDO:
    """A data class representing a dictionary object for a particular
    device model
    """

    data_type_class = CiA301DataType

    idx_re = re.compile(r"^([0-9a-f]{4})(?:-([0-9a-f]{2}))?h$", flags=re.I)

    def __init__(
        self,
        index=None,
        subindex=0,
        data_type=None,
        name=None,
        index_name=None,
        ro=False,
        pdo_mapping=None,
        default_value=0,
        min_value=None,
        max_value=None,
    ):
        self.index = self.data_type_class.uint16(index)
        self.subindex = self.data_type_class.uint8(subindex or 0)
        self.data_type = data_type
        self.name = name
        self.index_name = index_name
        self.ro = ro
        self.pdo_mapping = pdo_mapping
        self.default_value = default_value
        self.min_value = min_value
        self.max_value = max_value

    def __str__(self):
        return f"{self.index:04X}-{self.subindex:02X}h"

    def __repr__(self):
        cname = self.__class__.__name__
        return f"<{cname} {self.__str__()}>"

    def to_data_type_value(self, value):
        return self.data_type(value)

    @classmethod
    def parse_idx_str(cls, idx_str):
        m = cls.idx_re.match(idx_str).groups()
        m = (m[0], m[1] or "0")
        idx, subidx = (int(i, 16) for i in m)
        dtc = cls.data_type_class
        return (dtc.uint16(idx), dtc.uint8(subidx))
