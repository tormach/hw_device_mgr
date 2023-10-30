from .sdo import EtherCATSDO
from ..config_io import ConfigIO
from ..logging import Logging
from lxml import etree
from pprint import pprint
from functools import lru_cache

__all__ = ("EtherCATXMLReader",)


class EtherCATXMLReader(ConfigIO):
    """Parse EtherCAT Slave Information "ESI" XML files."""

    sdo_class = EtherCATSDO
    _device_registry = dict()

    default_datatypes_package = "hw_device_mgr.ethercat"
    default_datatypes_resource = "esi_base_types.xml"

    logger = Logging.getLogger(__name__)

    @classmethod
    def str_to_int(cls, s):
        if s.startswith("#x"):
            return int(s[2:], 16)
        else:
            return int(s, 10)

    @classmethod
    def uint(cls, num, numbits=16):
        dtc = cls.sdo_class.data_type_class
        return getattr(dtc, f"uint{numbits}")(num)

    @property
    def data_type_class(self):
        return self.sdo_class.data_type_class

    def __init__(self, tree, LcId="1033"):
        """Init object with locale ID."""
        self.tree = tree
        self.LcId = LcId  # Discard anything not in this locale

    def safe_set(self, dst, key, val, prefix=None):
        """
        Set dictionary key without clobbering.

        Set `dst[key]=val`, and raise an exception if the change would
        clobber an old value; if `prefix` is set, prepend to `key` if
        the change would clobber.
        """
        if key not in dst:  # No conflict
            dst[key] = val
            return

        elif dst[key] == val:  # Already set & value matches
            return

        elif prefix is None:  # Top-level obj already set but value mismatch
            print(f"Unsafe set '({prefix}){key}' = {repr(val)} processing dst:")
            pprint(dst)
            raise RuntimeError(f"Unsafe set key {key} = val {val}")

        else:  # 2nd-level obj; try adding prefix
            try:
                self.safe_set(dst, prefix + key, dst[key])
                dst[key] = val
                return
            except RuntimeError as e:
                print(
                    f"Exception setting '({prefix}){key}' = {repr(val)}; dst:"
                )
                pprint(dst)
                raise e

    def safe_update(self, dst, src, prefix=None):
        """
        Update dict like `dict.update()` non-destructively.

        Update a copy of `dst` with `src`, raising exceptions and with
        `prefix` arg like in the `safe_set` method.

        Return the copied dict.
        """
        dst_copy = dst.copy()
        for key, val in src.items():
            try:
                self.safe_set(dst_copy, key, val, prefix)
            except RuntimeError as e:
                print(f"Exception updating '({prefix}){key}' = '{val}'; src:")
                pprint(src)
                raise e
        return dst_copy

    def tree_to_obj(self, tree):
        """
        Translate an XML element into a Python object.

        The resulting object can be printed and won't lose information;
        used in fall-through cases.
        """
        return dict(
            tag=tree.tag,
            subelements=[self.tree_to_obj(e) for e in tree],
            attributes=dict(tree.items()),
            text=tree.text,
        )

    def read_object(self, obj, subindex=None):
        # Recursively traverse a datatype or object XML element and
        # turn it into a simpler Python data structure
        res = dict()
        for subobj in obj:
            key = subobj.tag
            # Locale may introduce duplicate <Name> elements; throw
            # away if locale doesn't match
            if "LcId" in subobj.keys() and subobj.get("LcId") != self.LcId:
                continue
            if key in res:  # Sanity
                raise RuntimeError(
                    "Duplicate key '%s': %s"
                    % (key, etree.tostring(obj).decode())
                )
            if key in {"Index"}:  # e.g. #x1000
                if not subobj.text.startswith("#x"):  # Sanity
                    raise RuntimeError(
                        'Key "%s" value "%s" should start with "#x"'
                        % (key, subobj.text)
                    )
                res[key] = self.uint(self.str_to_int(subobj.text))
            elif key in {"MinValue", "MaxValue", "DefaultValue"}:
                # e.g. -32767, 00 (?!?), #x0001
                t = subobj.text
                res[key] = self.str_to_int(t)
            elif key in {"MinData", "MaxData"}:
                # e.g. 78EC, FFFFFFFF, 00
                res[key] = int(subobj.text, 16)
            elif key == "DefaultData":
                res[key] = int(subobj.text, 16)
            elif key in {
                "Name",
                "Type",
                "BaseType",
                "Category",
                "Access",
                "PdoMapping",
                "DefaultString",
            }:
                res[key] = subobj.text.rstrip()
            elif key in {"BitSize", "BitOffs", "SubIdx", "LBound", "Elements"}:
                res[key] = int(subobj.text)
            elif key in {"SubIndex", "BitLen"}:  # RxPdo, TxPdo
                res[key] = int(subobj.text)
            elif key in {"DataType"} and len(subobj) == 0:  # RxPdo, TxPdo
                res[key] = subobj.text
            elif key in {"Backup", "Setting"}:
                res[key] = int(subobj.text)
            elif key in {"Info", "Flags", "ArrayInfo"}:
                res[key] = self.read_object(subobj)
            elif key == "SubItem":
                subitem = self.read_object(subobj)
                res.setdefault("SubItems", list()).append(subitem)
            elif (
                type(key).__name__ == "cython_function_or_method"
                and key.func_name == "Comment"
            ):
                res["Comment"] = subobj.text
            else:
                # WTF:  Don't lose any data
                print(key)
                pprint(type(key))
                pprint(type(key).__name__)
                res[key] = self.tree_to_obj(subobj)
        if subindex is not None:
            if "SubIndex" in res:  # Sanity
                raise RuntimeError("'SubIndex' redefined")
            res["SubIndex"] = subindex
        return res

    @property
    def vendor_xml(self):
        """Return the XML `Vendor` element."""
        # Main file structure:
        # <EtherCATInfo>
        #   <Vendor>
        #     <Id/>
        #     <Name/>
        #     <URL/>
        #   </Vendor>
        #   [...]
        # </EtherCATInfo>
        vendors = self.tree.xpath("/EtherCATInfo/Vendor")
        if len(vendors) != 1:
            raise RuntimeError(f"{len(vendors)} <Vendor> sections in XML")
        return vendors[0]

    @property
    def vendor_id(self):
        id_str = self.vendor_xml.xpath("Id")[0].text  # Should only ever be one
        return self.uint(self.str_to_int(id_str))

    @property
    def devices_xml(self):
        """Return the list of XML `Device` elements."""
        # Main file structure:
        # <EtherCATInfo>
        #   <Vendor/>  <!-- not parsed -->
        #   <Descriptions>
        #     <Groups/>  <!-- not parsed -->
        #     <Devices>  <!-- returns <Device/> child elements -->
        #       <Device/>
        #       [...]
        #     </Devices>
        #   </Descriptions>
        # </EtherCATInfo>
        return self.tree.xpath("/EtherCATInfo/Descriptions/Devices/Device")

    """Map XML data types to `ethercat -t TYPE`

    http://www.dige.ai/uploadfiles/2020/01/20200109105424701.pdf
    https://etherlab.org/download/ethercat/ethercat-1.5.2.pdf

    Format: 'XML_type': 'etherlab_type' # NUMBITS Description
    """

    # Data type definitiens
    #
    # <DataTypes>
    #   <DataType>
    #     <!--Std type (see ETG.2000)-->
    #     <Name>USINT</Name>
    #     <BitSize>8</BitSize>
    #   </DataType>
    #   <DataType>
    #     <Name>DT1018</Name>
    #     <BitSize>144</BitSize>
    #     <SubItem>
    #       <SubIdx>0</SubIdx>
    #       <Name>SubIndex 000</Name>
    #       <Type>USINT</Type>
    #       <BitSize>8</BitSize>
    #       <BitOffs>0</BitOffs>
    #       <Flags>
    #         <Access>ro</Access>
    #       </Flags>
    #     </SubItem>
    #     [...]
    #   </DataType>
    #   [...]

    def expand_subitems(self, subitems):
        """Translate `SubItem` objects within complex `DataType` objects."""
        subidx = 0
        expanded_subitems = list()
        for subitem in subitems:
            # Non-array subitems specify `<SubIdx/>` tag with int subindex
            if "SubIdx" in subitem:
                subidx = subitem["SubIdx"]
                expanded_subitems.append(subitem)
                continue

            # If no subindex, must be an array
            if not subitem["Type"].endswith("ARR"):
                raise RuntimeError("Can't find SubIdx or array in type")

            # Array subitem
            # print("subitem:", subitem)
            array_type = self.datatypes[subitem["Type"]].copy()
            # print("array_type:", array_type)
            array_info = array_type.pop("ArrayInfo")
            self.safe_set(array_type, "Type", array_type.pop("BaseType"))
            array_type.pop("TypeName")
            elements = array_info["Elements"]
            assert array_info["LBound"] == subidx + 1
            for i in range(elements):
                subidx += 1
                new_subitem = self.safe_update(array_type, subitem, "Array")
                self.safe_set(new_subitem, "SubIdx", subidx)
                self.safe_set(new_subitem, "ArrayName", new_subitem.pop("Name"))
                self.safe_set(new_subitem, "AryBitSize", new_subitem["BitSize"])
                new_subitem["BitSize"] = int(
                    new_subitem["AryBitSize"] / elements
                )
                new_subitem["Ary"] = True
                # print("new_subitem:", new_subitem)
                expanded_subitems.append(new_subitem)
        return expanded_subitems

    def massage_type(self, otype, **add_keys):
        """Parse a `DataType` object."""
        if "SubItems" in otype:
            otype["OldSubItems"] = old_subitems = otype.pop("SubItems")
            otype["SubItems"] = self.expand_subitems(old_subitems)
        self.safe_set(otype, "TypeName", otype.pop("Name"))
        key = otype["TypeName"]
        if key in self.datatypes:  # Sanity
            raise RuntimeError("Duplicate datatype '%s'" % key)
        if "Name" in otype:
            raise RuntimeError('Found "Name" attr in type')
        if add_keys:
            self.safe_update(otype, add_keys)
        self.datatypes[key] = otype

    def read_datatypes(self, device):
        """Parse device `<DataType/>` elements into `self.datatypes` dict."""
        self.datatypes = dict()
        # First process <DataType/> without <SubItem/> children...
        for dt in device.xpath(
            "Profile/Dictionary/DataTypes/DataType[not(SubItem)]"
        ):
            self.massage_type(self.read_object(dt))
        # ...So any complex <DataType><SubItem><Type/> will already be known
        for dt in device.xpath(
            "Profile/Dictionary/DataTypes/DataType[SubItem]"
        ):
            self.massage_type(self.read_object(dt))

    def read_default_datatypes(self):
        """Read default datatypes for ESI files without them."""
        rsrc = (self.default_datatypes_package, self.default_datatypes_resource)
        with self.open_resource(*rsrc) as f:
            tree = etree.parse(f)
        dts = tree.xpath("/DataTypes/DataType")
        assert len(dts), f"Unable to parse {rsrc}"
        for dt in tree.xpath("/DataTypes/DataType"):
            self.massage_type(self.read_object(dt), from_defaults=True)

    @classmethod
    def is_base_type(cls, name):
        # EtherCAT ESI device descriptions have complex data types
        try:
            cls.sdo_class.data_type_class.by_name(name)
        except KeyError:
            return False
        else:
            return True

    def type_data_list(self, o):
        type_name = o["Type"]
        otype = self.datatypes[type_name].copy()  # Manipulated below
        otypes = []
        for i in range(len(otype.get("SubItems", range(1)))):
            otypes.append(self.type_data(type_name, i))
        return otypes

    def type_data(self, type_name, type_idx=0):
        """
        Return type data for an object.

        Include `SubItem` objects if present.
        """
        otype = self.datatypes[type_name].copy()  # Manipulated below
        if "SubItems" in otype:
            subitems = otype.pop("SubItems")
            if type_idx >= len(subitems):
                pprint(otype)
                pprint(subitems)
                raise RuntimeError(
                    "Type %s has no SubItem %d" % (type_name, type_idx)
                )
            subtype = subitems[type_idx].copy()
            for key, val in otype.items():
                if key in subtype:
                    subtype["ParentType%s" % key] = val
                else:
                    subtype[key] = val
            subtype.pop("OldSubItems", None)
            otype = subtype
        if "TypeName" not in otype:
            pprint(otype)
            raise RuntimeError("Type %s not expanded" % otype["Name"])
        type_name = otype.get("TypeName")
        if self.is_base_type(type_name):
            self.safe_set(otype, "Type", type_name)
            self.safe_set(
                otype, "DataType", self.data_type_class.by_name(type_name)
            )
        return otype

    def read_device_type(self, device):
        return device.xpath("Type")[0]  # Should only ever be one

    # Object definitions
    #
    # <Objects>
    # 	<Object>
    # 		<Index>#x1000</Index>
    # 		<Name>Device type</Name>
    # 		<Type>UDINT</Type>
    # 		<BitSize>32</BitSize>
    # 		<Info>
    # 			<DefaultValue>#x00020192</DefaultValue>
    #           <SubItem>
    #               <Name>SubIndex 000</Name>
    #               <Info>
    #                   <DefaultValue>25</DefaultValue>
    #               </Info>
    #           </SubItem>
    #           [...]
    # 		</Info>
    # 		<Flags>
    # 			<Access>ro</Access>
    # 			<Category>o</Category>
    # 		</Flags>
    # 	</Object>
    #   [...]

    def read_objects(self, device):
        """
        Parse a device's XML `<Object/>` tags into simple Python object.

        Populate type data.
        """
        sdos = dict()

        # Read data types first
        self.read_datatypes(device)
        if not self.datatypes:
            self.read_default_datatypes()

        # Build object dictionary
        for obj in device.xpath("Profile/Dictionary/Objects/Object"):
            # Basic object
            o = self.read_object(obj)
            # Type data
            otypes = self.type_data_list(o)

            # pprint(o)
            # Flatten out subindexes
            for i, otype in enumerate(otypes):
                # Make copy and update with any Info/SubItems
                osub = o.copy()
                if len(o.get("Info", dict()).get("SubItems", [])) > i:
                    osubupdate = o["Info"]["SubItems"][i]
                    osub.pop("Info")
                    osub = self.safe_update(osub, osubupdate, "Parent")

                # Add type data
                osub = self.safe_update(osub, otype, "Parent")
                if "ArrayType" in osub:
                    type_name = osub["ArrayType"]
                else:
                    type_name = osub["Type"]
                try:
                    ecat_type = self.data_type_class.by_name(type_name)
                except KeyError as e:
                    print(self.data_type_class._name_re_registry)
                    raise KeyError(f"Reading XML:  {str(e)}")
                self.safe_set(osub, "DataType", ecat_type)

                # Flatten out Flags, Info
                flags = osub.pop("Flags")
                if flags:
                    for a in ("Access", "Category", "PdoMapping"):
                        v = flags.pop(a, None)
                        if v is not None:
                            osub[a] = v
                info = osub.pop("Info", None)
                if info:
                    for a in (
                        "DefaultValue",
                        "DefaultData",
                        "DefaultString",
                        "MinValue",
                        "MaxValue",
                        "MinData",
                        "MaxData",
                    ):
                        v = info.pop(a, None)
                        if v is not None:
                            osub[a] = v
                # ParentName is like an index name
                if "ParentName" in osub:
                    osub["IndexName"] = osub.pop("ParentName")
                # Remove ParentFlags:  "Access" and "Category" of
                # unknown value
                # osub.pop("ParentFlags", None)
                for a in (
                    "ParentType",
                    "ParentBitSize",
                    "ParentTypeBitSize",
                    "ParentFlags",
                ):
                    osub.pop(a, None)

                # Add to objects dict
                ix = (
                    self.uint(osub["Index"]),
                    self.uint(osub.get("SubIdx", 0), 8),
                )
                assert ix not in sdos, f"Duplicate SDO {ix}: {osub}"
                sdos[ix] = osub
        # print(f"Unused:  {list(self._unused.keys())}")
        return sdos

    def munge_pdo_entry(self, pdo_entry, pdo_type):
        # Munge an RxPdo/TxPdo Entry to be treated as SDO
        o = self.read_object(pdo_entry)
        dtc = self.data_type_class
        # Munge field names & values
        o["Index"] = self.uint(o["Index"])
        if o["Index"] == 0x00:
            return o  # Some Elmo ESI [RT]xPdo final entry is zero
        o["SubIdx"] = self.uint(o.get("SubIndex", 0x00), 8)
        o["Type"] = o.pop("DataType")
        o["DataType"] = dtc.by_name(o["Type"])
        if "BitLen" not in o:
            pprint(o)
            raise KeyError("No 'BitLen' subelement in PDO")
        o["BitSize"] = o.pop("BitLen")
        # Add implicit fields
        o["Access"] = "rw" if pdo_type == "RxPdo" else "ro"
        o["PdoMapping"] = "R" if pdo_type == "RxPdo" else "T"
        o["from_pdo"] = pdo_type
        return o

    def munge_pdo(self, obj, pdo_type):
        o = dict(
            SubIdx=0x00,
            Type="USINT",
            DataType=self.data_type_class.uint8,
            BitSize="8",
            Access="ro",
            Name="SubIndex 000",
        )
        for subobj in obj:
            if subobj.tag == "Index":
                o["Index"] = self.uint(self.str_to_int(subobj.text))
            elif subobj.tag == "Name":
                o["IndexName"] = subobj.text.rstrip()
            elif subobj.tag in {"Entry", "Exclude"}:
                pass
            else:
                raise RuntimeError(f"Unknown {pdo_type} tag {subobj.tag}")
        return o

    def read_fixed_pdo_entries(self, device, sdo_data):
        pdos = dict()
        for pdo_type in ("RxPdo", "TxPdo"):
            # Parse RxPdo & TxPdo elements
            for obj in device.xpath(f"{pdo_type}"):
                data = self.munge_pdo(obj, pdo_type)
                ix = (data.get("Index"), data.get("SubIndex", 0))
                assert ix not in pdos, f"Duplicate PDO mapping:  {data}"
                if ix not in sdo_data:
                    pdos[ix] = data
            # Parse RxPdo & TxPdo elements Entry child elements
            for obj in device.xpath(f"{pdo_type}[@Fixed='1']/Entry"):
                data = self.munge_pdo_entry(obj, pdo_type)
                ix = (data.get("Index"), data.get("SubIndex", 0))
                if ix[0] == 0x00:
                    continue  # Some Elmo ESI [RT]xPdo final entry is zero
                assert ix not in pdos, f"Duplicate PDO entry:  {data}"
                if ix not in sdo_data:
                    pdos[ix] = data
        return pdos

    sdo_translations = dict(
        # Translate SDO data from read_objects() to SDOs.add_sdo() args
        Index="index",
        SubIdx="subindex",
        DataType="data_type",
        Name="name",
        IndexName="index_name",
        # `Info`
        DefaultValue="default_value",
        MinValue="min_value",
        MaxValue="max_value",
        # `Flags`
        Access="access",  # Needs further munging
        PdoMapping="pdo_mapping",
    )

    def add_sdo(self, sdos, data):
        sdo = dict()
        for key_src, key_dst in self.sdo_translations.items():
            sdo[key_dst] = data.get(key_src, None)
        sdo["ro"] = sdo.pop("access", "ro") == "ro"
        sdo["data_type"] = sdo["data_type"].shared_name
        idx = sdo["index"] = self.uint(sdo.pop("index"))
        subidx = sdo["subindex"] = self.uint(sdo.pop("subindex") or 0, 8)
        assert (idx, subidx) not in sdos
        sdos[idx, subidx] = sdo

    # DC definitions
    #
    # <Dc>
    # 	<OpMode>
    # 		<Name>DC</Name>
    # 		<Desc>DC-Synchron</Desc>
    #       <AssignActivate>#x300</AssignActivate>
    #       <CycleTimeSync0 Factor="1">0</CycleTimeSync0>
    #       <ShiftTimeSync0>0</ShiftTimeSync0>
    #       <CycleTimeSync1 Factor="1">0</CycleTimeSync1>
    #     </OpMode>
    #   </Dc>

    def read_dc_opmodes(self, device):
        """Parse XML `<Dc><OpMode/></Dc>` tags into simple Python object."""
        opmodes = list()
        for obj in device.xpath("Dc/OpMode"):
            opmode = dict()
            for subobj in obj:
                key = subobj.tag
                if key in {
                    "AssignActivate",
                    "CycleTimeSync0",
                    "ShiftTimeSync0",
                    "CycleTimeSync1",
                }:
                    opmode[key] = self.str_to_int(subobj.text)
                else:
                    opmode[key] = subobj.text
            opmodes.append(opmode)
        return opmodes

    def device_model_id(self, device_xml):
        device_type = self.read_device_type(device_xml)
        product_code = self.str_to_int(device_type.get("ProductCode"))
        model_id = tuple(
            self.uint(i, 32) for i in (self.vendor_id, product_code)
        )
        return model_id

    @lru_cache
    def parse_sdos(self):
        """Parse device SDO info from ESI XML."""
        model_sdos = dict()
        for dxml in self.devices_xml:
            model_id = self.device_model_id(dxml)
            reg = self._device_registry.setdefault(model_id, dict())
            model_sdos[model_id] = reg["sdos"] = dict()
            sdo_data = self.read_objects(dxml)
            for sd in sdo_data.values():
                self.add_sdo(reg["sdos"], sd)
            pdo_data = self.read_fixed_pdo_entries(dxml, sdo_data)
            for pd in pdo_data.values():
                self.add_sdo(reg["sdos"], pd)

        return model_sdos

    @lru_cache
    def parse_dc_opmodes(self):
        """Parse device DC OpModes info from ESI XML."""
        dc_opmodes = dict()
        for dxml in self.devices_xml:
            model_id = self.device_model_id(dxml)
            opmodes = self.read_dc_opmodes(dxml)
            assert isinstance(opmodes, list)
            dc_opmodes[model_id] = opmodes
        return dc_opmodes

    @classmethod
    @lru_cache
    def read_from_resource(cls, package, resource, LcId="1033"):
        cls.logger.info(f"Reading ESI from ({package}, {resource})")
        with cls.open_resource(package, resource) as f:
            tree = etree.parse(f)
        return cls(tree, LcId=LcId)

    @classmethod
    @lru_cache
    def read_from_path(cls, fpath, LcId="1033"):
        cls.logger.info(f"Reading ESI from {fpath}")
        with cls.open_path(fpath) as f:
            tree = etree.parse(f)
        return cls(tree, LcId=LcId)
