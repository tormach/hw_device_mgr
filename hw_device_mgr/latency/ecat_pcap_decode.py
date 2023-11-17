#!/usr/bin/env python3

# https://www.kollmorgen.com/sites/default/files/How%20to%20capture%20and%20use%20WireShark%20trace%20data%20with%20EtherCAT%20applications.pdf
#
import sys
import re
import enum
import pyshark
import subprocess
import tempfile
import struct
import time as _time
from functools import cached_property
import argparse
from lxml import etree


class PDO(int):
    def __new__(cls, number, base=10, **kwargs):
        return super().__new__(cls, number, base)

    def __init__(self, x, base=10, *, pdo, length):
        self.pdo = pdo
        self.length = length

    def __str__(self):
        return f"0x{{:0{int(self.length) * 2}X}}".format(self)

    def __repr__(self):
        return f"<PDO {self.pdo}={str(self)}>"


class EcatDecoder:
    class ecat_cmd(enum.IntEnum):
        # https://infosys.beckhoff.com/content/1033/tc3_io_intro/1257993099.html
        NOP = 0
        APRD = 1
        APWR = 2
        APRW = 3
        FPRD = 4
        FPWR = 5
        FPRW = 6
        BRD = 7
        BWR = 8
        BRW = 9
        LRD = 10
        LWR = 11
        LRW = 12
        ARMW = 13
        # Above link doesn't list FRMW, but pcap dissectors do
        FRMW = 14

    def ethercat_capture(self, callback, time=30):
        # For non-zero time, flush pcap data first & wait for new data
        if time != 0:
            sys.stderr.write(f"Flushing pcap data and waiting {time} seconds\n")
            cmd = ["ethercat", "pcap", "-r"]
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
            _time.sleep(time)
        # Write pcap data to temp file and run callback
        with tempfile.NamedTemporaryFile() as pcap_f:
            cmd = ["ethercat", "pcap"]
            subprocess.run(cmd, check=True, stdout=pcap_f)
            cap = pyshark.FileCapture(input_file=pcap_f.name)
            return callback(cap)

    def file_capture(self, fname, callback):
        cap = pyshark.FileCapture(fname)
        return callback(cap)

    datagram_attr_re = re.compile(r"^sub([0-9]+)_([a-z]+)$")

    def eth_data(self, pkt):
        for layer in pkt.layers:
            if layer.layer_name == "eth":
                # Slaves send packets with MAC set to master's and 'LG' bit (17)
                # set (denoting local, not factory MAC address).
                src_controller = layer.src_lg == "0"
                break
        timestamp = float(pkt.sniff_timestamp)
        return src_controller, timestamp

    @cached_property
    def eth_headers(self):
        return ("timestamp", "src_master")

    def ecat_layers(self, pkt):
        for layer in pkt.layers:
            if layer.layer_name != "ecat":
                continue
            yield layer

    def ecat_layer_datagrams(self, pkt):
        for layer in self.ecat_layers(pkt):
            datagrams = dict()
            for attr in layer.field_names:
                m = self.datagram_attr_re.match(attr)
                if m:
                    ix, field = m.groups()
                    datagram = datagrams.setdefault(ix, dict(ix=ix))
                    datagram[field] = getattr(layer, attr)
            for ix, datagram in datagrams.items():
                datagram["cmd"] = self.ecat_cmd(int(datagram.pop("cmd"), 16))
                yield datagram

    def lrw_command(self, pkt):
        for d in self.ecat_layer_datagrams(pkt):
            if d["cmd"] == self.ecat_cmd.LRW:
                # There will only ever be one ECAT layer with one datagram
                return d
        return None  # Packet has no ECAT layers w/no LRW cmd datagrams

    def lrw_command_values(self, pkt):
        cmd = self.lrw_command(pkt)
        if cmd is None:
            return None
        assert "data" in cmd
        values = self.compiled_struct.unpack(cmd["data"].binary_value)
        return values

    @cached_property
    def compiled_struct(self):
        obj = struct.Struct(self.struct_format)
        sys.stderr.write(f"PDO data size={obj.size} fmt='{obj.format}'\n")
        return obj

    def read_packets(self, cap, max_count=None):
        count_matched = 0
        self.data = data = list()
        for count, pkt in enumerate(cap):
            # Loop counting
            if max_count is not None and count >= max_count:
                break
            if count % 100 == 0:
                sys.stderr.write(".")  # Show progress
                sys.stderr.flush()

            # Decode values from packet, if applicable
            values = self.lrw_command_values(pkt)
            if values is None:
                # Not an EtherCAT packet with LRW command
                continue
            count_matched += 1
            data.append((self.eth_data(pkt), values))

        sys.stderr.write(f"\nPackets processed:  {count_matched} of {count}\n")

    def print_csv_header(self):
        eth_headers = '","'.join(self.eth_headers)
        ecat_headers = '","'.join(self.entry_names)
        print(f'"{eth_headers}","{ecat_headers}"')

    def print_csv_rows(self):
        for eth_data, ecat_data in self.data:
            src_controller, timestamp = eth_data
            eth_data_str = f'{timestamp:.6f},"{src_controller}"'
            ecat_data_str = ",".join(str(i) for i in ecat_data)
            print(f"{eth_data_str},{ecat_data_str}")

    def print_csv(self):
        self.print_csv_header()
        self.print_csv_rows()

    format_char_map = {
        # Map number of bytes to struct format character
        1: "b",
        2: "h",
        4: "i",
    }
    signed_hal_types = {"s32", "float"}

    def parse_ethercat_xml(self, fname):
        with open(fname) as f:
            tree = etree.parse(f)

        names = list()
        struct_format = "<"  # Header:  little-endian
        masters = tree.xpath("//masters/master")
        for master in masters:
            master_name = master.get("name")
            if master_name is None:
                master_name = master.get("idx")
            # sys.stderr.write(f"master {master_name}\n")

            slaves = master.xpath("slave")
            sys.stderr.write(f"master {master_name}:  {len(slaves)} slaves\n")

            for slave in slaves:
                slave_name = slave.get("name")
                if slave_name is None:
                    slave_name = slave.get("idx")

                # sys.stderr.write(f"  slave {slave_name}\n")

                entries = slave.xpath("syncManager/pdo/pdoEntry")
                sys.stderr.write(
                    f"  slave {slave_name}:  {len(entries)} entries\n"
                )
                for entry in entries:
                    # Construct name
                    idx = entry.get("idx")
                    sub_idx = entry.get("subIdx")
                    hal_pin = entry.get("halPin")
                    name = "{}.{}.{}{}h{}".format(
                        master_name,
                        slave_name,
                        idx.upper(),
                        "" if sub_idx is None else f"-{sub_idx.upper()}",
                        "" if hal_pin is None else f".{hal_pin}",
                    )
                    names.append(name)

                    # Construct format character
                    num_bytes = int(int(entry.get("bitLen")) / 8)
                    format_char = self.format_char_map[num_bytes]
                    signed = (
                        entry.get("halType").lower() in self.signed_hal_types
                    )
                    if not signed:
                        format_char = format_char.upper()
                    struct_format += format_char
                    sys.stderr.write(
                        f"    {name}: {num_bytes} bytes; signed={signed}\n"
                    )

        self.entry_names = names
        self.struct_format = struct_format

    @classmethod
    def cli(cls):
        parser = argparse.ArgumentParser(
            description="Capture and extract EtherCAT process data."
        )

        parser.add_argument(
            "--conf", type=str, help="ethercat.conf.xml file path"
        )
        parser.add_argument(
            "--ethercat-pcap",
            action="store_true",
            help="Use `ethercat pcap` as source",
        )
        parser.add_argument(
            "--time",
            type=int,
            default=30,
            help=(
                "Collect this many seconds of traffic."
                "  With `--ethercat-pcap`, reset pcap data and wait this many"
                " seconds before dump. (0:  don't reset and use existing data)"
            ),
        )
        parser.add_argument(
            "--pcap-file", type=str, help="Use named file as pcap source"
        )

        args = parser.parse_args()
        obj = cls()
        obj.parse_ethercat_xml(args.conf)
        if args.ethercat_pcap:
            # Read from `ethercat pcap` output
            obj.ethercat_capture(obj.read_packets, time=args.time)
        elif args.pcap_file:
            # Read from pcap file
            obj.file_capture(args.pcap_file, obj.read_packets)
        obj.print_csv()


def main():
    sys.exit(EcatDecoder.cli())


if __name__ == "__main__":
    main()
