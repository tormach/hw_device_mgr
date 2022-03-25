#!/usr/bin/env python3

# https://www.kollmorgen.com/sites/default/files/How%20to%20capture%20and%20use%20WireShark%20trace%20data%20with%20EtherCAT%20applications.pdf
#
# RxPDO/bitlen:  6040/2, 6060/1, 607A/4, 60FE.01/4
# TxPDO/bitlen:  6041/2, 6061/1, 6064/4, 6077/2, 6078/2, 60FD/4, 1002/4, 603F/2
# Data from EtherCAT datagram LRW command:
#   0f00080000000000000000371208ffffffff0800080008003f0010c846000000
# Command:
# $0 0f00080000000000000000371208ffffffff0800080008003f0010c846000000
#      2 1 4 4 2 1 4 2 2 4 4 2
import os
import sys
import re
import enum
import pyshark
from hw_device_mgr.lcec.command import LCECCommand

# $0 ~/elmo.2022-02-24.ethercat_switch_on_disabled.pcapng


class PDO(int):
    def __new__(cls, number, base=10, **kwargs):
        return super().__new__(cls, number, base)

    def __init__(self, x, base=10, *, pdo, length):
        self.pdo = pdo
        self.length = length

    def __str__(self):
        return f"0x{{:0{int(self.length)*2}X}}".format(self)

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

    def __init__(self, pdos, bus=0):
        self.fname = fname
        self.pdos = pdos
        self.cap = None
        self.command = LCECCommand()
        self.bus = bus

    def live_capture(self, **pyshark_kwargs):
        cap = pyshark.LiveCapture(LCECCommand().master_nic())
        cap.sniff(**pyshark_kwargs)
        self.cap = cap

    def file_capture(self, fname):
        assert os.path.exists(fname), f"File '{fname}' doesn't exist"
        cap = pyshark.FileCapture(fname)
        self.cap = cap

    def read_pdo_mapping(self, address, index):
        cmd = self.command
        dtc = cmd.data_type_class
        nr_assgn = cmd.upload(address=address, index=index, datatype=dtc.uint8)
        assert nr_assgn == 1, f"{address:04X}h assignments {nr_assgn} != 1"
        assgn_ix = cmd.upload(
            address=address, index=index, subindex=1, datatype=dtc.uint16
        )
        nr_pdos = cmd.upload(
            address=address, index=assgn_ix, datatype=dtc.uint8
        )
        res = list()
        for i in range(nr_pdos):
            raw = cmd.upload(
                address=address,
                index=assgn_ix,
                subindex=i + 1,
                datatype=dtc.uint32,
            )
            ix = (raw & 0xFFFF0000) >> 16
            subidx = raw & 0x000000FF
            res.append((ix, subidx))
            # FIXME add SDO length
        return res

    # def scan_pdos(self):
    #     cmd = self.command
    #     for address, model_id in self.command.scan_bus(bus=self.bus):
    #         sm2_pdos = self.read_pdo_mapping(address=address, index=0x1C12)
    #         sm2_pdos = self.read_pdo_mapping(address=address, index=0x1C13)

    def timestamp_diff(self, pkt1, pkt2):
        diff = float(pkt1.sniff_timestamp) - float(pkt2.sniff_timestamp)
        return int(diff * 1e9)

    def print_obj(self, obj, maxlen=500):
        for attr in dir(obj):
            if attr.startswith("_"):
                continue
            try:
                val = repr(getattr(obj, attr))
            except Exception:
                print(f"{attr}:  (Couldn't get repr)")
            else:
                val = val if len(val) < maxlen else val[:maxlen]
                print(f"{attr}:  {val}")

    datagram_attr_re = re.compile(r"^sub([0-9]+)_([a-z]+)$")

    def layer_datagrams(self, layer):
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

    def decode_lrw_pdos(self, datagram):
        assert datagram["cmd"] == self.ecat_cmd.LRW
        assert "data" in datagram
        data = datagram["data"]
        raw = data.raw_value
        # Reverse octets so values are in correct order
        octets = "".join(
            raw[i - 2 : i] for i in range(len(raw), 0, -2)  # noqa:  E225
        )
        res = dict()
        for pdo, length in self.pdos:
            # PDO values are in reverse order; pull values off end
            res[pdo] = PDO(
                octets[-length * 2 :],  # noqa:  E203
                base=16,
                pdo=pdo,
                length=length,
            )
            octets = octets[: -length * 2]
        assert octets == ""
        return res

    def delta_stats(self, deltas):
        avg_p = int(sum(deltas) / len(deltas))
        min_p = min(d for d in deltas if d > 0)
        max_p = max(deltas)
        return avg_p, min_p, max_p

    def print_ecat_datagram(self, d):
        if d["cmd"] != self.ecat_cmd.LRW:
            return
        pdos = self.decode_lrw_pdos(d)
        for k, v in pdos.items():
            print(f"  {k}:  {v}")

    def ecat_layers(self, pkt):
        return [i for i in pkt.layers if i.layer_name == "ecat"]

    def lrw_command(self, pkt):
        for layer in self.ecat_layers(pkt):
            for d in self.layer_datagrams(layer):
                if d["cmd"] == self.ecat_cmd.LRW:
                    return d
        return None

    def is_op_enab(self, pkt):
        cmd = self.lrw_command(pkt)
        if not cmd:
            return None
        pdos = self.decode_lrw_pdos(cmd)
        return (pdos["6041h"] & 0x27) == 0x27

    def print_ecat_packet(self, ix, pkt, delta):
        print(f"Packet #{ix}:  time delta (ns): {delta}")
        cmd = self.lrw_command(pkt)
        if not cmd:
            print("  (No LRW command)")
        self.print_ecat_datagram(cmd)

    def read_packets(self):
        assert self.hasattr("cap"), "No file or live capture to process"
        prev_pkts = list()
        print_more = 0
        delta = 0
        for count, pkt in enumerate(self.cap):
            ethercat_layer = pkt.layers[0]
            assert ethercat_layer.layer_name == "eth"
            if ethercat_layer.src_lg == "0":
                # Ignore packets from controller (slaves send packets with 'lg'
                # bit set; maybe not reliable if master does that, but why would
                # it?)
                continue

            if prev_pkts:
                prev_pkt, prev_delta, prev_count = prev_pkts[-1]
                delta = self.timestamp_diff(pkt, prev_pkt)
                if delta > 2500000 and self.is_op_enab(prev_pkt):
                    # Print previous packet:  After >~2.5ms, already disabled
                    self.print_ecat_packet(prev_count, prev_pkt, prev_delta)
                    print_more = 4
            if print_more:
                print_more -= 1
                self.print_ecat_packet(count, pkt, delta)
                if not print_more:
                    print("----------------------")

            prev_pkts.append((pkt, delta, count))
            if len(prev_pkts) > 10:
                prev_pkts.pop(0)


if __name__ == "__main__":
    fname = sys.argv[1]
    pdos = [
        ("6040h", 2),
        ("6060h", 1),
        ("607Ah", 4),
        ("60FE-01h", 4),
        ("6041h", 2),
        ("6061h", 1),
        ("6064h", 4),
        ("6077h", 2),
        ("6078h", 2),
        ("60FDh", 4),
        ("1002h", 4),
        ("603Fh", 2),
    ]

    d = EcatDecoder(pdos)
    d.file_capture(fname)
    d.read_packets()
