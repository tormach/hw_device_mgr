#!/usr/bin/env python3

import re
import sys
import tempfile
import subprocess
import argparse
import logging


class HalSamplerDecoder:
    logger = logging.getLogger(__name__)

    sampler_pin_re = re.compile(
        r"^.* sampler\.([0-9]+)\.pin.([0-9]+).* <== ([^.]+)$"
    )

    halcmd_show_pin_re = re.compile(
        r"^\s+([0-9]+)\s+"  # Comp ID
        r"(?:([0-9]+)\s+)?"  # Optional Inst ID
        r"(u32|s32|float|bit)\s+"  # Pin type
        r"(IN|OUT|I/O)\s+"  # Pin dir
        r"(TRUE|FALSE|[-0x1-9A-Fe.]+)\s+"  # Pin value
        r"([^\s]+)"  # Pin name
        # The following only apply to Machinekit
        r"(?:\s+([0-9.]+)\s+)?"  # Optional epsilon
        r"([-l]{4})?"  # Optional flags
        r"(?:\s+[<==>]+\s+([^ ]+))?"  # Optional Linked to:
    )
    halcmd_show_pin_cols = (
        "comp_id",
        "inst_id",
        "type",
        "dir",
        "val",
        "name",
        "epsilon",
        "flags",
        "link",
    )
    halcmd_show_pin_conv = (
        int,
        int,
        str,
        str,
        lambda x: (
            True
            if x == "TRUE"
            else (False if x == "FALSE" else float(x) if "." in x else int(x))
        ),
        str,
        float,
        str,
        str,
    )

    @classmethod
    def halcmd_show_pin(cls, glob):
        cmd = ["halcmd", "show", "pin", glob]
        cls.logger.debug(f"running cmd:  {' '.join(cmd)}")
        proc = subprocess.run(cmd, capture_output=True, check=True)
        res = list()
        for line in proc.stdout.splitlines():
            cls.logger.debug(f"  line:  {line}")
            m = cls.halcmd_show_pin_re.match(line.decode("utf-8"))
            if m is None:
                continue  # Skip headers (or fix regex)
            vals_conv = zip(m.groups(), cls.halcmd_show_pin_conv)
            vals_list = (
                None if v is None else conv(v) for v, conv in vals_conv
            )
            vals = {k: v for k, v in zip(cls.halcmd_show_pin_cols, vals_list)}
            res.append(vals)
        return res

    def sampler_curr_depth(self, channel=0):
        # Only run this when sampler is disabled or the results will be stale
        # immediately
        res = self.halcmd_show_pin(f"sampler.{channel}.curr-depth")
        assert len(res) == 1, f"sampler {channel} curr-depth {len(res)} lines"
        return res[0]["val"]

    def enable_sampler(self, channel=0, disable=False):
        val = "0" if disable else "1"
        cmd = ["halcmd", "sets", "lat-debug-enable", val]
        subprocess.run(cmd, capture_output=True, check=True)

    def disable_sampler(self, channel=0):
        self.enable_sampler(channel=channel, disable=True)

    def read_signal_names_from_hal(self, channel=0):
        pin_prefix = f"sampler.{channel}.pin."
        self.sig_names = sig_names = list()
        res = self.halcmd_show_pin(pin_prefix)
        pins = dict()
        for pin in res:
            name = pin["name"]
            assert name.startswith(pin_prefix), f"Unexpected {pin['name']}"
            pin_ix = int(name[len(pin_prefix) :])
            pins[pin_ix] = pin["link"]
        num_pins = len(pins)
        assert num_pins > 0, "No sampler pins found"
        for ix in range(num_pins):
            assert ix in pins, f"No pin {ix} found among {num_pins} pins"
            sig_names.append(pins[ix])

    def run_halsampler(self, time=30, update_frequency=1000, channel=0):
        samples = update_frequency * time
        self.read_signal_names_from_hal(channel=channel)
        self.disable_sampler(channel=channel)
        curr_depth = self.sampler_curr_depth(channel=channel)
        self.enable_sampler(channel=channel)
        with tempfile.NamedTemporaryFile() as sampler_f:
            hs_cmd = f"halsampler -t -c {channel} -n {samples + curr_depth}"
            cmd = ["/bin/bash", "-c", f"{hs_cmd} > {sampler_f.name}"]
            sys.stderr.write(f"Running command:  '{' '.join(cmd)}'\n")
            subprocess.run(cmd, check=True)
            self.raw_data = sampler_f.readlines()[curr_depth:]
        self.disable_sampler(channel=channel)

    def read_halsampler_output(self, f):
        self.raw_data = f.readlines()

    def set_signal_names(self, names):
        self.sig_names = names

    def print_csv_header(self):
        headers_joined = '","'.join(self.sig_names)
        print(f'"sample_number","{headers_joined}"')

    def print_csv_rows(self):
        sys.stderr.write("Processing lines")
        overruns = 0
        for count, line_raw in enumerate(self.raw_data):
            line = line_raw.rstrip().decode("utf-8")
            if line == "overrun":
                overruns += 1
                continue
            csv = line.replace(" ", ",")
            print(csv)
            if count % 100 == 0:
                sys.stderr.write(".")
        if overruns:
            self.logger.warn(f"{overruns} overruns in sampler output")
        sys.stderr.write("\n")

    def print_csv(self):
        self.print_csv_header()
        self.print_csv_rows()

    @classmethod
    def cli(cls):
        parser = argparse.ArgumentParser(
            description="Run halsampler and convert to CSV."
        )

        parser.add_argument(
            "--time",
            type=int,
            default=30,
            help="Sample this many secounds of data (default 30).",
        )
        parser.add_argument(
            "--update-frequency",
            type=int,
            default=1000,
            help="Updates per second in Hz (default 1000).",
        )
        parser.add_argument(
            "--channel",
            type=int,
            default=0,
            help="HAL sampler channel (default 0).",
        )
        parser.add_argument(
            "--halsampler-output",
            type=str,
            default=None,
            help=(
                "halsampler output file to convert to CSV"
                " (don't run halsampler) (requires --names args)"
            ),
        )
        parser.add_argument(
            "--names", nargs="+", default=[], help="Names of fields"
        )
        parser.add_argument(
            "--debug",
            action="store_true",
            help="Set log level to debug",
        )

        args = parser.parse_args()

        obj = HalSamplerDecoder()
        obj.logger.setLevel(logging.DEBUG if args.debug else logging.INFO)

        if args.halsampler_output is None:
            obj.run_halsampler(
                time=args.time,
                update_frequency=args.update_frequency,
                channel=args.channel,
            )
        else:
            obj.set_signal_names(args.names)
            with open(args.halsampler_output, "br") as f:
                obj.read_halsampler_output(f)
        obj.print_csv()


def main():
    sys.exit(HalSamplerDecoder.cli())


if __name__ == "__main__":
    main()
