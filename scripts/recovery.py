#!/usr/bin/env python3
"""Mass-erase, flash, or factory-reset a CMSIS-DAP target via pyOCD.

Chips that enable NVM write protection after a bad flash or crash loop can be
recovered with a mass erase (e.g. nRF54L15 fails OpenOCD with 0xe000ed00).
"""

from __future__ import annotations

import argparse
import logging
import os
import subprocess
import sys
from typing import List

try:
    from pyocd.probe.aggregator import DebugProbeAggregator
except ImportError:
    print("[ERROR] pyocd not installed. Run: uv sync", file=sys.stderr)
    sys.exit(5)

LOG = logging.getLogger("recovery")


def configure_logging() -> None:
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
    LOG.addHandler(handler)
    LOG.setLevel(logging.INFO)


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Recover / flash a target via CMSIS-DAP (pyOCD)."
    )
    parser.add_argument(
        "--mode",
        choices=["erase", "flash", "factory"],
        required=True,
        help="erase = mass erase only; flash = flash only; factory = erase then flash",
    )
    parser.add_argument("--probe", help="CMSIS-DAP probe unique ID (e.g. 06E4B471).")
    parser.add_argument(
        "--firmware",
        help="Firmware image path, e.g. merged.hex (required for flash/factory modes).",
    )
    parser.add_argument(
        "--frequency",
        type=int,
        default=4_000_000,
        help="SWD frequency in Hz (default 4000000).",
    )
    parser.add_argument(
        "--target",
        default="nrf54l",
        help="pyOCD target name (default nrf54l).",
    )
    return parser.parse_args(argv)


def list_probes():
    return DebugProbeAggregator.get_all_connected_probes()


def select_probe(probe_id: str | None) -> str:
    if probe_id:
        for probe in list_probes():
            if probe.unique_id == probe_id:
                LOG.info("Using probe %s (%s)", probe.unique_id, probe.description)
                return probe.unique_id
        LOG.error("Probe %s not found.", probe_id)
        sys.exit(2)

    probes = list_probes()
    if not probes:
        LOG.error("No CMSIS-DAP probes detected. Is the board connected over USB?")
        sys.exit(2)
    if len(probes) == 1:
        LOG.info(
            "Auto-selected probe %s (%s)", probes[0].unique_id, probes[0].description
        )
        return probes[0].unique_id

    LOG.error("Multiple probes detected; pass --probe <id>:")
    for probe in probes:
        LOG.error("  %s : %s", probe.unique_id, probe.description)
    sys.exit(2)


def run_pyocd(args: List[str]) -> int:
    cmd = [sys.executable, "-m", "pyocd"] + args
    LOG.info("Running: %s", " ".join(cmd))
    return subprocess.run(cmd, check=False).returncode


def perform_erase(probe_id: str, target: str) -> None:
    LOG.info("Mass erasing (unlocks write protection if set)...")
    rc = run_pyocd(
        ["erase", "--mass", "--target", target, "--chip", "--probe", probe_id]
    )
    if rc == 0:
        LOG.info("Mass erase succeeded.")
        return

    LOG.warning("Mass erase failed; trying standard chip erase...")
    rc = run_pyocd(["erase", "--target", target, "--chip", "--probe", probe_id])
    if rc != 0:
        LOG.error("Erase failed. Try unplug/replug USB, then rerun.")
        sys.exit(3)
    LOG.info("Standard chip erase succeeded.")


def perform_flash(probe_id: str, firmware: str, freq: int, target: str) -> None:
    LOG.info("Flashing %s...", firmware)
    rc = run_pyocd(
        [
            "flash",
            "--target",
            target,
            "--frequency",
            str(freq),
            firmware,
            "--probe",
            probe_id,
        ]
    )
    if rc != 0:
        LOG.error("Flash failed.")
        sys.exit(4)
    LOG.info("Flash completed.")


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    configure_logging()

    if args.mode in ("flash", "factory"):
        if not args.firmware:
            LOG.error("--firmware is required for %s mode.", args.mode)
            return 5
        if not os.path.isfile(args.firmware):
            LOG.error("Firmware not found: %s", args.firmware)
            return 5

    probe_id = select_probe(args.probe)

    if args.mode in ("erase", "factory"):
        perform_erase(probe_id, args.target)

    if args.mode in ("flash", "factory"):
        perform_flash(probe_id, args.firmware, args.frequency, args.target)

    LOG.info("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
