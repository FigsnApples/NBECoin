#have python 3.9 or above 
# to see which version you have use 'which python'
#or use 'python --version'
#to download a specific version use pip install python==version u want 
# to delete old just do update or pip uninstall python 
# how to run the code (copy n paste the bottom into terminal !)
#install the following as well if u dont have it 
#pip install numpy matplotlib scipy bleak
##
#additional for windows
#pip install bleak[winrt]
##
#cd ~/desktop/Multimodal_MN
#python Multimodal_MN_BLE.py run 
#written by me jose 


#!/usr/bin/env python3
"""
SWV_BLE_BIO.py
- Connects to ESP32_AMPEROMETRIC over BLE
- Subscribes to TWO notify characteristics:
    SWV: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E   (unchanged format)
    BIO: 6E400004-B5A3-F393-E0A9-E50E24DCCA9E   (CSV biomarkers)
- Writes CSV logs, auto-splits "segments" on SWV idle gaps (default: 30s)

SWV notify line format expected (from your firmware):
  "idx:<idx>, step:<step>, phase:<F|R>, I_uA:<float>\n"

BIO notify line format expected:
  "<sys>,<dia>,<hr>,<spo2>,<tmp117_C>,<icmTemp_C>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n"

Install:
  pip install bleak

Run:
  python SWV_BLE_BIO.py run
"""

import argparse
import asyncio
import csv
import os
import re
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Tuple

from bleak import BleakClient, BleakScanner

# ===== BLE UUIDs (match your firmware) =====
SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
SWV_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
BIO_CHAR_UUID = "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"

DEFAULT_DEVICE_NAME = "ESP32_AMPEROMETRIC"

# ===== SWV parse regex =====
SWV_RE = re.compile(
    r"idx:(?P<idx>\d+)\s*,\s*step:(?P<step>\d+)\s*,\s*phase:(?P<phase>[FR])\s*,\s*I_uA:(?P<iua>[-+0-9.eE]+)"
)


@dataclass
class SegmentWriters:
    seg_id: int
    seg_dir: str
    swv_fp: any
    swv_writer: csv.writer
    bio_fp: any
    bio_writer: csv.writer
    combined_fp: any
    combined_writer: csv.writer


def now_tag() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def ensure_dir(p: str) -> str:
    os.makedirs(p, exist_ok=True)
    return p


def safe_float(s: str) -> float:
    try:
        return float(s)
    except Exception:
        return float("nan")


class SegmentManager:
    def __init__(self, root_outdir: str):
        self.root_outdir = ensure_dir(root_outdir)
        self._seg: Optional[SegmentWriters] = None
        self._seg_id = 0

        # simple caches for "combined" rows
        self._last_bio: Optional[Tuple[float, ...]] = None
        self._last_bio_ts: Optional[float] = None

    def set_latest_bio(self, ts: float, vals: Tuple[float, ...]) -> None:
        self._last_bio = vals
        self._last_bio_ts = ts

    def new_segment(self, reason: str) -> None:
        self.close()

        self._seg_id += 1
        seg_dir = ensure_dir(os.path.join(self.root_outdir, f"seg_{self._seg_id:04d}"))
        print(f"[SEG] -> {seg_dir}  ({reason})")

        # SWV CSV
        swv_path = os.path.join(seg_dir, "swv.csv")
        swv_fp = open(swv_path, "w", newline="")
        swv_writer = csv.writer(swv_fp)
        swv_writer.writerow(["ts_epoch", "ts_iso", "idx", "step", "phase", "I_uA"])

        # BIO CSV
        bio_path = os.path.join(seg_dir, "bio.csv")
        bio_fp = open(bio_path, "w", newline="")
        bio_writer = csv.writer(bio_fp)
        bio_writer.writerow(
            ["ts_epoch", "ts_iso",
             "sys", "dia", "hr", "spo2",
             "tmp117_C", "icmTemp_C",
             "ax", "ay", "az",
             "gx", "gy", "gz"]
        )

        # COMBINED CSV (one row per SWV sample + latest BIO snapshot)
        comb_path = os.path.join(seg_dir, "combined.csv")
        combined_fp = open(comb_path, "w", newline="")
        combined_writer = csv.writer(combined_fp)
        combined_writer.writerow(
            ["ts_epoch", "ts_iso",
             "idx", "step", "phase", "I_uA",
             "bio_ts_epoch", "bio_age_s",
             "sys", "dia", "hr", "spo2",
             "tmp117_C", "icmTemp_C",
             "ax", "ay", "az",
             "gx", "gy", "gz"]
        )

        self._seg = SegmentWriters(
            seg_id=self._seg_id,
            seg_dir=seg_dir,
            swv_fp=swv_fp, swv_writer=swv_writer,
            bio_fp=bio_fp, bio_writer=bio_writer,
            combined_fp=combined_fp, combined_writer=combined_writer,
        )

    def close(self) -> None:
        if not self._seg:
            return
        for fp in (self._seg.swv_fp, self._seg.bio_fp, self._seg.combined_fp):
            try:
                fp.flush()
                fp.close()
            except Exception:
                pass
        self._seg = None

    def write_swv(self, ts: float, idx: int, step: int, phase: str, iua: float) -> None:
        if not self._seg:
            self.new_segment("initial")

        iso = datetime.fromtimestamp(ts).isoformat()
        self._seg.swv_writer.writerow([ts, iso, idx, step, phase, iua])

        # combined row uses latest bio snapshot (if available)
        if self._last_bio and self._last_bio_ts is not None:
            bio_age = ts - self._last_bio_ts
            row = [
                ts, iso,
                idx, step, phase, iua,
                self._last_bio_ts, bio_age,
                *self._last_bio
            ]
        else:
            row = [
                ts, iso,
                idx, step, phase, iua,
                "", "",
                *([float("nan")] * 12)
            ]
        self._seg.combined_writer.writerow(row)

    def write_bio(self, ts: float, vals: Tuple[float, ...]) -> None:
        if not self._seg:
            self.new_segment("initial (bio first)")

        iso = datetime.fromtimestamp(ts).isoformat()
        self._seg.bio_writer.writerow([ts, iso, *vals])


async def find_device(name: str, address: Optional[str], scan_timeout: float) -> str:
    if address:
        return address

    print(f"[SCAN] Looking for '{name}' ...")
    dev = await BleakScanner.find_device_by_filter(
        lambda d, ad: (d.name == name) if d and d.name else False,
        timeout=scan_timeout
    )
    if not dev:
        raise RuntimeError(f"Device '{name}' not found. Increase --scan-timeout or pass --address.")
    print(f"[FOUND] ✅ {dev.name} @ {dev.address}")
    return dev.address


def decode_bytes(data: bytearray) -> str:
    try:
        return data.decode("utf-8", errors="ignore").strip()
    except Exception:
        return ""


def parse_swv(line: str) -> Optional[Tuple[int, int, str, float]]:
    m = SWV_RE.search(line)
    if not m:
        return None
    idx = int(m.group("idx"))
    step = int(m.group("step"))
    phase = m.group("phase")
    iua = safe_float(m.group("iua"))
    return idx, step, phase, iua


def parse_bio_csv(line: str) -> Optional[Tuple[float, ...]]:
    # sys,dia,hr,spo2,tmp117_C,icmTemp_C,ax,ay,az,gx,gy,gz
    parts = [p.strip() for p in line.split(",")]
    if len(parts) < 12:
        return None
    vals = tuple(safe_float(p) for p in parts[:12])
    return vals


async def run(args: argparse.Namespace) -> None:
    session_dir = ensure_dir(os.path.join(args.outdir, f"SWV_BIO_{now_tag()}"))
    print(f"[OUT] {session_dir}")

    mgr = SegmentManager(session_dir)
    mgr.new_segment("initial")

    # Track idle segmentation based on SWV notifications
    last_swv_ts = time.time()

    async def idle_watcher():
        nonlocal last_swv_ts
        while True:
            await asyncio.sleep(0.5)
            if not args.split_on_idle:
                continue
            gap = time.time() - last_swv_ts
            if gap >= args.idle_gap_s:
                print(f"[IDLE] No SWV data for {gap:.1f}s -> new segment")
                mgr.new_segment(f"idle_gap_{args.idle_gap_s}s")
                last_swv_ts = time.time()

    address = await find_device(args.name, args.address, args.scan_timeout)

    # We reconnect forever unless --once
    attempt = 0
    while True:
        attempt += 1
        print(f"[CONNECT] Attempt {attempt} -> {address}")

        try:
            async with BleakClient(address, timeout=args.connect_timeout) as client:
                if not client.is_connected:
                    raise RuntimeError("BLE client not connected after context enter.")
                print("[CONNECT] ✅ Connected")

                # Optional: verify characteristics exist
                svcs = await client.get_services()
                if SERVICE_UUID.lower() not in [s.uuid.lower() for s in svcs]:
                    print("[WARN] Service UUID not found in GATT (still trying to subscribe).")

                idle_task = asyncio.create_task(idle_watcher())

                def on_swv(_, data: bytearray):
                    nonlocal last_swv_ts
                    ts = time.time()
                    last_swv_ts = ts

                    line = decode_bytes(data)
                    parsed = parse_swv(line)
                    if not parsed:
                        if args.verbose:
                            print(f"[SWV?] {line}")
                        return
                    idx, step, phase, iua = parsed

                    mgr.write_swv(ts, idx, step, phase, iua)

                    if args.print_swv:
                        print(f"[SWV] idx={idx} step={step} phase={phase} I_uA={iua:.9f}")

                def on_bio(_, data: bytearray):
                    ts = time.time()
                    line = decode_bytes(data)

                    vals = parse_bio_csv(line)
                    if not vals:
                        if args.verbose:
                            print(f"[BIO?] {line}")
                        return

                    mgr.set_latest_bio(ts, vals)
                    mgr.write_bio(ts, vals)

                    if args.print_bio:
                        (sysv, diav, hr, spo2, tmpc, icmt, ax, ay, az, gx, gy, gz) = vals
                        print(f"[BIO] sys={sysv:.0f} dia={diav:.0f} hr={hr:.2f} spo2={spo2:.2f} "
                              f"tmp={tmpc:.3f} icmT={icmt:.2f}")

                # Subscribe
                await client.start_notify(SWV_CHAR_UUID, on_swv)
                await client.start_notify(BIO_CHAR_UUID, on_bio)
                print("[STREAM] Subscribed to SWV + BIO notifications")

                # Run until disconnect
                while client.is_connected:
                    await asyncio.sleep(0.25)

                idle_task.cancel()

        except asyncio.CancelledError:
            raise
        except Exception as e:
            print(f"[ERROR] {type(e).__name__}: {e}")

        if args.once:
            break

        print(f"[RETRY] Reconnecting in {args.reconnect_delay_s:.1f}s ...")
        await asyncio.sleep(args.reconnect_delay_s)

    mgr.close()


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="SWV_BLE_BIO.py")
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="Connect + log SWV and biomarkers")
    r.add_argument("--name", default=DEFAULT_DEVICE_NAME, help="BLE device name to scan for")
    r.add_argument("--address", default=None, help="BLE address/UUID to connect directly (skip scan)")
    r.add_argument("--scan-timeout", type=float, default=12.0)
    r.add_argument("--connect-timeout", type=float, default=20.0)
    r.add_argument("--outdir", default="Data", help="Root output directory")

    r.add_argument("--split-on-idle", action="store_true", default=True,
                   help="Auto-split into new segments if SWV notifications stop")
    r.add_argument("--idle-gap-s", type=float, default=30.0,
                   help="Idle gap seconds to trigger new segment")
    r.add_argument("--reconnect-delay-s", type=float, default=2.0)
    r.add_argument("--once", action="store_true", help="Exit after first disconnect/error")

    r.add_argument("--print-swv", action="store_true", default=False)
    r.add_argument("--print-bio", action="store_true", default=False)
    r.add_argument("--verbose", action="store_true", default=False)

    return p


def main():
    p = build_argparser()
    args = p.parse_args()

    if args.cmd == "run":
        try:
            asyncio.run(run(args))
        except KeyboardInterrupt:
            print("\n[STOP] KeyboardInterrupt")
    else:
        p.print_help()


if __name__ == "__main__":
    main()
