#!/usr/bin/env python3
"""Capture OpenStride BLE RSC measurements; optionally show a live browser plot."""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import struct
import threading
import time
import webbrowser
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import ClassVar

RSC_MEASUREMENT_UUID = "00002a53-0000-1000-8000-00805f9b34fb"
RSC_SERVICE_UUID = "00001814-0000-1000-8000-00805f9b34fb"

RSC_FLAG_STRIDE_LENGTH = 0x01
RSC_FLAG_TOTAL_DISTANCE = 0x02
RSC_FLAG_RUNNING = 0x04


@dataclass(frozen=True)
class RscSample:
    host_time_s: float
    flags: int
    speed_mps: float
    cadence_spm: int
    stride_length_m: float | None
    total_distance_m: float | None
    running: bool
    raw_hex: str


def parse_rsc_measurement(
    payload: bytes, host_time_s: float | None = None
) -> RscSample:
    """Decode Bluetooth SIG RSC Measurement characteristic payload bytes."""
    if len(payload) < 4:
        raise ValueError(f"RSC payload too short: {payload.hex()}")

    flags = payload[0]
    idx = 1

    speed_raw = struct.unpack_from("<H", payload, idx)[0]
    idx += 2
    speed_mps = speed_raw / 256.0

    cadence_spm = payload[idx]
    idx += 1

    stride_length_m: float | None = None
    if flags & RSC_FLAG_STRIDE_LENGTH:
        if len(payload) < idx + 2:
            raise ValueError(f"RSC stride length missing: {payload.hex()}")
        stride_raw = struct.unpack_from("<H", payload, idx)[0]
        idx += 2
        stride_length_m = stride_raw / 100.0

    total_distance_m: float | None = None
    if flags & RSC_FLAG_TOTAL_DISTANCE:
        if len(payload) < idx + 4:
            raise ValueError(f"RSC total distance missing: {payload.hex()}")
        distance_raw = struct.unpack_from("<I", payload, idx)[0]
        idx += 4
        total_distance_m = distance_raw / 10.0

    return RscSample(
        host_time_s=time.monotonic() if host_time_s is None else host_time_s,
        flags=flags,
        speed_mps=speed_mps,
        cadence_spm=cadence_spm,
        stride_length_m=stride_length_m,
        total_distance_m=total_distance_m,
        running=bool(flags & RSC_FLAG_RUNNING),
        raw_hex=payload.hex(" "),
    )


def open_csv(path: Path) -> tuple[object, csv.DictWriter]:
    path.parent.mkdir(parents=True, exist_ok=True)
    fp = path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(
        fp,
        fieldnames=[
            "elapsed_s",
            "host_time_s",
            "flags",
            "speed_mps",
            "cadence_spm",
            "stride_length_m",
            "total_distance_m",
            "running",
            "raw_hex",
        ],
    )
    writer.writeheader()
    return fp, writer


def write_csv_row(
    writer: csv.DictWriter, start_time_s: float, sample: RscSample
) -> None:
    writer.writerow(
        {
            "elapsed_s": f"{sample.host_time_s - start_time_s:.3f}",
            "host_time_s": f"{sample.host_time_s:.6f}",
            "flags": f"0x{sample.flags:02x}",
            "speed_mps": f"{sample.speed_mps:.4f}",
            "cadence_spm": sample.cadence_spm,
            "stride_length_m": ""
            if sample.stride_length_m is None
            else f"{sample.stride_length_m:.3f}",
            "total_distance_m": ""
            if sample.total_distance_m is None
            else f"{sample.total_distance_m:.3f}",
            "running": int(sample.running),
            "raw_hex": sample.raw_hex,
        }
    )


def _matches_openstride(device, advertisement_data, name: str) -> bool:
    local_name = advertisement_data.local_name or device.name
    if local_name == name:
        return True
    service_uuids = {u.lower() for u in (advertisement_data.service_uuids or [])}
    return RSC_SERVICE_UUID in service_uuids


async def find_openstride(name: str, scan_timeout_s: float, address: str | None):
    try:
        from bleak import BleakScanner
    except ImportError as exc:
        raise SystemExit(
            "This script requires bleak. Run with: uv run --with bleak scripts/capture_rsc.py"
        ) from exc

    if address:
        print(f"Using BLE address {address!r} (skipping scan)...")
        return await BleakScanner.get_device_by_address(address, timeout=scan_timeout_s)

    print(
        f"Scanning for BLE device named {name!r} "
        f"or advertising RSC service {RSC_SERVICE_UUID}..."
    )
    device = await BleakScanner.find_device_by_filter(
        lambda d, adv: _matches_openstride(d, adv, name),
        timeout=scan_timeout_s,
    )
    if device is not None:
        return device

    print("Scan found no match. Nearby devices (name / address / services):")
    seen: set[str] = set()
    dump_timeout_s = min(5.0, scan_timeout_s)
    discovered = await BleakScanner.discover(timeout=dump_timeout_s, return_adv=True)
    for device, adv in discovered.values():
        if device.address in seen:
            continue
        seen.add(device.address)
        label = adv.local_name or device.name or "<no name>"
        services = ", ".join(adv.service_uuids) if adv.service_uuids else "<none>"
        print(f"  {label!r} @ {device.address}: {services}")

    raise SystemExit(
        f"Did not find BLE device named {name!r}. "
        "After a capture session the board must be advertising again (reflash or reset). "
        "If you know the address, pass --address."
    )


_PLOT_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>OpenStride live</title>
  <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/uplot@1.6.31/dist/uPlot.min.css" />
  <script src="https://cdn.jsdelivr.net/npm/uplot@1.6.31/dist/uPlot.iife.min.js"></script>
  <style>
    body { font-family: system-ui, sans-serif; margin: 1rem; background: #0f1115; color: #e8eaed; }
    h1 { font-size: 1.1rem; font-weight: 600; margin: 0 0 0.35rem; }
    #status { font-size: 0.85rem; color: #9aa0a6; margin-bottom: 0.75rem; }
    .metrics {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 0.6rem;
      margin-bottom: 0.9rem;
    }
    .metric {
      border: 1px solid #2a2f3a;
      border-radius: 8px;
      background: #151922;
      padding: 0.5rem 0.65rem;
    }
    .metric .label { display: block; font-size: 0.76rem; color: #9aa0a6; margin-bottom: 0.35rem; text-transform: uppercase; letter-spacing: 0.02em; }
    .metric .latest { display: block; font-size: 1.05rem; font-weight: 600; line-height: 1.2; }
    .metric .avg { display: block; margin-top: 0.2rem; font-size: 0.78rem; color: #a6c7ff; }
  </style>
</head>
<body>
  <h1>OpenStride RSC</h1>
  <div id="status">Waiting for samples...</div>
  <div class="metrics">
    <div class="metric">
      <span class="label">Speed</span>
      <span class="latest" id="speed-latest">--</span>
      <span class="avg" id="speed-avg">Window avg: --</span>
    </div>
    <div class="metric">
      <span class="label">Cadence</span>
      <span class="latest" id="cadence-latest">--</span>
      <span class="avg" id="cadence-avg">Window avg: --</span>
    </div>
    <div class="metric">
      <span class="label">Stride Length</span>
      <span class="latest" id="stride-latest">--</span>
      <span class="avg" id="stride-avg">Window avg: --</span>
    </div>
    <div class="metric">
      <span class="label">Total Distance</span>
      <span class="latest" id="distance-latest">--</span>
      <span class="avg" id="distance-avg">Window avg: --</span>
    </div>
  </div>
  <div id="chart"></div>
  <script>
    const pollMs = __POLL_MS__;
    const statsWindowS = __STATS_WINDOW_S__;
    const width = () => Math.max(640, window.innerWidth - 32);

    const chart = new uPlot({
      width: width(),
      height: 420,
      series: [
        {},
        {
          label: "speed m/s",
          scale: "speed",
          stroke: "#4fc3f7",
          width: 2.5,
          dash: [10, 6],
          points: { show: true, size: 3, stroke: "#4fc3f7", fill: "#4fc3f7" },
        },
        { label: "cadence spm", scale: "cadence", stroke: "#81c784", width: 2 },
        { label: "stride m", scale: "stride", stroke: "#ba68c8", width: 2 },
        { label: "distance m", scale: "distance", stroke: "rgba(255,183,77,0.7)", width: 1.5 },
      ],
      axes: [
        { stroke: "#9aa0a6", grid: { stroke: "#2a2f3a" } },
        { scale: "speed", stroke: "#4fc3f7", grid: { stroke: "#2a2f3a" } },
        { scale: "cadence", stroke: "#81c784", side: 1, grid: { show: false } },
        { scale: "stride", stroke: "#ba68c8", side: 1, grid: { show: false } },
        { scale: "distance", stroke: "#ffb74d", side: 1, grid: { show: false } },
      ],
      scales: {
        x: { time: false },
        speed: { auto: true },
        cadence: { auto: true },
        stride: { auto: true },
        distance: { auto: true },
      },
    }, [[0], [0], [0], [0], [0]], document.getElementById("chart"));

    window.addEventListener("resize", () => chart.setSize({ width: width(), height: 420 }));

    function finite(v) {
      return Number.isFinite(v);
    }

    function formatMetric(v, decimals, unit) {
      if (!finite(v)) return "--";
      return `${v.toFixed(decimals)} ${unit}`;
    }

    function windowAverage(elapsed, values, windowS) {
      if (!elapsed.length || !values.length) return null;
      const end = elapsed[elapsed.length - 1];
      const start = end - windowS;
      let sum = 0.0;
      let count = 0;
      for (let i = elapsed.length - 1; i >= 0; i -= 1) {
        if (elapsed[i] < start) break;
        const value = values[i];
        if (finite(value)) {
          sum += value;
          count += 1;
        }
      }
      return count > 0 ? (sum / count) : null;
    }

    function updateStat(prefix, values, elapsed, decimals, unit) {
      const latest = values.length ? values[values.length - 1] : null;
      const avg = windowAverage(elapsed, values, statsWindowS);
      document.getElementById(`${prefix}-latest`).textContent = formatMetric(latest, decimals, unit);
      document.getElementById(`${prefix}-avg`).textContent =
        `Window avg (${statsWindowS.toFixed(1)}s): ${formatMetric(avg, decimals, unit)}`;
    }

    let lastSeq = -1;
    async function poll() {
      try {
        const res = await fetch("/data.json", { cache: "no-store" });
        const d = await res.json();
        if (d.seq === lastSeq) return;
        lastSeq = d.seq;
        const n = d.elapsed.length;
        document.getElementById("status").textContent =
          n ? `Samples: ${n} | latest ${d.elapsed[n - 1].toFixed(2)} s` : "Connected - waiting for RSC notifications...";
        if (n === 0) return;
        chart.setData([d.elapsed, d.speed, d.cadence, d.stride, d.distance]);
        updateStat("speed", d.speed, d.elapsed, 2, "m/s");
        updateStat("cadence", d.cadence, d.elapsed, 0, "spm");
        updateStat("stride", d.stride, d.elapsed, 2, "m");
        updateStat("distance", d.distance, d.elapsed, 2, "m");
      } catch (e) {
        document.getElementById("status").textContent = "Poll error: " + e;
      }
    }
    setInterval(poll, pollMs);
    poll();
  </script>
</body>
</html>
"""


class BrowserLivePlot:
    """Local HTTP server + browser chart (only used with --plot)."""

    def __init__(
        self, max_points: int, port: int, poll_ms: int, stats_window_s: float
    ) -> None:
        self._max_points = max_points
        self._lock = threading.Lock()
        self._seq = 0
        self._json_cache: bytes = (
            b'{"seq":0,"elapsed":[],"speed":[],"cadence":[],"stride":[],"distance":[]}'
        )
        self._elapsed: list[float] = []
        self._speed: list[float] = []
        self._cadence: list[float] = []
        self._stride: list[float | None] = []
        self._distance: list[float | None] = []

        handler_cls = self._make_handler(poll_ms, stats_window_s)
        self._server = HTTPServer(("127.0.0.1", port), handler_cls)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

        url = f"http://127.0.0.1:{port}/"
        print(f"Live plot: {url}")
        webbrowser.open(url)

    def _make_handler(
        self, poll_ms: int, stats_window_s: float
    ) -> type[BaseHTTPRequestHandler]:
        plot = self
        html = _PLOT_HTML.replace("__POLL_MS__", str(poll_ms)).replace(
            "__STATS_WINDOW_S__", f"{stats_window_s:.2f}"
        )

        class Handler(BaseHTTPRequestHandler):
            plot_state: ClassVar[BrowserLivePlot] = plot
            index_html: ClassVar[str] = html

            def log_message(self, _format: str, *_args: object) -> None:
                return

            def do_GET(self) -> None:
                if self.path in ("/", "/index.html"):
                    body = self.index_html.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path == "/data.json":
                    body = self.plot_state.json_payload()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                self.send_error(404)

        return Handler

    def json_payload(self) -> bytes:
        with self._lock:
            return self._json_cache

    def _rebuild_cache_locked(self) -> None:
        self._json_cache = json.dumps(
            {
                "seq": self._seq,
                "elapsed": self._elapsed,
                "speed": self._speed,
                "cadence": self._cadence,
                "stride": self._stride,
                "distance": self._distance,
            },
            separators=(",", ":"),
        ).encode("utf-8")

    def add(self, elapsed_s: float, sample: RscSample) -> None:
        with self._lock:
            self._elapsed.append(round(elapsed_s, 3))
            self._speed.append(round(sample.speed_mps, 3))
            self._cadence.append(float(sample.cadence_spm))
            self._stride.append(
                None
                if sample.stride_length_m is None
                else round(sample.stride_length_m, 3)
            )
            last_distance = self._distance[-1] if self._distance else None
            distance = (
                sample.total_distance_m
                if sample.total_distance_m is not None
                else last_distance
            )
            self._distance.append(None if distance is None else round(distance, 2))
            if len(self._elapsed) > self._max_points:
                self._elapsed = self._elapsed[-self._max_points :]
                self._speed = self._speed[-self._max_points :]
                self._cadence = self._cadence[-self._max_points :]
                self._stride = self._stride[-self._max_points :]
                self._distance = self._distance[-self._max_points :]
            self._seq += 1
            self._rebuild_cache_locked()

    def close(self) -> None:
        self._server.shutdown()
        self._thread.join(timeout=2.0)


def format_sample_line(elapsed_s: float, sample: RscSample) -> str:
    return (
        f"{elapsed_s:8.3f}s speed={sample.speed_mps:5.2f}m/s "
        f"cadence={sample.cadence_spm:3d}spm "
        f"stride={sample.stride_length_m or 0.0:4.2f}m "
        f"distance={sample.total_distance_m or 0.0:6.1f}m "
        f"raw={sample.raw_hex}"
    )


async def capture(args: argparse.Namespace) -> None:
    try:
        from bleak import BleakClient
    except ImportError as exc:
        raise SystemExit(
            "This script requires bleak. Run with: uv run --with bleak scripts/capture_rsc.py"
        ) from exc

    plot = (
        BrowserLivePlot(
            args.plot_max_points,
            args.plot_port,
            args.plot_poll_ms,
            args.plot_stats_window_s,
        )
        if args.plot
        else None
    )

    csv_fp = None
    csv_writer = None
    try:
        device = await find_openstride(args.name, args.scan_timeout, args.address)
        if device is None:
            raise SystemExit(f"Could not resolve BLE address {args.address!r}")

        queue: asyncio.Queue[RscSample] = asyncio.Queue()
        loop = asyncio.get_running_loop()

        def on_notify(_sender, data: bytearray) -> None:
            try:
                sample = parse_rsc_measurement(bytes(data))
            except ValueError as exc:
                print(f"Decode error: {exc}")
                return
            loop.call_soon_threadsafe(queue.put_nowait, sample)

        if args.csv:
            csv_fp, csv_writer = open_csv(args.csv)
            print(f"Writing CSV to {args.csv}")

        start_time_s = time.monotonic()
        last_csv_flush_s = start_time_s
        stop_at_s = None if args.duration <= 0 else start_time_s + args.duration

        async with BleakClient(device) as client:
            print(f"Connected to {device.address}; subscribing to RSC Measurement...")
            await client.start_notify(RSC_MEASUREMENT_UUID, on_notify)

            while True:
                if stop_at_s is not None and time.monotonic() >= stop_at_s:
                    break

                timeout_s = 0.2
                if stop_at_s is not None:
                    timeout_s = max(0.05, min(timeout_s, stop_at_s - time.monotonic()))

                try:
                    sample = await asyncio.wait_for(queue.get(), timeout=timeout_s)
                except asyncio.TimeoutError:
                    continue

                elapsed_s = sample.host_time_s - start_time_s
                print(format_sample_line(elapsed_s, sample))

                if csv_writer is not None:
                    write_csv_row(csv_writer, start_time_s, sample)
                    now_s = time.monotonic()
                    if (now_s - last_csv_flush_s) >= args.csv_flush_interval_s:
                        csv_fp.flush()
                        last_csv_flush_s = now_s
                if plot is not None:
                    plot.add(elapsed_s, sample)

            await client.stop_notify(RSC_MEASUREMENT_UUID)
    finally:
        if csv_fp is not None:
            csv_fp.flush()
            csv_fp.close()
        if plot is not None:
            plot.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--name", default="OpenStride", help="BLE local name to connect to"
    )
    parser.add_argument(
        "--address",
        default=None,
        help="BLE address (skip name scan); useful when the device is bonded but name is missing",
    )
    parser.add_argument(
        "--scan-timeout", type=float, default=15.0, help="BLE scan timeout in seconds"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Capture duration; 0 means until Ctrl-C",
    )
    parser.add_argument(
        "--csv", type=Path, default=Path("captures/rsc.csv"), help="CSV output path"
    )
    parser.add_argument(
        "--no-csv",
        action="store_const",
        const=None,
        dest="csv",
        help="Disable CSV output",
    )
    parser.add_argument(
        "--csv-flush-interval-s",
        type=float,
        default=0.5,
        help="How often CSV writes are flushed to disk (lower is safer, higher is smoother)",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Open a live browser plot at http://127.0.0.1:<plot-port>/",
    )

    plot = parser.add_argument_group("live plot (requires --plot)")
    plot.add_argument(
        "--plot-port", type=int, default=8765, help="HTTP port for the live plot"
    )
    plot.add_argument(
        "--plot-poll-ms",
        type=int,
        default=250,
        help="Browser refresh interval in ms (match ~4 Hz BLE rate)",
    )
    plot.add_argument(
        "--plot-max-points",
        type=int,
        default=240,
        help="Max points in the live plot buffer (~60 s at 4 Hz)",
    )
    plot.add_argument(
        "--plot-stats-window-s",
        type=float,
        default=5.0,
        help="Rolling average window for the metric cards",
    )
    return parser.parse_args()


def main() -> None:
    try:
        asyncio.run(capture(parse_args()))
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
