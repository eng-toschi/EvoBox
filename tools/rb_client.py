#!/usr/bin/env python3
"""Cliente BLE de bancada para o EvoBox / RaceBox Mini S (SPEC §10.2).

Fala o protocolo RaceBox Rev 8 pelo serviço UART (bleak). Mede o que os
critérios de bancada pedem: primeiro frame após o subscribe (A2), taxa e
gaps pelo iTOW (A3), latência de comando (A4), download com contagem e
marcadores (A7/A12), erase com progresso e cancel (A9).

Exemplos:
  python3 tools/rb_client.py scan
  python3 tools/rb_client.py info
  python3 tools/rb_client.py live --seconds 60 --csv captures/live.csv
  python3 tools/rb_client.py cmd status
  python3 tools/rb_client.py cmd reccfg-read
  python3 tools/rb_client.py cmd preset-a          # Track Day do Racevo (R9)
  python3 tools/rb_client.py cmd raw B562FF220000
  python3 tools/rb_client.py download
  python3 tools/rb_client.py erase [--cancel-at 30]
  python3 tools/rb_client.py walkthrough           # 0x22 → 0x30 → 0x23 como o Racevo (R7/R12)
"""
import argparse
import asyncio
import csv
import os
import struct
import sys
import time
from typing import Callable, Dict, List, Optional, Tuple

from bleak import BleakClient, BleakScanner

UART_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
UART_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
DIS = {
    "model": "00002a24-0000-1000-8000-00805f9b34fb",
    "serial": "00002a25-0000-1000-8000-00805f9b34fb",
    "firmware": "00002a26-0000-1000-8000-00805f9b34fb",
    "hardware": "00002a27-0000-1000-8000-00805f9b34fb",
    "manufacturer": "00002a29-0000-1000-8000-00805f9b34fb",
}
NAME_PREFIXES = ("RaceBox Mini S ", "RaceBox Mini ", "RaceBox Micro ", "GoAwake Logger ")
MS_PER_WEEK = 604800000

LIVE_FMT = "<IHBBBBBBIiBBBBiiiiIIiiIIHBBhhhhhh"
LIVE_FIELDS = [
    "itow", "year", "month", "day", "hour", "minute", "second", "validity", "time_acc_ns",
    "nano", "fix_status", "fix_flags", "datetime_flags", "num_sv", "lon", "lat", "wgs_alt_mm",
    "msl_alt_mm", "h_acc_mm", "v_acc_mm", "speed_mmps", "heading", "speed_acc_mmps",
    "heading_acc", "pdop", "latlon_flags", "battery", "gx", "gy", "gz", "rx", "ry", "rz",
]
assert struct.calcsize(LIVE_FMT) == 80


# --- framing ------------------------------------------------------------------

def checksum(body: bytes) -> Tuple[int, int]:
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def frame(cls: int, mid: int, payload: bytes = b"") -> bytes:
    body = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    a, b = checksum(body)
    return b"\xb5\x62" + body + bytes([a, b])


class Assembler:
    """Mesmo contrato do FrameAssembler (SPEC §4.4 / Racevo 04 §3.3)."""

    def __init__(self, max_payload: int = 504):
        self.buf = bytearray()
        self.max_payload = max_payload
        self.bad = 0
        self.noise = 0

    def feed(self, data: bytes) -> List[Tuple[int, int, bytes]]:
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"\xb5\x62")
            if i < 0:
                keep = 1 if self.buf[-1:] == b"\xb5" else 0
                self.noise += len(self.buf) - keep
                del self.buf[: len(self.buf) - keep]
                return out
            if i > 0:
                self.noise += i
                del self.buf[:i]
            if len(self.buf) < 6:
                return out
            plen = struct.unpack_from("<H", self.buf, 4)[0]
            if plen > self.max_payload:
                self.bad += 1
                del self.buf[:1]
                continue
            if len(self.buf) < plen + 8:
                return out
            a, b = checksum(bytes(self.buf[2 : 6 + plen]))
            if a != self.buf[6 + plen] or b != self.buf[7 + plen]:
                self.bad += 1
                del self.buf[:1]
                continue
            out.append((self.buf[2], self.buf[3], bytes(self.buf[6 : 6 + plen])))
            del self.buf[: plen + 8]


def decode_live(p: bytes) -> Dict[str, int]:
    return dict(zip(LIVE_FIELDS, struct.unpack(LIVE_FMT, p)))


def rec_config_bytes(enable, rate, flags, speed, stat_s, nofix_s, shut_s) -> bytes:
    return struct.pack("<BBBBHHHH", enable, rate, flags, 0, speed, stat_s, nofix_s, shut_s)


def describe(cls: int, mid: int, p: bytes) -> str:
    if cls != 0xFF:
        return "class 0x%02X id 0x%02X len %d" % (cls, mid, len(p))
    if mid in (0x02, 0x03) and len(p) == 2:
        return "%s of 0x%02X 0x%02X" % ("ACK" if mid == 0x02 else "NACK", p[0], p[1])
    if mid == 0x22 and len(p) == 12:
        rec, pct, sec, _, stored, cap = struct.unpack("<BBBBII", p)
        return "STATUS recording=%d memory=%d%% security=0x%02X stored=%d capacity=%d" % (rec, pct, sec, stored, cap)
    if mid == 0x25 and len(p) == 12:
        e, r, f, _, sp, st, nf, sh = struct.unpack("<BBBBHHHH", p)
        return "RECCFG enable=%d rate=%d flags=0x%02X speed=%dmm/s stat=%ds nofix=%ds shutdown=%ds" % (e, r, f, sp, st, nf, sh)
    if mid == 0x26 and len(p) == 12:
        s, _, r, f, sp, st, nf, sh = struct.unpack("<BBBBHHHH", p)
        return "STATE state=%d rate=%d flags=0x%02X speed=%d stat=%d nofix=%d shutdown=%d" % (s, r, f, sp, st, nf, sh)
    if mid == 0x27 and len(p) == 3:
        return "GNSSCFG dynModel=%d speed3d=%d minHAcc=%.1fm" % (p[0], p[1], p[2] / 10.0)
    if mid == 0x23 and len(p) == 4:
        return "DOWNLOAD expected=%d" % struct.unpack("<I", p)[0]
    if mid == 0x24 and len(p) == 1:
        return "ERASE progress=%d%%" % p[0]
    if mid in (0x01, 0x21) and len(p) == 80:
        d = decode_live(p)
        return "%s iTOW=%d fix=%d ok=%d sv=%d spd=%d g=(%d,%d,%d)" % (
            "LIVE" if mid == 0x01 else "HIST", d["itow"], d["fix_status"], d["fix_flags"] & 1,
            d["num_sv"], d["speed_mmps"], d["gx"], d["gy"], d["gz"])
    return "0xFF 0x%02X len %d" % (mid, len(p))


# --- conexão ------------------------------------------------------------------

async def find_device(name: Optional[str], timeout: float):
    """Acha UM device. Sem --name, recusa escolher se houver mais de um
    candidato (um RaceBox real por perto também anuncia "RaceBox Mini S ...")."""
    def is_candidate(dev, adv):
        n = adv.local_name or dev.name or ""
        if name:
            return name in n
        if any(n.startswith(p) for p in NAME_PREFIXES):
            return True
        return UART_SERVICE in [u.lower() for u in (adv.service_uuids or [])]

    if name:
        dev = await BleakScanner.find_device_by_filter(is_candidate, timeout=timeout)
        if dev is None:
            sys.exit("device not found (name filter=%r)" % name)
        return dev
    found = await BleakScanner.discover(timeout=min(timeout, 5.0), return_adv=True)
    cands = [(d, a) for d, a in found.values() if is_candidate(d, a)]
    if not cands:
        sys.exit("no RaceBox-protocol device found")
    if len(cands) > 1:
        names = ", ".join(repr(a.local_name or d.name) for d, a in cands)
        sys.exit("several candidates (%s): pass --name <serial> to pick one" % names)
    return cands[0][0]


class Session:
    def __init__(self, client: BleakClient, verbose: bool):
        self.client = client
        self.asm = Assembler()
        self.verbose = verbose
        self.handlers: List[Callable[[int, int, bytes, float], None]] = []
        self.subscribe_t: Optional[float] = None
        self.first_frame_t: Optional[float] = None
        self.notifies = 0
        self.notify_bytes = 0
        self.max_notify = 0

    def _on_notify(self, _char, data: bytearray):
        t = time.monotonic()
        self.notifies += 1
        self.notify_bytes += len(data)
        self.max_notify = max(self.max_notify, len(data))
        for cls, mid, p in self.asm.feed(bytes(data)):
            if self.first_frame_t is None and cls == 0xFF and mid == 0x01:
                self.first_frame_t = t
            if self.verbose:
                print("  <- " + describe(cls, mid, p))
            for h in list(self.handlers):
                h(cls, mid, p, t)

    async def subscribe(self):
        self.subscribe_t = time.monotonic()
        await self.client.start_notify(UART_TX, self._on_notify)

    async def write(self, data: bytes):
        try:
            await self.client.write_gatt_char(UART_RX, data, response=True)
        except Exception:
            await self.client.write_gatt_char(UART_RX, data, response=False)  # R16

    async def command(self, cls: int, mid: int, payload: bytes, expect: Callable[[int, int, bytes], bool],
                      timeout: float = 8.0) -> Tuple[Optional[Tuple[int, int, bytes]], float]:
        fut = asyncio.get_running_loop().create_future()

        def h(c, m, p, t):
            if not fut.done() and expect(c, m, p):
                fut.set_result(((c, m, p), t))

        self.handlers.append(h)
        t0 = time.monotonic()
        await self.write(frame(cls, mid, payload))
        try:
            res, t = await asyncio.wait_for(fut, timeout)
            return res, (t - t0) * 1000.0
        except asyncio.TimeoutError:
            return None, (time.monotonic() - t0) * 1000.0
        finally:
            self.handlers.remove(h)


async def open_session(args) -> Tuple[BleakClient, Session]:
    dev = await find_device(args.name, args.scan_timeout)
    client = BleakClient(dev)
    await client.connect()
    s = Session(client, getattr(args, "verbose", False))
    return client, s


async def read_dis(client: BleakClient) -> Dict[str, str]:
    out = {}
    for k, u in DIS.items():
        try:
            out[k] = bytes(await client.read_gatt_char(u)).decode("utf-8", errors="replace")
        except Exception as e:  # noqa: BLE001
            out[k] = "<%s>" % e
    return out


# --- subcomandos ----------------------------------------------------------------

async def cmd_scan(args):
    found = await BleakScanner.discover(timeout=args.scan_timeout, return_adv=True)
    for _addr, (dev, adv) in found.items():
        n = adv.local_name or dev.name or ""
        uuids = [u.lower() for u in (adv.service_uuids or [])]
        if any(n.startswith(p) for p in NAME_PREFIXES) or UART_SERVICE in uuids:
            print("%s  rssi=%d  name=%r  uart_uuid=%s" % (dev.address, adv.rssi, n, UART_SERVICE in uuids))


async def cmd_info(args):
    client, s = await open_session(args)
    try:
        dis = await read_dis(client)
        for k, v in dis.items():
            print("%-12s %r" % (k, v))
        print("%-12s %d" % ("mtu", client.mtu_size))
        svcs = [str(x.uuid).lower() for x in client.services]
        print("%-12s %s" % ("uart", UART_SERVICE in svcs))
        ok = dis["model"] == "RaceBox Mini S"
        print("racevo model classification: %s" % ("miniS" if ok else "other"))
    finally:
        await client.disconnect()


class LiveStats:
    def __init__(self):
        self.frames = 0
        self.last_itow: Optional[int] = None
        self.dts: Dict[int, int] = {}
        self.backwards = 0
        self.rows: List[Dict[str, int]] = []
        self.first_itow: Optional[int] = None

    def add(self, d: Dict[str, int], keep_rows: bool):
        self.frames += 1
        it = d["itow"]
        if self.first_itow is None:
            self.first_itow = it
        if self.last_itow is not None:
            dt = (it - self.last_itow) % MS_PER_WEEK
            if dt == 0 or dt > MS_PER_WEEK // 2:
                self.backwards += 1
            self.dts[dt] = self.dts.get(dt, 0) + 1
        self.last_itow = it
        if keep_rows:
            self.rows.append(d)

    def report(self, period_ms: Optional[int]):
        n = sum(self.dts.values())
        print("frames=%d  dITOW histogram (ms:count): %s" % (
            self.frames, ", ".join("%d:%d" % kv for kv in sorted(self.dts.items()))))
        if n:
            mode = max(self.dts.items(), key=lambda kv: kv[1])[0]
            p = period_ms or mode
            gaps = sum(c for dt, c in self.dts.items() if dt * 2 > p * 3)
            print("period=%d ms  on-period=%d/%d  gaps(>1.5x)=%d  non-monotonic=%d  rate=%.2f Hz" % (
                p, self.dts.get(p, 0), n, gaps, self.backwards, 1000.0 / p))


async def cmd_live(args):
    client, s = await open_session(args)
    stats = LiveStats()
    try:
        print("connected, mtu=%d" % client.mtu_size)

        def on(cls, mid, p, _t):
            if cls == 0xFF and mid == 0x01 and len(p) == 80:
                stats.add(decode_live(p), bool(args.csv))
                if args.print_every and stats.frames % args.print_every == 0:
                    print("  " + describe(cls, mid, p))

        s.handlers.append(on)
        await s.subscribe()
        t_end = time.monotonic() + args.seconds
        while time.monotonic() < t_end and client.is_connected:
            await asyncio.sleep(0.2)
        if not client.is_connected:
            print("link dropped after %.1f s" % (args.seconds - (t_end - time.monotonic())))
        else:
            await client.stop_notify(UART_TX)
    finally:
        try:
            await client.disconnect()
        except Exception:  # noqa: BLE001 — o link pode já ter caído
            pass
    if s.first_frame_t is not None:
        print("first live frame %.0f ms after subscribe (A2: < 1000 ms)" % ((s.first_frame_t - s.subscribe_t) * 1000))
    else:
        print("NO live frame received")
    print("notifies=%d bytes=%d max_notify=%d assembler_bad=%d noise=%d" % (
        s.notifies, s.notify_bytes, s.max_notify, s.asm.bad, s.asm.noise))
    stats.report(args.period)
    if args.csv:
        os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=LIVE_FIELDS)
            w.writeheader()
            w.writerows(stats.rows)
        print("csv: %s (%d rows)" % (args.csv, len(stats.rows)))


NAMED = {
    "status": (0x22, b""),
    "reccfg-read": (0x25, b""),
    "gnsscfg-read": (0x27, b""),
    "unlock": (0x30, struct.pack("<I", 123456)),
    "preset-a": (0x25, rec_config_bytes(1, 0, 0x1F, 1389, 30, 30, 300)),
    "preset-b": (0x25, rec_config_bytes(1, 0, 0x1D, 0, 0, 30, 600)),
    "rec-stop": (0x25, rec_config_bytes(0, 0, 0, 0, 0, 0, 0)),
    "rec-10hz": (0x25, rec_config_bytes(1, 1, 0x00, 0, 0, 0, 0)),
}


async def cmd_cmd(args):
    client, s = await open_session(args)
    try:
        await s.subscribe()
        await asyncio.sleep(0.3)
        for name in args.what:
            if name == "raw":
                continue
            if name in NAMED:
                mid, payload = NAMED[name]
            elif name.lower().startswith("b562"):
                raw = bytes.fromhex(name)
                asm = Assembler()
                frames = asm.feed(raw)
                if len(frames) != 1 or len(raw) != len(frames[0][2]) + 8:
                    sys.exit("raw %r is not exactly one valid UBX frame (checksum included)" % name)
                mid, payload = frames[0][1], frames[0][2]
            else:
                sys.exit("unknown command %r (known: %s)" % (name, ", ".join(NAMED)))

            def expect(c, m, p, mid=mid):
                return c == 0xFF and (m == mid or (m in (0x02, 0x03) and len(p) == 2 and p[1] == mid))

            res, ms = await s.command(0xFF, mid, payload, expect)
            if res is None:
                print("%-12s -> TIMEOUT after %.0f ms" % (name, ms))
            else:
                print("%-12s -> %s  (%.0f ms)" % (name, describe(*res), ms))
    finally:
        await client.disconnect()


async def cmd_download(args):
    client, s = await open_session(args)
    hist = 0
    rows: List[Dict[str, int]] = []
    states: List[Tuple[int, int]] = []
    done = asyncio.get_running_loop().create_future()
    expected = [None]
    t_first = [None]
    try:
        await s.subscribe()
        await asyncio.sleep(0.3)

        def on(cls, mid, p, t):
            nonlocal hist
            if cls != 0xFF:
                return
            if mid == 0x23 and len(p) == 4:
                expected[0] = struct.unpack("<I", p)[0]
                print("download: expected <= %d records" % expected[0])
            elif mid == 0x21 and len(p) == 80:
                hist += 1
                if args.csv:
                    rows.append(decode_live(p))
                if t_first[0] is None:
                    t_first[0] = t
                if args.cancel_after and hist == args.cancel_after:
                    asyncio.ensure_future(s.write(frame(0xFF, 0x23, b"\x00")))
            elif mid == 0x26 and len(p) == 12:
                states.append((hist, p[0]))
            elif mid in (0x02, 0x03) and len(p) == 2 and p[1] == 0x23 and not done.done():
                done.set_result((mid == 0x02, t))

        s.handlers.append(on)
        t0 = time.monotonic()
        await s.write(frame(0xFF, 0x23))
        ok, t_end = await asyncio.wait_for(done, args.timeout)
    finally:
        await client.disconnect()
    secs = t_end - t0
    print("download %s: %d records (0x21), %d state markers, %.1f s, %.1f KB/s" % (
        "ACK" if ok else "NACK", hist, len(states), secs, (s.notify_bytes / 1024.0) / max(secs, 1e-6)))
    for at, st in states:
        print("  0x26 state=%d after %d records" % (st, at))
    runs = sum(1 for _, st in states if st == 1) or (1 if hist else 0)
    print("runs (Racevo splitDownloadIntoRuns): %d" % runs)
    if args.csv:
        os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=LIVE_FIELDS)
            w.writeheader()
            w.writerows(rows)
        print("csv: %s (%d rows)" % (args.csv, len(rows)))


async def cmd_erase(args):
    client, s = await open_session(args)
    progress: List[int] = []
    done = asyncio.get_running_loop().create_future()
    try:
        await s.subscribe()
        await asyncio.sleep(0.3)

        def on(cls, mid, p, t):
            if cls != 0xFF:
                return
            if mid == 0x24 and len(p) == 1:
                progress.append(p[0])
                print("  erase %d%%" % p[0])
                if args.cancel_at is not None and p[0] >= args.cancel_at and len(progress) == 1 + progress.index(p[0]):
                    asyncio.ensure_future(s.write(frame(0xFF, 0x24, b"\x00")))
            elif mid in (0x02, 0x03) and len(p) == 2 and p[1] == 0x24 and not done.done():
                done.set_result(mid == 0x02)

        s.handlers.append(on)
        t0 = time.monotonic()
        await s.write(frame(0xFF, 0x24))
        ok = await asyncio.wait_for(done, args.timeout)
    finally:
        await client.disconnect()
    print("erase %s after %.1f s, progress steps=%d last=%s" % (
        "ACK" if ok else "NACK", time.monotonic() - t0, len(progress), progress[-1] if progress else None))


async def cmd_walkthrough(args):
    """Sequência do walkthrough standalone do Racevo (R7/R12)."""
    client, s = await open_session(args)
    try:
        dis = await read_dis(client)
        print("DIS model=%r serial=%r fw=%r" % (dis["model"], dis["serial"], dis["firmware"]))
        await s.subscribe()
        await asyncio.sleep(0.5)
        res, ms = await s.command(0xFF, 0x22, b"", lambda c, m, p: c == 0xFF and m == 0x22)
        print("0x22 -> %s (%.0f ms)" % (describe(*res) if res else "TIMEOUT", ms))
        if res and res[2][2] & 0x01 and not res[2][2] & 0x02:
            r2, ms = await s.command(0xFF, 0x30, struct.pack("<I", 0),
                                     lambda c, m, p: c == 0xFF and m in (2, 3) and p[1:2] == b"\x30")
            print("0x30 -> %s (%.0f ms)" % (describe(*r2) if r2 else "TIMEOUT", ms))
    finally:
        await client.disconnect()
    await cmd_download(args)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default=None, help="substring do nome anunciado")
    ap.add_argument("--scan-timeout", type=float, default=10.0)
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    sub.add_parser("info")
    p = sub.add_parser("live")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--csv", default=None)
    p.add_argument("--period", type=int, default=None, help="período esperado (ms)")
    p.add_argument("--print-every", type=int, default=0)
    p = sub.add_parser("cmd")
    p.add_argument("what", nargs="+")
    p = sub.add_parser("download")
    p.add_argument("--timeout", type=float, default=600.0)
    p.add_argument("--cancel-after", type=int, default=0)
    p.add_argument("--csv", default=None)
    p = sub.add_parser("erase")
    p.add_argument("--timeout", type=float, default=600.0)
    p.add_argument("--cancel-at", type=int, default=None)
    p = sub.add_parser("walkthrough")
    p.add_argument("--timeout", type=float, default=600.0)
    p.add_argument("--cancel-after", type=int, default=0)
    p.add_argument("--csv", default=None)
    args = ap.parse_args()
    fn = {"scan": cmd_scan, "info": cmd_info, "live": cmd_live, "cmd": cmd_cmd,
          "download": cmd_download, "erase": cmd_erase, "walkthrough": cmd_walkthrough}[args.cmd]
    asyncio.run(fn(args))


if __name__ == "__main__":
    main()
