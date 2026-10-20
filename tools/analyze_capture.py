#!/usr/bin/env python3
"""Análise de uma captura CSV do rb_client.py (live).

  python3 tools/analyze_capture.py captures/x.csv imu      # A6: médias/desvios de g e rotação
  python3 tools/analyze_capture.py captures/x.csv itow     # Δ iTOW, gaps, monotonicidade
  python3 tools/analyze_capture.py captures/x.csv laps     # voltas por cruzamento de linha (modo sim)
"""
import csv
import math
import statistics
import sys

MS_PER_WEEK = 604800000


def load(path):
    with open(path) as f:
        return [{k: int(v) for k, v in r.items()} for r in csv.DictReader(f)]


def imu(rows):
    print("frames=%d" % len(rows))
    for k, unit, scale in (("gx", "mg", 1), ("gy", "mg", 1), ("gz", "mg", 1),
                           ("rx", "dps", 0.01), ("ry", "dps", 0.01), ("rz", "dps", 0.01)):
        v = [r[k] * scale for r in rows]
        print("%s: mean=%9.3f %s  std=%7.3f  min=%9.3f  max=%9.3f" % (
            k, statistics.mean(v), unit, statistics.pstdev(v), min(v), max(v)))


def itow(rows):
    dts = {}
    back = 0
    for a, b in zip(rows, rows[1:]):
        dt = (b["itow"] - a["itow"]) % MS_PER_WEEK
        if dt == 0 or dt > MS_PER_WEEK // 2:
            back += 1
        dts[dt] = dts.get(dt, 0) + 1
    mode = max(dts.items(), key=lambda kv: kv[1])[0] if dts else 0
    gaps = sum(c for dt, c in dts.items() if dt * 2 > mode * 3)
    span = (rows[-1]["itow"] - rows[0]["itow"]) % MS_PER_WEEK / 1000.0 if rows else 0
    print("frames=%d span=%.1f s period=%d ms on-period=%d gaps=%d non-monotonic=%d" % (
        len(rows), span, mode, dts.get(mode, 0), gaps, back))
    print("histogram:", ", ".join("%d:%d" % kv for kv in sorted(dts.items())))


def laps(rows):
    if len(rows) < 3:
        sys.exit("too few rows")
    lat = [r["lat"] / 1e7 for r in rows]
    lon = [r["lon"] / 1e7 for r in rows]
    clat, clon = statistics.mean(lat), statistics.mean(lon)
    k = math.cos(math.radians(clat))
    ang = [math.atan2(math.radians(la - clat), math.radians(lo - clon) * k) for la, lo in zip(lat, lon)]
    # desembrulha e marca voltas quando o ângulo acumulado cruza múltiplos de 2π
    acc = [0.0]
    for a, b in zip(ang, ang[1:]):
        d = b - a
        if d > math.pi:
            d -= 2 * math.pi
        elif d < -math.pi:
            d += 2 * math.pi
        acc.append(acc[-1] + d)
    t = [((r["itow"] - rows[0]["itow"]) % MS_PER_WEEK) / 1000.0 for r in rows]
    crossings = []
    for i in range(1, len(acc)):
        n0, n1 = math.floor(acc[i - 1] / (2 * math.pi)), math.floor(acc[i] / (2 * math.pi))
        if n0 != n1:
            target = max(n0, n1) * 2 * math.pi
            f = (target - acc[i - 1]) / (acc[i] - acc[i - 1])
            crossings.append(t[i - 1] + f * (t[i] - t[i - 1]))
    lt = [b - a for a, b in zip(crossings, crossings[1:])]
    radius = statistics.mean(math.hypot(math.radians(la - clat) * 6371000, math.radians(lo - clon) * 6371000 * k)
                             for la, lo in zip(lat, lon))
    print("frames=%d radius=%.2f m crossings=%d" % (len(rows), radius, len(crossings)))
    for i, x in enumerate(lt, 1):
        print("lap %d: %.3f s" % (i, x))
    if lt:
        print("laps=%d mean=%.3f s spread=%.3f s" % (len(lt), statistics.mean(lt), max(lt) - min(lt)))


def main():
    if len(sys.argv) != 3 or sys.argv[2] not in ("imu", "itow", "laps"):
        sys.exit(__doc__)
    rows = load(sys.argv[1])
    {"imu": imu, "itow": itow, "laps": laps}[sys.argv[2]](rows)


if __name__ == "__main__":
    main()
