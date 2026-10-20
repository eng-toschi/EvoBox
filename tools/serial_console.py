#!/usr/bin/env python3
"""Console serial com tempo limite (o macOS não tem `timeout`; `pio device
monitor` sem limite trava sessões automatizadas).

Exemplos:
  python3 tools/serial_console.py --seconds 15                 # reseta e captura o boot
  python3 tools/serial_console.py --boot-wait 5 -c "gnss info" -c "stats" --seconds 3
  python3 tools/serial_console.py --boot-wait 5 -c "rate 0" --wait 5 -c stats --seconds 2

Nesta placa (CP2102) abrir a porta reseta o ESP32 mesmo com DTR/RTS em
nível baixo; por isso use --boot-wait antes de mandar comandos.
"""
import argparse
import glob
import sys
import time

import serial


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbserial-*"))
    if not ports:
        sys.exit("no /dev/cu.usbserial-* port found")
    return ports[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=10.0, help="captura após o último comando")
    ap.add_argument("--no-reset", action="store_true", help="não pulsa EN ao abrir")
    ap.add_argument("--boot-wait", type=float, default=0.0, help="espera antes do 1º comando")
    ap.add_argument("-c", "--cmd", action="append", default=[], help="comando (repetível)")
    ap.add_argument("--wait", action="append", type=float, default=[],
                    help="espera (s) após o comando de mesmo índice")
    args = ap.parse_args()

    port = args.port or find_port()
    s = serial.Serial()
    s.port = port
    s.baudrate = args.baud
    s.timeout = 0.05
    s.dtr = False
    s.rts = False
    s.open()
    if not args.no_reset:
        # EN via RTS (CP2102 + transistores do devkit): pulso de reset.
        s.rts = True
        time.sleep(0.1)
        s.rts = False

    def pump(seconds):
        end = time.time() + seconds
        while time.time() < end:
            data = s.read(4096)
            if data:
                sys.stdout.write(data.decode("utf-8", errors="replace"))
                sys.stdout.flush()

    pump(args.boot_wait)
    for i, c in enumerate(args.cmd):
        s.write((c + "\n").encode())
        pump(args.wait[i] if i < len(args.wait) else 0.5)
    pump(args.seconds)
    s.close()


if __name__ == "__main__":
    main()
