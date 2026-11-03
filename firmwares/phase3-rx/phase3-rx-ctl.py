#!/usr/bin/env python3
"""
phase3-rx-ctl.py -- X6100 BASE (phase3-rx) control + RF-path explorer REPL.

Runs on the R16 CPU (Linux) and drives the BASE over I2C (STM32 I2C3 slave at
7-bit address 0x72 on /dev/i2c-0). Each command is one I2C write:
[opcode, b0, b1, b2, b3] with the 4 payload bytes little-endian. The BASE
echoes every command to its USART1 diagnostic line.

Normal control (opcodes 0x01..):
    TUNE   <hz>      T 1410000     tune the receiver (Hz); also sets RF switch + BAND_V
    VOLUME <float>   V 8.0         audio gain / post-AGC trim (0 = silent)
    AGC    <t a d>   A -12 70 250  feed-forward AGC: target dBFS, attack/decay/[hang] ms
                     A 0 0 0       disable AGC (back to fixed VOLUME)
    LOOFF  <hz>      O -100        LO offset Hz, signed (ftuned = station + offset; SSB wants 0)
    MODE   <m>       M USB         demod mode AM/LSB/USB (or 1/2/3); auto-sets LO offset
    LINMULT <float>  L 0.05        HPL/MAIN (R16-capture) level, independent of VOLUME (0=silent)

RF-path explorer / debug (opcodes 0x81.., bypass the freq->index logic):
    RFPRIM <0..9>    P 0           force primary RF band index (PD9/10/11/13)
    RFSEC  <0..6>    S 0           force secondary sub-band index (SPI6 image bit)
    BANDV  <code>    B 0x11D       force BAND_V DAC2 code (12-bit; accepts 0x..)
    RFIMG  <image>   I 0x2000      write a raw 24-bit SPI6 image (probe the 74HC595)
    DUMP             D             dump CS4272 + VersaClock + RF state to USART1

    HELP / H / ?,  QUIT / Q

Up-arrow recalls previous commands (readline). Pure standard library.

Usage:
    ./phase3-rx-ctl.py                 # interactive REPL, /dev/i2c-0 @ 0x72
    ./phase3-rx-ctl.py --bus 0 --addr 0x72
    echo "P 0" | ./phase3-rx-ctl.py    # one-shot
"""
import argparse
import fcntl
import os
import struct
import sys

try:
    import readline  # noqa: F401  -- enables history + line editing in input()
except ImportError:
    readline = None

I2C_SLAVE       = 0x0703
I2C_SLAVE_FORCE = 0x0706

# command table: name -> (aliases, opcode, argtype, help)
#   argtype: 'u32' (base-0 int), 'f32' (float), None (no arg)
CMDS = {
    "TUNE":   (("T",),      0x01, "u32", "tune Hz, e.g. TUNE 1410000"),
    "VOLUME": (("V", "VOL"),0x02, "f32", "audio gain, e.g. VOLUME 8.0 (0=silent)"),
    "AGC":    (("A",),      0x03, "agc", "AGC <tgt_dBFS> <atk_ms> <dcy_ms> [hang_ms], "
                                         "e.g. AGC -12 70 250; AGC 0 0 0 = off (fixed VOLUME)"),
    "LOOFF":  (("O",),      0x04, "i32", "LO offset Hz, signed (ftuned = station + offset; "
                                         "default -100; AM offset-invariant, SSB wants 0)"),
    "MODE":   (("M",),      0x05, "mode", "demod mode: MODE AM|LSB|USB (or 1|2|3); "
                                          "auto-sets LO offset (AM -100, SSB 0)"),
    "LINMULT":(("L",),      0x06, "f32", "HPL/MAIN (R16-capture) level, e.g. LINMULT 0.05 "
                                         "(0=silent; independent of VOLUME/speaker)"),
    "RFPRIM": (("P",),      0x81, "u32", "force primary RF index 0..9"),
    "RFSEC":  (("S",),      0x82, "u32", "force secondary sub-band index 0..6"),
    "BANDV":  (("B",),      0x83, "u32", "force BAND_V DAC2 code, e.g. BANDV 0x11D"),
    "RFIMG":  (("I",),      0x84, "u32", "write raw 24-bit SPI6 image, e.g. RFIMG 0x2000"),
    "SC6":    (("C",),      0x85, "u32", "shift 6-bit word 0..0x3F through PC5/PB0/PB1 RF chain, e.g. SC6 0x25"),
    "SC8PG6": (("G",),      0x86, "u32", "shift 8-bit word through PC5/PB0/PG6 RF chain, e.g. SC8PG6 0x1A"),
    "GPIO":   (("X",),      0x87, "gpio", "drive a pin: GPIO <PortPin> <mode>, e.g. GPIO G8 4. "
                                          "mode 0=in 1=in+pd 2=in+pu 3=out-lo 4=out-hi 5=OD-lo 6=OD-hi"),
    "CODEC":  (("Z",),      0x88, "codec", "write TLV320 reg: CODEC <page> <reg> <val> (all base-0), "
                                           "e.g. CODEC 1 0x11 0x00 (unmute HPR 0 dB), CODEC 1 0x11 0x09 (+9 dB)"),
    "DUMP":   (("D",),      0x8F, None,  "dump CS4272/VersaClock/RF state to USART1"),
}
ALIAS = {a: name for name, (aliases, *_) in CMDS.items() for a in aliases}
ALIAS.update({name: name for name in CMDS})

# Minimum argument count per argtype; a command invoked with fewer args is a
# QUERY (prints the last value this script set). AGC's 4th arg (hang) is optional.
REQ = {"u32": 1, "i32": 1, "f32": 1, "mode": 1, "gpio": 2, "agc": 3, "codec": 3}

# Client-side mirror of what we've set, shown on a no-argument query. There is NO
# firmware read-back, so this reflects only commands sent since the script
# started, seeded with the firmware's boot defaults (RX_TUNED_HZ=1410000 Hz,
# VOLUME=0 silent, AGC ON -12/70/250). Stale after a reflash/reboot or if another
# tool writes 0x72 -- restart the script to reset.
STATE = {
    "TUNE":   "1410000 (0x158418)",
    "VOLUME": "0",
    "AGC":    "ON target=-12.00 dBFS atk=70ms dcy=250ms hang=0ms",
    "LOOFF":  "-100",
    "MODE":   "AM",
    "LINMULT": "0",
}


class Base:
    def __init__(self, bus, addr):
        self.fd = os.open("/dev/i2c-%d" % bus, os.O_RDWR)
        try:
            fcntl.ioctl(self.fd, I2C_SLAVE, addr)
        except OSError:
            fcntl.ioctl(self.fd, I2C_SLAVE_FORCE, addr)

    def send(self, opcode, payload=b"\x00\x00\x00\x00"):
        frame = bytes([opcode]) + payload
        n = os.write(self.fd, frame)
        if n != len(frame):
            raise IOError("short write %d/%d" % (n, len(frame)))

    def close(self):
        os.close(self.fd)


def handle(base, line):
    parts = line.split()
    if not parts:
        return True
    key = parts[0].upper()
    if key in ("QUIT", "EXIT", "Q"):
        return False
    if key in ("HELP", "H", "?"):
        for name, (aliases, op, argt, doc) in CMDS.items():
            print("  %-7s %-5s %s" % (name, "/".join(aliases), doc))
        print("  QUIT / Q")
        print("  (a command with no argument prints its last-set value, "
              "e.g. 'V', 'T', 'A')")
        return True
    name = ALIAS.get(key)
    if name is None:
        print("  ? unknown command %r (try: help)" % parts[0])
        return True
    aliases, opcode, argtype, _ = CMDS[name]
    # No-argument query: show the value this script last set (client-side mirror,
    # no firmware read-back). DUMP (argtype None) has no value and always sends.
    if argtype is not None and (len(parts) - 1) < REQ[argtype]:
        print("  %s = %s" % (name, STATE.get(name, "unknown (not set this session)")))
        return True
    try:
        if argtype is None:
            base.send(opcode)
            print("  -> %s sent" % name)
        elif argtype == "u32":
            val = int(parts[1], 0)            # base-0: accepts 1410000 or 0x2000
            base.send(opcode, struct.pack("<I", val & 0xFFFFFFFF))
            STATE[name] = "%d (0x%X)" % (val, val)
            print("  -> %s %s sent" % (name, STATE[name]))
        elif argtype == "i32":
            val = int(parts[1], 0)            # signed, base-0: accepts -100 or 0
            base.send(opcode, struct.pack("<i", val))
            STATE[name] = "%d" % val
            print("  -> %s %s Hz sent" % (name, STATE[name]))
        elif argtype == "mode":
            modes = {"AM": 1, "LSB": 2, "USB": 3, "1": 1, "2": 2, "3": 3}
            key = parts[1].upper()
            if key not in modes:
                raise ValueError("mode AM/LSB/USB or 1/2/3")
            m = modes[key]
            base.send(opcode, struct.pack("<I", m))
            STATE[name] = {1: "AM", 2: "LSB", 3: "USB"}[m]
            print("  -> MODE %s sent" % STATE[name])
        elif argtype == "f32":
            val = float(parts[1])
            base.send(opcode, struct.pack("<f", val))
            STATE[name] = "%g" % val
            print("  -> %s %s sent" % (name, STATE[name]))
        elif argtype == "gpio":
            pp = parts[1].upper()                 # e.g. "G8", "E11"
            port = ord(pp[0]) - ord("A")
            pin = int(pp[1:], 0)
            mode = int(parts[2], 0)
            if not (0 <= port <= 6 and 0 <= pin <= 15 and 0 <= mode <= 6):
                raise ValueError("port A-G, pin 0-15, mode 0-6")
            val = (port & 0xF) | ((pin & 0xF) << 4) | ((mode & 0xF) << 8)
            base.send(opcode, struct.pack("<I", val))
            STATE[name] = "P%s%d mode=%d (arg=0x%03X)" % (chr(ord("A") + port), pin, mode, val)
            print("  -> GPIO %s sent" % STATE[name])
        elif argtype == "agc":
            tgt = float(parts[1])
            atk = int(parts[2], 0)
            dcy = int(parts[3], 0)
            hang = int(parts[4], 0) if len(parts) > 4 else 0
            cdb = int(round(tgt * 100.0))
            if not (-32768 <= cdb <= 32767 and 0 <= atk <= 0xFFFF
                    and 0 <= dcy <= 0xFFFF and 0 <= hang <= 0xFFFF):
                raise ValueError("target +-327 dBFS, atk/dcy/hang 0..65535 ms")
            base.send(opcode, struct.pack("<hHHH", cdb, atk, dcy, hang))
            off = (cdb == 0 and atk == 0 and dcy == 0)
            STATE[name] = ("%s target=%.2f dBFS atk=%dms dcy=%dms hang=%dms"
                           % ("OFF" if off else "ON", tgt, atk, dcy, hang))
            print("  -> AGC %s sent" % STATE[name])
        elif argtype == "codec":
            page = int(parts[1], 0)
            reg = int(parts[2], 0)
            cval = int(parts[3], 0)
            if not (0 <= page <= 255 and 0 <= reg <= 255 and 0 <= cval <= 255):
                raise ValueError("page/reg/val are 0..255")
            arg = (page & 0xFF) | ((reg & 0xFF) << 8) | ((cval & 0xFF) << 16)
            base.send(opcode, struct.pack("<I", arg))
            STATE[name] = "P%d R0x%02X = 0x%02X" % (page, reg, cval)
            print("  -> CODEC %s sent" % STATE[name])
    except (IndexError, ValueError) as e:
        print("  ! %s -- %s needs an argument" % (e, name))
    except OSError as e:
        print("  ! I2C error: %s" % e)
    return True


def main():
    ap = argparse.ArgumentParser(description="X6100 BASE phase3-rx control + RF explorer")
    ap.add_argument("--bus", type=int, default=0, help="I2C bus number (default 0)")
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=0x72,
                    help="BASE 7-bit I2C address (default 0x72)")
    args = ap.parse_args()

    try:
        base = Base(args.bus, args.addr)
    except OSError as e:
        sys.exit("cannot open /dev/i2c-%d @ 0x%02X: %s" % (args.bus, args.addr, e))

    histfile = os.path.expanduser("~/.phase3-rx-ctl_history")
    if readline is not None:
        try:
            readline.read_history_file(histfile)
        except OSError:
            pass

    print("X6100 BASE phase3-rx on /dev/i2c-%d @ 0x%02X. Commands: TUNE VOLUME AGC LOOFF MODE "
          "LINMULT RFPRIM RFSEC BANDV RFIMG SC6 DUMP (help, quit)." % (args.bus, args.addr))
    try:
        if not sys.stdin.isatty():
            for line in sys.stdin:
                handle(base, line.strip())
        else:
            while True:
                try:
                    line = input("rx> ")
                except EOFError:
                    break
                if not handle(base, line):
                    break
    finally:
        if readline is not None:
            try:
                readline.write_history_file(histfile)
            except OSError:
                pass
        base.close()


if __name__ == "__main__":
    main()
