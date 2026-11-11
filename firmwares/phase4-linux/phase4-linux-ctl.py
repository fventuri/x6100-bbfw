#!/usr/bin/env python3
"""
phase4-linux-ctl.py -- X6100 BASE (phase4-linux) OEM I2C control-plane tool.

Runs on the R16 CPU (Linux) and drives the phase4-linux BASE over the OEM
hardware contract: the STM32 I2C3 slave at 7-bit 0x72 on /dev/i2c-0, using the
exact framing the stock app (xradio / thread_remote_admin) uses -- NOT the
phase3-rx custom-opcode protocol.

OEM framing (x6100-base docs/linux_hardware_interface.md + docs/protocol.md):
  * Address = byte offset = control-word-index * 4, sent BIG-ENDIAN (hi, lo).
  * WRITE  = one I2C_RDWR message: [off_hi][off_lo][payload little-endian...].
  * READ   = TWO separate ioctls: an offset-only 2-byte write (STOP), then a
             one-message read. The slave keeps the offset across the STOP.
  * The read/write windows are DIRECTION-DEPENDENT (same address, different RAM):
      - a READ at 0x0000..0x007F returns the 128-byte firmware-id ("V1.1.9...");
        0x0080..0x00FF -> 0; 0x0100..0x0FFF -> calibration; 0xFFFE.. -> 0x61.
      - a WRITE at <0xF000 lands in the 56-word (0xE0 B) control file;
        0xF000..0xFFFD -> calibration; 0xFFFE.. -> command window.
    => You CANNOT read back a control word you just wrote. Confirm control
       writes and the boot-handshake release on the BASE USART1 diagnostic
       (/dev/ttyACM1, 115200): it prints "reg[..]" changes and, once reg55 bit
       0x00100000 is set, "Linux boot-ready seen ... Continuing."

Commands (REPL or piped; up-arrow history):
  TUNE  <hz> [a|b]        set VFO A/B frequency (REG 1/7); default VFO a
  MODE  <name> [a|b]      set VFO A/B mode (REG 4/10): lsb lsb_dig usb usb_dig
                          cw cwr am nfm; default VFO a
  VOL   <0..59>           set rxvol (REG 14): 0=mute, 30=0 dB, 59=+29 dB
  VFO   <a|b>             select the active VFO (REG 13 bit0)
  ID                      read the 128-byte fw-id @offset 0; expect "V1.1.9..."
  READ  <off> [len]       raw read of the READ window (default len 16; off base-0)
  REG   <idx> <u32>       write one control word: offset idx*4, value LE u32
  WRITE <off> <byte...>   raw write: BE offset then the given bytes (base-0)
  BULK  [w0..w55]         write the whole 56-word window @0 (missing words = 0)
  CMD   <byte...>         write the command window @0xFFFE
  GO | HANDSHAKE          REG 55 0x00100001 -- release the Linux boot handshake
  INIT                    zero the 56-word window, then GO (mimics X6100Control)
  HELP | H | ?,  QUIT | Q

Usage:
  ./phase4-linux-ctl.py                     # REPL, /dev/i2c-0 @ 0x72
  ./phase4-linux-ctl.py --bus 0 --addr 0x72
  echo "ID" | ./phase4-linux-ctl.py         # one-shot
  ./phase4-linux-ctl.py --dry-run           # print the I2C byte stream, no device
  echo -e "ID\nGO" | ./phase4-linux-ctl.py --dry-run
"""
import argparse
import ctypes
import fcntl
import os
import struct
import sys

try:
    import readline  # noqa: F401  -- line editing + history in input()
except ImportError:
    readline = None

I2C_RDWR        = 0x0707
I2C_SLAVE_FORCE = 0x0706
I2C_M_RD        = 0x0001

HANDSHAKE_INDEX = 55            # byte offset 0xDC
HANDSHAKE_VALUE = 0x00100001    # bit 0x00100000 releases the BASE boot loop
CMD_OFFSET      = 0xFFFE        # command window
CTRL_WORDS      = 56            # 0xE0-byte control file

# Control-word indices the phase4-linux BASE acts on (firmware control_scan_apply;
# x6100-base docs/protocol.md). These back the convenience commands below.
VFO_FREQ_IDX   = {"a": 1,  "b": 7}    # reg1/7  VFO A/B frequency (u32 Hz)
VFO_MODE_IDX   = {"a": 4,  "b": 10}   # reg4/10 VFO A/B mode (enum 0..7)
ACTIVE_VFO_IDX = 13                   # reg13 bit0: 0=VFO A, 1=VFO B
RXVOL_IDX      = 14                   # reg14 low byte: rxvol 0..59
RXVOL_MAX      = 59                   # OEM range: 0=mute, 30=0 dB, 59=+29 dB
# OEM mode enum (docs/protocol.md:278): 0..7 = LSB,LSB_DIG,USB,USB_DIG,CW,CWR,AM,NFM
MODE_ENUM = {"lsb": 0, "lsb_dig": 1, "lsbdig": 1, "usb": 2, "usb_dig": 3,
             "usbdig": 3, "cw": 4, "cwr": 5, "am": 6, "nfm": 7, "fm": 7}


class i2c_msg(ctypes.Structure):
    # Native kernel struct i2c_msg; ctypes gives correct layout/alignment on
    # both the 32-bit R16 target and a 64-bit dev host.
    _fields_ = [("addr", ctypes.c_uint16),
                ("flags", ctypes.c_uint16),
                ("len", ctypes.c_uint16),
                ("buf", ctypes.c_void_p)]


class i2c_rdwr_ioctl_data(ctypes.Structure):
    _fields_ = [("msgs", ctypes.POINTER(i2c_msg)),
                ("nmsgs", ctypes.c_uint32)]


class Base:
    """OEM-framed access to the BASE I2C3 slave via raw I2C_RDWR messages."""

    def __init__(self, bus, addr, dry_run=False):
        self.addr = addr
        self.dry_run = dry_run
        self.fd = None
        if not dry_run:
            self.fd = os.open("/dev/i2c-%d" % bus, os.O_RDWR)

    def _xfer(self, msgs):
        arr = (i2c_msg * len(msgs))(*msgs)
        req = i2c_rdwr_ioctl_data(
            msgs=ctypes.cast(arr, ctypes.POINTER(i2c_msg)), nmsgs=len(msgs))
        fcntl.ioctl(self.fd, I2C_RDWR, req)

    def write(self, off, payload=b""):
        """One I2C_RDWR write: [off_hi][off_lo][payload...] (STOP at the end)."""
        stream = bytes([(off >> 8) & 0xFF, off & 0xFF]) + bytes(payload)
        if self.dry_run:
            print("  [W @0x%04X] %s" % (off, stream.hex(" ")))
            return
        buf = ctypes.create_string_buffer(stream, len(stream))
        msg = i2c_msg(addr=self.addr, flags=0, len=len(stream),
                      buf=ctypes.cast(buf, ctypes.c_void_p))
        self._xfer([msg])

    def read(self, off, n):
        """Offset-only write (STOP), then a separate n-byte read -- OEM ui2c_read."""
        self.write(off, b"")                    # select offset, STOP between
        if self.dry_run:
            print("  [R @0x%04X] %d bytes" % (off, n))
            return bytes(n)
        rbuf = ctypes.create_string_buffer(n)
        msg = i2c_msg(addr=self.addr, flags=I2C_M_RD, len=n,
                      buf=ctypes.cast(rbuf, ctypes.c_void_p))
        self._xfer([msg])
        return rbuf.raw[:n]

    def close(self):
        if self.fd is not None:
            os.close(self.fd)


def cmd_id(base):
    data = base.read(0x0000, 0x80)
    if base.dry_run:
        return
    s = data.split(b"\x00", 1)[0]
    txt = "".join(chr(c) if 32 <= c < 127 else "." for c in s)
    ok = s.startswith(b"V1.1.9")
    print("  fw-id: %r  [%s]" % (txt, "OK: V1.1.9 pairing" if ok
                                 else "UNEXPECTED (want V1.1.9...)"))
    print("  hex  : %s" % data[:len(s) + 1].hex(" "))


def cmd_read(base, parts):
    off = int(parts[1], 0)
    n = int(parts[2], 0) if len(parts) > 2 else 16
    data = base.read(off, n)
    if base.dry_run:
        return
    print("  @0x%04X: %s" % (off, data.hex(" ")))


def cmd_reg(base, parts):
    idx = int(parts[1], 0)
    val = int(parts[2], 0) & 0xFFFFFFFF
    if not 0 <= idx < CTRL_WORDS:
        raise ValueError("reg index 0..%d" % (CTRL_WORDS - 1))
    base.write(idx * 4, struct.pack("<I", val))
    print("  -> REG[%d] @0x%04X = 0x%08X" % (idx, idx * 4, val))


def _vfo_arg(parts, i):
    """Optional trailing a|b VFO selector (default a)."""
    vfo = parts[i].lower() if len(parts) > i else "a"
    if vfo not in ("a", "b"):
        raise ValueError("VFO must be a or b")
    return vfo


def cmd_tune(base, parts):
    hz = int(parts[1], 0)
    vfo = _vfo_arg(parts, 2)
    idx = VFO_FREQ_IDX[vfo]
    base.write(idx * 4, struct.pack("<I", hz & 0xFFFFFFFF))
    print("  -> TUNE VFO %s = %d Hz (REG[%d])" % (vfo.upper(), hz, idx))


def cmd_mode(base, parts):
    name = parts[1].lower()
    if name not in MODE_ENUM:
        raise ValueError("mode one of: %s" % ", ".join(sorted(MODE_ENUM)))
    vfo = _vfo_arg(parts, 2)
    val = MODE_ENUM[name]
    idx = VFO_MODE_IDX[vfo]
    base.write(idx * 4, struct.pack("<I", val))
    note = {4: " (RX: zero-beat, no CW tone)", 5: " (RX: zero-beat, no CW tone)",
            7: " (RX: AM demod placeholder)"}.get(val, "")
    print("  -> MODE VFO %s = %s (enum %d, REG[%d])%s"
          % (vfo.upper(), name.upper(), val, idx, note))


def cmd_vol(base, parts):
    v = int(parts[1], 0)
    if not 0 <= v <= RXVOL_MAX:
        raise ValueError("rxvol 0..%d (0=mute, 30=0 dB, 59=+29 dB)" % RXVOL_MAX)
    base.write(RXVOL_IDX * 4, struct.pack("<I", v))
    desc = "mute" if v == 0 else "%+d dB net" % (v - 30)
    print("  -> VOL rxvol=%d (REG[%d]); %s" % (v, RXVOL_IDX, desc))


def cmd_vfo(base, parts):
    sel = _vfo_arg(parts, 1)
    val = 0 if sel == "a" else 1
    base.write(ACTIVE_VFO_IDX * 4, struct.pack("<I", val))
    print("  -> VFO active = %s (REG[%d] = %d)" % (sel.upper(), ACTIVE_VFO_IDX, val))


def cmd_write(base, parts):
    off = int(parts[1], 0)
    payload = bytes(int(p, 0) & 0xFF for p in parts[2:])
    base.write(off, payload)
    print("  -> WRITE @0x%04X %s" % (off, payload.hex(" ") or "(offset only)"))


def cmd_bulk(base, parts):
    words = [int(p, 0) & 0xFFFFFFFF for p in parts[1:]]
    if len(words) > CTRL_WORDS:
        raise ValueError("at most %d words" % CTRL_WORDS)
    words += [0] * (CTRL_WORDS - len(words))
    payload = b"".join(struct.pack("<I", w) for w in words)
    base.write(0x0000, payload)                 # 2 + 56*4 = 226 bytes
    print("  -> BULK 56-word window @0 (%d bytes)" % (len(payload) + 2))


def cmd_cmdwin(base, parts):
    payload = bytes(int(p, 0) & 0xFF for p in parts[1:])
    base.write(CMD_OFFSET, payload)
    print("  -> CMD @0x%04X %s" % (CMD_OFFSET, payload.hex(" ")))


def cmd_go(base):
    base.write(HANDSHAKE_INDEX * 4, struct.pack("<I", HANDSHAKE_VALUE))
    print("  -> HANDSHAKE: REG[%d] @0x%04X = 0x%08X "
          "(watch USART1 for 'Linux boot-ready seen')"
          % (HANDSHAKE_INDEX, HANDSHAKE_INDEX * 4, HANDSHAKE_VALUE))


def cmd_init(base):
    cmd_bulk(base, ["BULK"])                     # all-zero 56-word window
    cmd_go(base)


HELP = """  -- convenience (wrap the raw REG writes with the firmware mapping) --
  TUNE  <hz> [a|b]      set VFO A/B frequency   (REG 1/7)   default a
  MODE  <name> [a|b]    set VFO A/B mode        (REG 4/10)  default a
                        name: lsb lsb_dig usb usb_dig cw cwr am nfm
  VOL   <0..59>         set rxvol               (REG 14)    0=mute 30=0dB 59=+29dB
  VFO   <a|b>           select active VFO       (REG 13 bit0)
  -- raw / OEM --
  ID                    read fw-id @0 (expect V1.1.9...)
  READ  <off> [len]     raw read of the READ window (default len 16)
  REG   <idx> <u32>     write control word at offset idx*4 (LE)
  WRITE <off> <byte...> raw write: BE offset then bytes
  BULK  [w0..w55]       write the 56-word window @0 (missing = 0)
  CMD   <byte...>       write the command window @0xFFFE
  GO | HANDSHAKE        REG 55 0x00100001 -- release the boot handshake
  INIT                  zero the window then GO
  QUIT | Q
  (reads are direction-dependent: low offsets return the fw-id, not your
   control words -- confirm writes/handshake on the BASE USART1 log.
   NB: a control word only takes effect once a VFO frequency is set, so
   TUNE first, then MODE/VOL.)"""


def handle(base, line):
    parts = line.split()
    if not parts:
        return True
    key = parts[0].upper()
    try:
        if key in ("QUIT", "EXIT", "Q"):
            return False
        if key in ("HELP", "H", "?"):
            print(HELP)
        elif key in ("ID", "FWID"):
            cmd_id(base)
        elif key == "READ":
            cmd_read(base, parts)
        elif key == "REG":
            cmd_reg(base, parts)
        elif key == "TUNE":
            cmd_tune(base, parts)
        elif key == "MODE":
            cmd_mode(base, parts)
        elif key in ("VOL", "VOLUME"):
            cmd_vol(base, parts)
        elif key == "VFO":
            cmd_vfo(base, parts)
        elif key == "WRITE":
            cmd_write(base, parts)
        elif key == "BULK":
            cmd_bulk(base, parts)
        elif key == "CMD":
            cmd_cmdwin(base, parts)
        elif key in ("GO", "HANDSHAKE"):
            cmd_go(base)
        elif key == "INIT":
            cmd_init(base)
        else:
            print("  ? unknown command %r (try: help)" % parts[0])
    except (IndexError, ValueError) as e:
        print("  ! bad arguments for %s: %s" % (key, e))
    except OSError as e:
        print("  ! I2C error: %s" % e)
    return True


def main():
    ap = argparse.ArgumentParser(description="X6100 BASE phase4-linux OEM I2C control plane")
    ap.add_argument("--bus", type=int, default=0, help="I2C bus number (default 0)")
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=0x72,
                    help="BASE 7-bit I2C address (default 0x72)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the I2C byte stream instead of touching a device")
    args = ap.parse_args()

    try:
        base = Base(args.bus, args.addr, dry_run=args.dry_run)
    except OSError as e:
        sys.exit("cannot open /dev/i2c-%d @ 0x%02X: %s" % (args.bus, args.addr, e))

    where = "DRY-RUN (no device)" if args.dry_run else "/dev/i2c-%d @ 0x%02X" % (args.bus, args.addr)
    print("X6100 BASE phase4-linux on %s. Commands: TUNE MODE VOL VFO | ID READ REG "
          "WRITE BULK CMD GO INIT (help, quit)." % where)

    histfile = os.path.expanduser("~/.phase4-linux-ctl_history")
    if readline is not None and not args.dry_run:
        try:
            readline.read_history_file(histfile)
        except OSError:
            pass
    try:
        if not sys.stdin.isatty():
            for line in sys.stdin:
                handle(base, line.strip())
        else:
            while True:
                try:
                    line = input("p4> ")
                except EOFError:
                    break
                if not handle(base, line):
                    break
    finally:
        if readline is not None and not args.dry_run:
            try:
                readline.write_history_file(histfile)
            except OSError:
                pass
        base.close()


if __name__ == "__main__":
    main()
