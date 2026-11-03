#!/usr/bin/env python3
"""
band_sweep.py -- phase3-rx RF band-switching validator / frequency sweep.

Validates the RX band-switching code across EVERY switching boundary, graded
entirely from firmware telemetry (no listening required), on a marginal
antenna. Runs on the DEV PC: it sends tune/dump commands to the BASE over SSH
(via /opt/fwdev/phase3-rx-ctl.py on the radio) and reads the BASE's USART1
diagnostics from a growing log file (the `cat /dev/ttyACM1 > /tmp/mcu-NN.log`
capture) -- it never opens the serial device itself (single-reader rule).

Three independent checks, none needing a confirmed signal:

  1. BOUNDARY FLIPS  -- at every primary / secondary / BAND_V edge, tune just
     below and just above and confirm the firmware selects the expected
     primary index (PD9/10/11/13), secondary sub-band bit (SPI6 image), and
     BAND_V DAC2 code. Pure register check vs the VERIFIED frequency_bands.csv
     tables; antenna-independent.
  2. FRONT-END ALIVE -- per band, the median env_dc / iq_rms on an antenna
     should sit above the internal noise floor and vary band to band. A band
     pinned flat flags a front end that is not enabling there.
  3. CARRIER CONTRAST (opportunistic) -- for standard-frequency carriers
     (WWV / CHU) and known broadcasters, measure env_dc ON the carrier vs
     ~20 kHz OFF it, in AM. A ratio above threshold objectively CONFIRMS
     reception in that band -- no ear needed. Absence only fails to confirm
     (marginal antenna), it does not disprove the band.

NOTE: cf88 ("RF chain6") is the TX-power attenuator (arg = 0x64 - power), not an
RX filter -- it is reported for completeness but NOT graded for RX.

The sweep is silent: VOLUME is never touched, so no audio and no approval are
needed. env_dc / iq_rms are measured pre-AGC, so AGC may stay enabled.

Usage (from the dev PC, radio reachable, log being captured):
    tools/band_sweep.py --log /tmp/mcu-11.log
    tools/band_sweep.py --log /tmp/mcu-11.log --quick
    tools/band_sweep.py --log /tmp/mcu-11.log --only carriers
    tools/band_sweep.py --log /tmp/mcu-11.log --report /tmp/sweep.txt
"""
import argparse
import os
import re
import subprocess
import sys
import time

# ----------------------------------------------------------------------------
# Firmware band tables -- EXACT mirror of firmwares/phase3-rx/firmware.c
# (rf_primary_index / rf_secondary_index / band_v_code). Kept in lockstep so
# "expected" is what the firmware should compute. Verified against x6100-base
# analysis/frequency_control/frequency_bands.csv.
# ----------------------------------------------------------------------------
AM_LO_OFFSET = -100          # firmware AM default; TUNE in AM applies this


def fw_primary_index(hz):
    if hz < 1500000 or hz > 55000000:
        return 0
    if hz <= 2200000:
        return 1
    if hz <= 3800000:
        return 2
    if hz <= 5500000:
        return 3
    if hz <= 7500000:
        return 4
    if hz <= 13500000:
        return 5
    if hz <= 21000000:
        return 6
    if hz <= 33000000:
        return 7
    if hz <= 50000000:
        return 8
    return 9


def fw_secondary_index(hz):
    if hz <= 2000000:
        return 0
    if hz <= 4000000:
        return 1
    if hz <= 8000000:
        return 2
    if hz <= 15000000:
        return 3
    if hz <= 22000000:
        return 4
    if hz <= 30000000:
        return 5
    return 6


def fw_band_v(hz):
    if hz <= 2000000:
        return 0x11D
    if hz <= 5000000:
        return 0x23A
    if hz <= 7000000:
        return 0x358
    if hz <= 10000000:
        return 0x475
    if hz <= 14000000:
        return 0x593
    if hz <= 18000000:
        return 0x6B0
    if hz <= 21000000:
        return 0x7CD
    if hz <= 24000000:
        return 0x8EB
    if hz <= 28000000:
        return 0xA08
    if hz <= 50000000:
        return 0xB26
    return 0xC43


def primary_idx_from_odr(odr):
    """Reconstruct the primary index from GPIOD.ODR (PD9=w1,10=w2,11=w4,13=w8)."""
    return (((odr >> 9) & 1) * 1 + ((odr >> 10) & 1) * 2
            + ((odr >> 11) & 1) * 4 + ((odr >> 13) & 1) * 8)


def secondary_bit_from_img(img):
    """The secondary sub-band bit is one of image bits 7..13; return its index
    (idx = 13 - bit) if exactly one of those bits is set, else None."""
    masked = img & 0x3F80
    if masked == 0 or (masked & (masked - 1)) != 0:
        return None
    bit = masked.bit_length() - 1
    return 13 - bit


# ----------------------------------------------------------------------------
# Log parsing
# ----------------------------------------------------------------------------
RE_TUNE = re.compile(r"I2C cmd: TUNE (\d+) Hz")
RE_RF = re.compile(r"RF: primary idx=(\d+) .*? secondary idx=(\d+) \(image bit (\d+)\)"
                   r"\s+img=0x([0-9A-Fa-f]+)")
RE_LO = re.compile(r"LO: OUT1=(\d+) Hz")
RE_CHAIN = re.compile(r"RF chain6=0x([0-9A-Fa-f]+)")
RE_DUMP = re.compile(r"DUMP BAND_V=0x([0-9A-Fa-f]+) RFimg=0x([0-9A-Fa-f]+)"
                     r" GPIOD\.ODR=0x([0-9A-Fa-f]+)")
RE_ALIVE = re.compile(r"iq_rms=(-?\d+) env_dc=(-?\d+) audio_pk=(-?\d+)")


def median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return None
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


class Telemetry:
    """Follows a growing USART1 log file by byte offset (never opens the tty)."""

    def __init__(self, path):
        self.path = path

    def size(self):
        try:
            return os.path.getsize(self.path)
        except OSError:
            return 0

    def read_from(self, offset):
        try:
            with open(self.path, "r", errors="replace") as f:
                f.seek(offset)
                return f.read()
        except OSError:
            return ""


class Radio:
    """Sends commands to phase3-rx over SSH via phase3-rx-ctl.py on the radio.

    Uses a single multiplexed SSH connection (ControlMaster) so dozens of sends
    reuse one TCP/SSH session instead of reconnecting each time -- rapid fresh
    connections to the embedded sshd drop ~10%% of sends. Each send is retried."""

    def __init__(self, ssh, ctl, dry=False, retries=3):
        self.ssh = ssh
        self.ctl = ctl
        self.dry = dry
        self.retries = retries

    def send(self, *cmds):
        # Feed the commands to ctl.py's stdin via printf with LITERAL \n (so the
        # ssh transport carries no real newlines -- those would split the remote
        # command). ctl.py reads one command per line. printf expands the \n on
        # the radio. Commands here never contain a single quote.
        payload = "\\n".join(cmds) + "\\n"
        remote = "printf '%s' | %s" % (payload, self.ctl)
        if self.dry:
            print("[dry] %s" % remote)
            return
        full = self.ssh + [remote]
        for attempt in range(1, self.retries + 1):
            try:
                r = subprocess.run(full, timeout=20, check=False,
                                   stdout=subprocess.DEVNULL,
                                   stderr=subprocess.PIPE)
                if r.returncode == 0:
                    return
                err = (r.stderr or b"").decode("utf-8", "replace").strip()
            except subprocess.TimeoutExpired:
                err = "timeout"
            if attempt < self.retries:
                time.sleep(0.5 * attempt)
        print("  ! ssh send failed after %d tries (%r): %s"
              % (self.retries, cmds, err))


# ----------------------------------------------------------------------------
# One measured step: tune, dump, dwell, parse the window.
# ----------------------------------------------------------------------------
class StepResult:
    def __init__(self, hz, label):
        self.hz = hz
        self.label = label
        self.rf = None          # (pidx, sidx, imgbit, img) from the RF: line
        self.lo = None          # OUT1 Hz
        self.chain6 = None      # cf88 word (TX power, not graded)
        self.dump = None        # (band_v, rfimg, odr)
        self.env = []           # env_dc samples
        self.iq = []            # iq_rms samples
        self.apk = []           # audio_pk samples

    @property
    def env_med(self):
        return median(self.env)

    @property
    def iq_med(self):
        return median(self.iq)


def drive_step(radio, hz, dwell):
    """Send the tune + dump and dwell so the BASE emits this freq's block and a
    few alive lines. No parsing here -- the log is block-buffered by `cat`, so we
    parse the whole capture after the sweep (parse_block), anchored on the unique
    `I2C cmd: TUNE <hz>` echo."""
    radio.send("T %d" % hz, "D")
    if not radio.dry:
        time.sleep(dwell)


def parse_block(hz, label, body):
    """Parse one TUNE block (text between this tune's echo and the next tune)."""
    res = StepResult(hz, label)
    m = RE_RF.search(body)
    if m:
        res.rf = (int(m.group(1)), int(m.group(2)), int(m.group(3)),
                  int(m.group(4), 16))
    m = RE_LO.search(body)
    if m:
        res.lo = int(m.group(1))
    m = RE_CHAIN.search(body)
    if m:
        res.chain6 = int(m.group(1), 16)
    m = RE_DUMP.search(body)
    if m:
        res.dump = (int(m.group(1), 16), int(m.group(2), 16), int(m.group(3), 16))
    for a in RE_ALIVE.finditer(body):
        res.iq.append(int(a.group(1)))
        res.env.append(int(a.group(2)))
        res.apk.append(int(a.group(3)))
    return res


def split_tune_blocks(window):
    """Yield (hz, body) for each TUNE echo in order; body runs to the next echo."""
    anchors = [(int(m.group(1)), m.start(), m.end())
               for m in RE_TUNE.finditer(window)]
    for i, (hz, _s, end) in enumerate(anchors):
        stop = anchors[i + 1][1] if i + 1 < len(anchors) else len(window)
        yield hz, window[end:stop]


def expected_for(hz, mode_am=True):
    ftuned = hz + (AM_LO_OFFSET if mode_am else 0)
    return {
        "pidx": fw_primary_index(ftuned),
        "sidx": fw_secondary_index(ftuned),
        "band_v": fw_band_v(hz),            # firmware uses station_hz for BAND_V
        "lo": 4 * ftuned,
    }


# ----------------------------------------------------------------------------
# Sweep plan
# ----------------------------------------------------------------------------
PRIMARY_EDGES = [1500000, 2200000, 3800000, 5500000, 7500000,
                 13500000, 21000000, 33000000, 50000000]
SECONDARY_EDGES = [2000000, 4000000, 8000000, 15000000, 22000000, 30000000]
BANDV_EDGES = [2000000, 5000000, 7000000, 10000000, 14000000,
               18000000, 21000000, 24000000, 28000000, 50000000]

# Representative mid-band frequency per primary band (for front-end-alive).
BAND_REPS = [
    (1, 1800000), (2, 3000000), (3, 4500000), (4, 6500000), (5, 10000000),
    (6, 18000000), (7, 27000000), (8, 40000000), (9, 52000000),
]

# Opportunistic carriers: (name, carrier_hz). AM; measured on vs +20 kHz off.
# WDOV 1410 is Franco's known-good AM broadcaster -- a POSITIVE CONTROL: if it
# shows contrast and the HF carriers do not, the method works and the antenna is
# simply marginal on HF (not a firmware failure). CHU (3.33/7.85/14.67 MHz) was
# decommissioned in 2025, so it is NOT used; WWV covers the same bands (2/5/6).
CARRIERS = [
    ("WDOV 1410 (ctrl)", 1410000),   # band 1 positive control (known-good AM)
    ("WWV 2.5 MHz", 2500000),        # band 2
    ("WWV 5 MHz", 5000000),          # band 3
    ("WWV 10 MHz", 10000000),        # band 5
    ("WWV 15 MHz", 15000000),        # band 6
    ("WWV 20 MHz", 20000000),        # band 6
    ("WWV 25 MHz", 25000000),        # band 7
]
CARRIER_OFFSET = 20000       # Hz off-carrier for the empty reference
CONTRAST_RATIO = 1.5         # env_dc on/off ratio to call a band CONFIRMED


def build_plan(which, guard=5000):
    """Return a list of ('boundary'|'rep'|'carrier', dict) steps."""
    plan = []
    if which in ("all", "boundaries", "quick"):
        edges = sorted(set(PRIMARY_EDGES + SECONDARY_EDGES + BANDV_EDGES))
        if which == "quick":
            edges = sorted(set(PRIMARY_EDGES))      # primary edges only
        for e in edges:
            plan.append(("boundary", {"hz": e - guard, "edge": e, "side": "below"}))
            plan.append(("boundary", {"hz": e + guard, "edge": e, "side": "above"}))
    if which in ("all", "reps"):
        for band, hz in BAND_REPS:
            plan.append(("rep", {"hz": hz, "band": band}))
    if which in ("all", "carriers", "quick"):
        for name, hz in CARRIERS:
            plan.append(("carrier", {"hz": hz, "name": name, "role": "on"}))
            plan.append(("carrier", {"hz": hz + CARRIER_OFFSET, "name": name,
                                     "role": "off"}))
    return plan


# ----------------------------------------------------------------------------
# Report
# ----------------------------------------------------------------------------
def fmt_boundary_report(rows):
    out = ["== BOUNDARY FLIPS (expected vs actual; antenna-independent) ==",
           "%-11s %-9s  %-13s %-13s %-10s %s"
           % ("freq", "edge/side", "pidx(exp/act)", "sidx(exp/act)",
              "BAND_V", "verdict")]
    npass = nfail = 0
    for r in rows:
        exp, act = r["exp"], r["res"]
        a_p = a_s = a_bv = "--"
        ok = True
        if act.rf:
            a_p, a_s = act.rf[0], act.rf[1]
            if a_p != exp["pidx"] or a_s != exp["sidx"]:
                ok = False
        else:
            ok = False
        if act.dump:
            a_bv = "0x%03X" % act.dump[0]
            if act.dump[0] != exp["band_v"]:
                ok = False
            # cross-check ODR-derived primary idx matches the RF: line
            odr_p = primary_idx_from_odr(act.dump[2])
            if act.rf and odr_p != act.rf[0]:
                ok = False
        else:
            ok = False
        verdict = "PASS" if ok else "**FAIL**"
        npass += ok
        nfail += not ok
        out.append("%-11d %-9s  %-13s %-13s %-10s %s" % (
            act.hz, "%.3fM/%s" % (r["edge"] / 1e6, r["side"]),
            "%d/%s" % (exp["pidx"], a_p), "%d/%s" % (exp["sidx"], a_s),
            "%s(0x%03X)" % (a_bv, exp["band_v"]), verdict))
    out.append("  -> %d pass, %d FAIL" % (npass, nfail))
    return "\n".join(out), nfail


def fmt_rep_report(rows):
    out = ["", "== FRONT-END ALIVE (per-band median levels, x1e-6) ==",
           "%-6s %-11s %-10s %-10s %s"
           % ("band", "freq", "env_dc", "iq_rms", "n")]
    for r in rows:
        act = r["res"]
        out.append("%-6d %-11d %-10s %-10s %d" % (
            r["band"], act.hz,
            "%.0f" % act.env_med if act.env_med is not None else "--",
            "%.0f" % act.iq_med if act.iq_med is not None else "--",
            len(act.env)))
    return "\n".join(out)


def fmt_carrier_report(pairs):
    out = ["", "== CARRIER CONTRAST (AM env_dc on vs +%d kHz off; x1e-6) =="
           % (CARRIER_OFFSET // 1000),
           "%-15s %-9s %-9s %-9s %s"
           % ("source", "on", "off", "ratio", "verdict")]
    confirmed = []
    for name, on, off in pairs:
        on_e = on.env_med if on else None
        off_e = off.env_med if off else None
        ratio = (on_e / off_e) if (on_e and off_e and off_e > 0) else None
        if ratio is not None and ratio >= CONTRAST_RATIO:
            verdict = "CONFIRMED"
            confirmed.append(name)
        elif ratio is not None:
            verdict = "no contrast"
        else:
            verdict = "no data"
        out.append("%-15s %-9s %-9s %-9s %s" % (
            name,
            "%.0f" % on_e if on_e is not None else "--",
            "%.0f" % off_e if off_e is not None else "--",
            "%.2f" % ratio if ratio is not None else "--",
            verdict))
    out.append("  -> CONFIRMED bands: %s"
               % (", ".join(confirmed) if confirmed else "none (marginal antenna; "
                  "no positive confirmation -- not a failure)"))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description="phase3-rx RF band-switching sweep")
    ap.add_argument("--log", default="/tmp/mcu-11.log",
                    help="USART1 capture file to follow (default /tmp/mcu-11.log)")
    ap.add_argument("--ssh", default="ssh -i ~/.ssh/x6100 -o ConnectTimeout=8 "
                    "-o StrictHostKeyChecking=no "
                    "-o ControlMaster=auto -o ControlPath=/tmp/.x6100-sweep-%r@%h:%p "
                    "-o ControlPersist=120 root@192.168.254.20",
                    help="ssh command prefix to reach the radio (multiplexed)")
    ap.add_argument("--retries", type=int, default=3,
                    help="send retries per command")
    ap.add_argument("--ctl", default="python3 /opt/fwdev/phase3-rx-ctl.py",
                    help="control script invocation on the radio")
    ap.add_argument("--only", choices=["all", "boundaries", "reps", "carriers", "quick"],
                    default="all", help="which checks to run")
    ap.add_argument("--quick", action="store_true",
                    help="shorthand for --only quick (primary edges + carriers)")
    ap.add_argument("--dwell", type=float, default=3.0,
                    help="seconds to dwell per step (alive-line sampling)")
    ap.add_argument("--boundary-dwell", type=float, default=1.6,
                    help="shorter dwell for register-only boundary checks")
    ap.add_argument("--report", default=None, help="also write the report here")
    ap.add_argument("--dry-run", action="store_true",
                    help="print commands, do not send (no radio needed)")
    args = ap.parse_args()

    which = "quick" if args.quick else args.only
    ssh = os.path.expanduser(args.ssh).split()
    ssh = [os.path.expanduser(p) if p.startswith("~") else p for p in ssh]
    radio = Radio(ssh, args.ctl, dry=args.dry_run, retries=args.retries)
    tele = Telemetry(args.log)

    if not args.dry_run and not os.path.exists(args.log):
        sys.exit("log file %s not found -- is the ttyACM1 capture running?" % args.log)

    print("phase3-rx band sweep: mode=AM, which=%s, log=%s" % (which, args.log))
    print("(silent: VOLUME untouched; env_dc/iq_rms are pre-AGC)\n")
    radio.send("M AM")          # AM for env_dc discriminator + default LO offset
    if not args.dry_run:
        time.sleep(1.0)

    plan = build_plan(which)

    # Phase 1: DRIVE. Send every tune+dump with a dwell so the BASE emits each
    # freq's block + alive lines. We parse afterward from the full capture.
    start_off = 0 if args.dry_run else tele.size()
    driven = []                 # ordered [(kind, p, tag)]
    for i, (kind, p) in enumerate(plan, 1):
        dwell = args.boundary_dwell if kind == "boundary" else args.dwell
        if kind == "boundary":
            tag = "%.3fM %s" % (p["edge"] / 1e6, p["side"])
        elif kind == "rep":
            tag = "band %d" % p.get("band", 0)
        else:
            tag = "%s %s" % (p["name"], p["role"])
        print("[%2d/%2d] %-22s T %d" % (i, len(plan), tag, p["hz"]))
        drive_step(radio, p["hz"], dwell)
        driven.append((kind, p, tag))

    # FLUSH SENTINEL: a unique freq used by no step (so its echo is unambiguous
    # in the block-buffered log); once it appears, every earlier step has
    # flushed. Then restore to the benign known-good WDOV 1410 AM, silent.
    SENTINEL = 1400000
    radio.send("T %d" % SENTINEL)
    radio.send("T 1410000", "M AM")

    # Phase 2: wait for the sentinel to flush, then read + parse the whole window.
    b_rows, rep_rows, car_steps = [], [], {}
    if not args.dry_run:
        deadline = time.time() + 70
        while time.time() < deadline:
            window = tele.read_from(start_off)
            if ("I2C cmd: TUNE %d Hz" % SENTINEL) in window:
                break
            # nudge the buffer along while we wait
            radio.send("D")
            time.sleep(3.0)
        window = tele.read_from(start_off)
        blocks = list(split_tune_blocks(window))
        # Blocks are 1:1 with sends, in order. Pair positionally; if a freq does
        # not line up (a dropped/duplicated send), resync by searching forward.
        bi = 0
        for kind, p, tag in driven:
            match = ""
            if bi < len(blocks) and blocks[bi][0] == p["hz"]:
                match = blocks[bi][1]
                bi += 1
            else:
                for j in range(bi, len(blocks)):
                    if blocks[j][0] == p["hz"]:
                        match = blocks[j][1]
                        bi = j + 1
                        break
                else:
                    print("  ! no log block for %s (T %d)" % (tag, p["hz"]))
            res = parse_block(p["hz"], tag, match)
            if kind == "boundary":
                b_rows.append({"edge": p["edge"], "side": p["side"],
                               "exp": expected_for(p["hz"]), "res": res})
            elif kind == "rep":
                rep_rows.append({"band": p["band"], "res": res})
            else:
                car_steps.setdefault(p["name"], {})[p["role"]] = res

    sections = []
    nfail = 0
    if b_rows:
        s, nfail = fmt_boundary_report(b_rows)
        sections.append(s)
    if rep_rows:
        sections.append(fmt_rep_report(rep_rows))
    if car_steps:
        pairs = [(name, d.get("on"), d.get("off"))
                 for name, d in car_steps.items()]
        sections.append(fmt_carrier_report(pairs))
    report = "\n".join(sections)
    print("\n" + report)
    if args.report:
        with open(args.report, "w") as f:
            f.write(report + "\n")
        print("\n[report written to %s]" % args.report)
    print("\nDONE (%s)" % ("boundary FAILs present -- see above" if nfail
                           else "no boundary failures"))


if __name__ == "__main__":
    main()
