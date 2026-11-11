#!/usr/bin/env python3
# /dev/ttyS1 USART6 flow-packet reader/validator for x6100-bbfw phase4-linux.
# Opens the port at 1.5 Mbaud, syncs on the 0xAA5555AA magic, collects 4136-byte
# frames, re-checks the STM32 hardware CRC32 (poly 0x04C11DB7, init 0xFFFFFFFF,
# non-reflected, 32-bit word input), and decodes the 512-complex-f32 IQ payload.
#
# R16 UART GOTCHA (kernel 7.3.1): the STM32 floods ttyS1 at 1.5 M continuously,
# so whenever NO process is draining the port its RX FIFO overruns and the
# DesignWare driver latches a RX error -- the next open then reads 0 bytes until
# the UART is reset. Two mitigations here: (1) open at a throwaway baud (1152000)
# then bounce to 1500000, which forces a divisor reprogram that clears the latch;
# (2) once reading, drain fast and never idle. A single standalone run works best
# as the FIRST read after the port has been idle a few seconds; the production
# x6100_gui avoids the latch entirely by opening once and reading forever. If you
# get "0 bytes / NO FRAMES", wait ~10 s (let the latch clear) and re-run.
import os, sys, struct, fcntl, time, select

DEV    = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyS1"
NWANT  = int(sys.argv[2]) if len(sys.argv) > 2 else 20
BAUD   = 1500000
FRAME  = 0x1028                 # 4136 bytes
NWORDS = FRAME // 4            # 1034 words
CRCW   = 0x409                 # 1033 words are CRC'd; word 1033 holds the CRC
MAGIC  = b"\xAA\x55\x55\xAA"

# ---- Port open at the custom 1.5 Mbaud. Prefer pyserial (it drives TCSETS2/
# BOTHER correctly for whatever kernel/arch is running -- the hand-rolled struct
# below has a fixed layout that broke on the refreshed R16 kernel 7.3.1). Fall
# back to the raw termios2 ioctl only if pyserial is absent. A reader object
# exposes read(n)->bytes and close(). ------------------------------------------
TCGETS2, TCSETS2 = 0x802C542A, 0x402C542B
BOTHER, CBAUD = 0o010000, 0o010017
CS8, CSIZE = 0o060, 0o060
CLOCAL, CREAD, PARENB, CSTOPB = 0o004000, 0o000200, 0o000400, 0o000100

class _Termios2Reader:
    def __init__(self, dev, baud):
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        b = bytearray(44)                               # sizeof(struct termios2)
        fcntl.ioctl(self.fd, TCGETS2, b)
        iflag, oflag, cflag, lflag = struct.unpack_from("IIII", b, 0)
        iflag = oflag = lflag = 0
        cflag &= ~(CBAUD | CSIZE | PARENB | CSTOPB)
        cflag |= CS8 | CLOCAL | CREAD | BOTHER
        struct.pack_into("IIII", b, 0, iflag, oflag, cflag, lflag)
        b[17 + 6] = 1; b[17 + 5] = 0                    # c_cc[VMIN]=1 VTIME=0
        struct.pack_into("II", b, 36, baud, baud)       # c_ispeed, c_ospeed
        fcntl.ioctl(self.fd, TCSETS2, b)
    def read(self, n):
        r, _, _ = select.select([self.fd], [], [], 2.0)
        if not r:
            return b""
        try:
            return os.read(self.fd, n)
        except BlockingIOError:
            return b""
    def close(self):
        os.close(self.fd)

def open_reader(dev, baud):
    try:
        import serial                               # pyserial: correct custom baud
        # Baud-bounce: open at a different standard baud first, then set the real
        # rate, so the driver reprograms the divisor and clears any latched RX
        # overrun from the port having been flooded while closed (see header).
        ser = serial.Serial(dev, 1152000, timeout=2.0)
        if baud != 1152000:
            ser.baudrate = baud
        ser.reset_input_buffer()
        class _PS:
            def read(self, n): return ser.read(n)   # blocks up to timeout
            def close(self): ser.close()
        return _PS(), "pyserial %s (baud-bounce)" % serial.VERSION
    except ImportError:
        return _Termios2Reader(dev, baud), "termios2"

def stm32_crc32(words):
    crc = 0xFFFFFFFF
    for w in words:
        crc ^= w
        for _ in range(32):
            crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if (crc & 0x80000000) else (crc << 1) & 0xFFFFFFFF
    return crc

print("reading %s @ %d baud, want %d frames (20 s max)..." % (DEV, BAUD, NWANT))
buf = bytearray()
frames, t_first, t_last, total = [], None, None, 0
deadline = time.monotonic() + 20.0
how = None
# Reopen-and-retry: the DesignWare RX overrun latch (see header) leaves some
# opens reading 0 bytes, and others delivering GARBLED bytes (slightly-wrong
# framing -> no magic). So the reopen is gated on FRAMES SYNCED, not raw bytes:
# a window that yields bytes but no 0xAA5555AA frames within ~2.5 s is discarded
# and reopened (fresh baud-bounce + settle) until a clean window lands.
while len(frames) < NWANT and time.monotonic() < deadline:
    try:
        rdr, how = open_reader(DEV, BAUD)
    except OSError as e:
        print("ERROR: could not open %s @ %d: %s" % (DEV, BAUD, e)); sys.exit(2)
    buf = bytearray()                               # fresh sync buffer per open
    frames_before = len(frames)
    stall = time.monotonic() + 2.5                  # reopen if no FRAME for 2.5 s
    while len(frames) < NWANT and time.monotonic() < deadline and time.monotonic() < stall:
        chunk = rdr.read(65536)
        if not chunk:
            continue
        total += len(chunk)
        buf += chunk
        while True:
            i = buf.find(MAGIC)
            if i < 0:
                if len(buf) > 2 * FRAME:
                    del buf[:-3]                     # keep a short tail for a split magic
                break
            if len(buf) - i < FRAME:
                if i:
                    del buf[:i]                      # drop pre-magic garbage
                break
            frames.append(bytes(buf[i:i + FRAME]))
            del buf[:i + FRAME]
            now = time.monotonic()
            if t_first is None:
                t_first = now
            t_last = now
            stall = now + 2.5                        # keep the open while FRAMES sync
            if len(frames) >= NWANT:
                break
    rdr.close()
    if len(frames) == frames_before:
        time.sleep(0.3)                             # nothing synced: settle, then reopen
print("transport: %s" % (how or "n/a"))

print("bytes read: %d" % total)
if not frames:
    print("NO FRAMES: 0xAA5555AA never synced -> wrong device/baud, or the R16 UART is not locking 1.5 M.")
    sys.exit(1)

okc = 0
for fr in frames:
    w = struct.unpack("<%dI" % NWORDS, fr)
    if stm32_crc32(w[:CRCW]) == w[CRCW]:
        okc += 1

w0 = struct.unpack("<%dI" % NWORDS, frames[0])
print("frames: %d    CRC OK: %d/%d" % (len(frames), okc, len(frames)))
print("frame0: magic=0x%08X  crc=0x%08X (recalc 0x%08X)"
      % (w0[0], w0[CRCW], stm32_crc32(w0[:CRCW])))
# Telemetry: flags bit1=tx bit2=atu bit3=vext bit4=charging; 0x100C/D vext/vbat
# in tenths of a volt; 0x100E battery capacity %.
_fl = w0[0x401]
print("telemetry: flags=0x%08X [tx=%d atu=%d vext=%d charging=%d]  Vext=%.1fV  Vbat=%.1fV  cap=%d%%"
      % (_fl, (_fl >> 1) & 1, (_fl >> 2) & 1, (_fl >> 3) & 1, (_fl >> 4) & 1,
         frames[0][0x100C] / 10.0, frames[0][0x100D] / 10.0, frames[0][0x100E]))
# hkey (0x1020): nonzero = a hand-mic/keypad button is held. Report any hkey seen
# across the captured frames (a press is only ~134 ms, so scan all frames).
_hkeys = sorted({struct.unpack_from("<I", fr, 0x1020)[0] for fr in frames} - {0})
print("hkey: 0x1020 across %d frames -> %s"
      % (len(frames), ("pressed " + " ".join("0x%02X" % k for k in _hkeys)) if _hkeys else "0 (no key held)"))

# ---- IQ region (0x0004..0x1003): 512 complex float32 (I,Q interleaved) at the
# 100 kHz raw rate (consumer maps across FULL_BW_HZ = 100 kHz). Decode + report
# RMS / DC / a coarse spectrum peak so live samples (and a carrier) are visible.
import math
FS_HZ = 100000.0
N_IQ  = 512                                     # fp32 complex count
def iq_unpack(fr):
    # The 0x1000-byte IQ region is either 512 fp32 complex (flow_fmt 0) or 1024
    # bf16 complex (flow_fmt 1 = the high 16 bits of each f32; rebuild via u16<<16).
    if fr[0x1018] == 1:                         # bf16 -> 1024 complex
        u16 = struct.unpack("<%dH" % (4 * N_IQ), fr[4:4 + 8 * N_IQ])
        vals = [struct.unpack("<f", struct.pack("<I", v << 16))[0] for v in u16]
    else:                                       # fp32 -> 512 complex
        vals = list(struct.unpack("<%df" % (2 * N_IQ), fr[4:4 + 8 * N_IQ]))
    return vals[0::2], vals[1::2]               # I, Q
def iq_rms(fr):
    I, Q = iq_unpack(fr)
    return (sum(i * i + q * q for i, q in zip(I, Q)) / len(I)) ** 0.5
def iq_stats(fr, fs=FS_HZ):
    I, Q = iq_unpack(fr); n_iq = len(I)
    nz  = sum(1 for v in I if v != 0.0) + sum(1 for v in Q if v != 0.0)
    dcI = sum(I) / n_iq; dcQ = sum(Q) / n_iq
    rms = (sum(i * i + q * q for i, q in zip(I, Q)) / n_iq) ** 0.5
    # Coarse DFT peak: correlate the block against NB test tones spanning
    # -fs/2..+fs/2 -> rough carrier offset from the LO (negative = below). fs is
    # the effective rate = 100 kHz / 2^fft_dec, so the span narrows at zoom.
    NB = 64
    best_hz, best_p = 0.0, -1.0
    for kb in range(NB):
        wk = 2.0 * math.pi * (kb - NB // 2) / NB  # normalized per-sample phase
        re = im = 0.0
        for n in range(n_iq):                   # x*exp(-j wk n), x = I + jQ
            c = math.cos(wk * n); s = math.sin(wk * n)
            re += I[n] * c + Q[n] * s
            im += Q[n] * c - I[n] * s
        p = re * re + im * im
        if p > best_p:
            best_p, best_hz = p, (kb - NB // 2) * fs / NB
    return nz, rms, dcI, dcQ, best_hz
_fd0 = (struct.unpack_from("<H", frames[0], 0x101A)[0] >> 1) & 0x7   # fft_dec
_fs0 = FS_HZ / (1 << _fd0)
_ns0 = N_IQ * (2 if frames[0][0x1018] == 1 else 1)                   # 512 or 1024
nz, rms, dcI, dcQ, peak_hz = iq_stats(frames[0], _fs0)
print("IQ payload: %d/%d nonzero (%s %d cplx)  RMS=%.3e  DC=(%.3e, %.3e)  peak~%+.1f kHz (span %g kHz)"
      % (nz, 2 * _ns0, "bf16" if frames[0][0x1018] == 1 else "fp32", _ns0,
         rms, dcI, dcQ, peak_hz / 1000.0, _fs0 / 1000.0))

# flow_info (rev>=8), 12 bytes at 0x1014: lo_freq u32, flow_fmt u8, {seq_n:4,
# seq_total:4} u8, then a 16-bit bitfield {vary_freq:1, fft_dec:3,
# audio_in_lvl_db:7, _pad1:5}. lo_freq != 0 confirms the GUI gets absolute
# panadapter positioning (base rev>=8).
def flow_info(fr):
    lo = struct.unpack_from("<I", fr, 0x1014)[0]
    fmt = fr[0x1018]
    seq = fr[0x1019]; seq_n = seq & 0xF; seq_total = (seq >> 4) & 0xF
    bits = struct.unpack_from("<H", fr, 0x101A)[0]
    vary = bits & 1; fft_dec = (bits >> 1) & 0x7; aud = (bits >> 4) & 0x7F
    return lo, fmt, seq_n, seq_total, vary, fft_dec, aud
lo, fmt, sn, st, vary, fftd, aud = flow_info(frames[0])
print("flow_info: lo_freq=%d Hz  fmt=%s  seq=%d/%d  vary=%d  fft_dec=%d(span=%d kHz)  audio_lvl=%ddB"
      % (lo, "fp32" if fmt == 0 else ("bf16" if fmt == 1 else "?%d" % fmt),
         sn, st, vary, fftd, 100 >> fftd, aud - 127))
if lo == 0:
    print("  -> lo_freq=0: GUI treats as OEM (no absolute positioning). Expected nonzero for rev>=8.")
if nz == 0:
    print("  -> IQ ALL ZERO: DSP not filling the flow ring (no RX / rx_flow_iq_copy not wired).")
else:
    # Per-frame RMS spread confirms the block is live (changes frame to frame).
    rmss = [iq_rms(fr) for fr in frames[:min(8, len(frames))]]
    print("  -> IQ LIVE: RMS over first %d frames min=%.3e max=%.3e (varies = real samples)"
          % (len(rmss), min(rmss), max(rmss)))
if t_first is not None and t_last is not None and len(frames) > 1:
    avg = (t_last - t_first) / (len(frames) - 1) * 1000.0
    print("avg cadence: %.1f ms over %d frames (firmware target ~35 ms)" % (avg, len(frames)))
