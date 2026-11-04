#-------------------------------------------------------------------------------
#
#  Karillon Corporation
#  Colorado Springs, Colorado
#
#  Copyright (c) 2026 by Karillon Corp.
#  Colorado Springs, Colorado. USA
#
#  This code is the property of Karillon Corporation (Karillon) and may not
#  be redistributed in any form without prior written permission from Karillon
#
#  Purpose: Send file data to the Newport DUTs over their JTAG ports.
#
#------------------------------------------------------------------------------
"""
newport_jtag.py -- JTAG to the 9 Newport DUTs through the FPGA (2026-10-01).

The FPGA (fpga/newport_eval/jtag_engine.v) only moves bits: shared TCK and
TRST, its own TMS/TDI/TDO per chip, a 4096-byte TDI buffer with expected-
TDO and mask buffers for comparing. Everything Newport-specific lives here:
the TAP state walk, the instruction register length and codes, TRST
polarity and the TCK rate. Those are SETTINGS, read from cfg\\cfg_jtag.py
(defaults below if it's missing) -- the real Newport values aren't known yet.

Chips: index i = 3*Y + X for chip (X,Y). Chip specs accept "all", indices
and ranges ("0-3,7"), and coordinates ("(1,1)" or "1:1").

Sending a file (send_file / send_data):
    1. IR scan of the load instruction on the selected chips
    2. Shift-DR with the file's bits, 32768 bits (4096 bytes) per chunk,
       staying in Shift-DR between chunks, leaving on the last bit
    3. Update-DR, back to Run-Test/Idle (+ RUNTEST_CLOCKS idle clocks)
All selected chips get the same data at the same time. With an expected
file (and optional mask), every chip's TDO is compared bit by bit during the
shift and each chip gets its own mismatch count and first bad bit. With
readback=True, a second pass shifts the same data again and expects what the
first pass wrote -- only meaningful if the register captures its own
contents.

Bit order: bit k of the DR is byte k//8, bit k%8 of the file (LSB of byte 0
goes in first). Use msb_first=True for files with the other order within
each byte.

No BYPASS: Newport's TAP has none, and nothing here relies on it. Every
chip has its own TDI/TDO, so nothing is daisy-chained. Instructions are
only ever the codes you configure -- never padded with 1s.

Unselected chips: TCK reaches all 9. The FPGA holds TMS low on chips that
aren't selected, which keeps them in Run-Test/Idle (where every operation
here starts and ends), so they're left alone.

Menu entries for Karillon_TestExec: jtag_idcode, jtag_send_file.

Bench use:
    python newport_jtag.py --idcode                         IDCODE of all 9
    python newport_jtag.py --file img.bin --chips all       send to all 9
    python newport_jtag.py --file img.bin --chips 0-3 --expect exp.bin
    python newport_jtag.py --file img.bin --readback        write, then verify
    python newport_jtag.py --debug ...                      simulated FPGA
    python newport_jtag.py --selftest                       offline checks
"""

import os
import re
import sys
import time

from newport_link import NewportError, JTAG_BUF_BYTES

NCHIP = 9
ALL_CHIPS = (1 << NCHIP) - 1
CLK_HZ = 250e6                  # FPGA clk250
CHUNK_BITS = JTAG_BUF_BYTES * 8

# ---------------------------------------------------------------------------
# Settings (cfg\cfg_jtag.py overrides these -- PLACEHOLDERS until the real
# Newport TAP spec is in hand)
# ---------------------------------------------------------------------------
_DEFAULTS = {
    "IR_LEN": 5,                         # instruction register length, bits
    "INSTRUCTIONS": {                    # name -> IR code
        "IDCODE": 0x01,
        "LOAD":   0x03,
    },
    "LOAD_INSTR": "LOAD",                # instruction whose DR takes file data
    "IDCODE_INSTR": "IDCODE",            # None if Newport has no IDCODE
    "IDCODE_EXPECT": None,               # e.g. 0x12345001; None = don't check
    "TRST_ACTIVE_LOW": True,             # IEEE 1149.1 TRST* is active low
    "TCK_HZ": 1_000_000,                 # 3.8 kHz .. 31.25 MHz
    "RUNTEST_CLOCKS": 0,                 # idle TCKs after each Update-DR
}
CFG = dict(_DEFAULTS)
CFG_SOURCE = "built-in defaults"


def _find_cfg():
    try:
        import cfg_jtag
        return cfg_jtag
    except ImportError:
        pass
    d = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "cfg"))
    if os.path.isfile(os.path.join(d, "cfg_jtag.py")):
        sys.path.insert(0, d)
        import cfg_jtag
        return cfg_jtag
    return None


def apply_cfg(module=None):
    """Load settings from module (default: cfg_jtag if found). Returns the
    source description, also kept in CFG_SOURCE."""
    global CFG, CFG_SOURCE
    CFG = {k: getattr(module, k, v) for k, v in _DEFAULTS.items()} if module else dict(_DEFAULTS)
    CFG["INSTRUCTIONS"] = dict(CFG["INSTRUCTIONS"])
    CFG_SOURCE = getattr(module, "__file__", "cfg_jtag") if module else "built-in defaults"
    return CFG_SOURCE


apply_cfg(_find_cfg())


# ---------------------------------------------------------------------------
# Chips
# ---------------------------------------------------------------------------
def chip_name(i):
    return "(%d,%d)" % (i % 3, i // 3)


def chips_of(mask):
    return [i for i in range(NCHIP) if mask >> i & 1]


def parse_chips(spec):
    """'all' | '0-3,7' | '(1,1) (2,2)' | '1:1' | iterable of ints -> mask."""
    if isinstance(spec, int):
        return spec
    if not isinstance(spec, str):
        m = 0
        for i in spec:
            if not 0 <= int(i) < NCHIP:
                raise ValueError("chip index must be 0..8")
            m |= 1 << int(i)
        return m
    text = spec.strip().lower()
    if text in ("all", "*", ""):
        return ALL_CHIPS
    mask = 0
    for x, y in re.findall(r"\(\s*([0-2])\s*,\s*([0-2])\s*\)", text):
        mask |= 1 << (3 * int(y) + int(x))
    text = re.sub(r"\(\s*[0-2]\s*,\s*[0-2]\s*\)", " ", text)
    for tok in re.split(r"[\s,]+", text.strip()):
        if not tok:
            continue
        m = re.fullmatch(r"([0-2]):([0-2])", tok)
        if m:
            mask |= 1 << (3 * int(m.group(2)) + int(m.group(1)))
            continue
        m = re.fullmatch(r"([0-8])(?:-([0-8]))?", tok)
        if not m:
            raise ValueError("bad chip spec %r (all, 0-8, 0-3, (X,Y) or X:Y)" % tok)
        a = int(m.group(1)); b = int(m.group(2)) if m.group(2) else a
        for i in range(min(a, b), max(a, b) + 1):
            mask |= 1 << i
    if not mask:
        raise ValueError("no chips selected")
    return mask


# ---------------------------------------------------------------------------
# Files / bits
# ---------------------------------------------------------------------------
def _rev8(b):
    return int("{:08b}".format(b)[::-1], 2)


def read_data_file(path, msb_first=False):
    """Bytes from a file: .bin = raw; anything else = hex text (2-digit
    bytes, any whitespace/commas, '#' comments, optional 0x)."""
    with open(path, "rb") as f:
        raw = f.read()
    if os.path.splitext(path)[1].lower() != ".bin":
        text = re.sub(r"#[^\n]*", "", raw.decode("ascii", errors="replace"))
        toks = [t for t in re.split(r"[\s,]+", text) if t]
        try:
            raw = bytes(int(t[2:] if t.lower().startswith("0x") else t, 16) for t in toks)
        except ValueError:
            raise ValueError("%s: not hex bytes" % path)
    if msb_first:
        raw = bytes(_rev8(b) for b in raw)
    return raw


def fit_bits(data, nbits, pad=0x00):
    """data trimmed/padded to exactly ceil(nbits/8) bytes, unused bits of the
    last byte cleared."""
    nbytes = (nbits + 7) // 8
    data = bytes(data[:nbytes]) + bytes([pad]) * max(0, nbytes - len(data))
    if nbits % 8:
        data = data[:-1] + bytes([data[-1] & ((1 << (nbits % 8)) - 1)])
    return data


# ---------------------------------------------------------------------------
# The JTAG layer
# ---------------------------------------------------------------------------
class NewportJtag:
    """TAP-level access to the Newport DUTs through a NewportLink."""

    def __init__(self, link, cfg=None):
        self.link = link
        self.cfg = dict(cfg or CFG)
        self.mask = None
        self.half = None

    # -- setup ----------------------------------------------------------------
    def set_tck(self, hz):
        half = int(round(CLK_HZ / (2.0 * hz)))
        half = min(max(half, 4), 0xFFFF)
        self.link.jclk(half)
        self.half = half
        return CLK_HZ / (2.0 * half)

    def select(self, chips):
        mask = parse_chips(chips)
        if mask != self.mask:
            self.link.jsel(mask)
            self.mask = mask
        return mask

    def trst(self, asserted):
        low = self.cfg["TRST_ACTIVE_LOW"]
        self.link.jrst(0 if (asserted == low) else 1)

    def start(self, chips="all"):
        """TCK rate, TRST pulse, TMS reset of all 9 chips, all left in
        Run-Test/Idle, then select `chips`."""
        self.set_tck(self.cfg["TCK_HZ"])
        self.select(ALL_CHIPS)
        self.trst(True)
        time.sleep(0.001)
        self.trst(False)
        self.reset()
        self.select(chips)

    def reset(self):
        """5 x TMS=1 (Test-Logic-Reset) then TMS=0 (Run-Test/Idle) on the
        selected chips."""
        self.link.jtms(6, 0x1F)

    def idle(self, clocks):
        while clocks > 0:
            n = min(clocks, 8)
            self.link.jtms(n, 0x00)
            clocks -= n

    # -- scans (all start and end in Run-Test/Idle) -----------------------------
    def _shift_long(self, nbits, data, expect=None, mask=None):
        """In Shift-xR: shift nbits of data in chunks; leaves on the last bit.
        Returns {chip: [count, first]} for chips that mismatched."""
        bad = {}
        off = 0
        data = fit_bits(data, nbits)
        if expect is not None:
            expect = fit_bits(expect, nbits)
            mask = fit_bits(mask if mask is not None else b"\xFF" * len(expect), nbits)
        while off < nbits:
            n = min(CHUNK_BITS, nbits - off)
            b0, b1 = off // 8, (off + n + 7) // 8
            self.link.jld("D", data[b0:b1])
            if expect is not None:
                self.link.jld("E", expect[b0:b1])
                self.link.jld("M", mask[b0:b1])
            last = off + n == nbits
            fm = self.link.jshf(n, last, expect is not None)
            for c in chips_of(fm):
                cnt, first = self.link.jerr(c)
                e = bad.setdefault(c, [0, None])
                e[0] += cnt
                if e[1] is None and first is not None:
                    e[1] = off + first
            off += n
        return bad

    def ir_scan(self, code):
        """Load an instruction on the selected chips. Returns what the
        capture chip shifted out (Capture-IR value) when IR_LEN <= 32."""
        irlen = self.cfg["IR_LEN"]
        self.link.jtms(4, 0x03)                 # RTI -> Select-DR -> Select-IR -> Capture-IR -> Shift-IR
        if irlen <= 32:
            cap = self.link.jsi(irlen, code, True)
        else:
            self._shift_long(irlen, code.to_bytes((irlen + 7) // 8, "little"))
            cap = None
        self.link.jtms(2, 0x01)                 # Exit1-IR -> Update-IR -> RTI
        return cap

    def instr(self, name):
        try:
            return self.cfg["INSTRUCTIONS"][name]
        except KeyError:
            raise ValueError("instruction %r isn't in cfg_jtag INSTRUCTIONS (%s)"
                             % (name, ", ".join(self.cfg["INSTRUCTIONS"])))

    def dr_scan_short(self, nbits, value=0, chip=None):
        """DR scan of up to 32 bits; returns the TDO of `chip` (default: the
        lowest selected chip)."""
        if chip is None:
            chip = chips_of(self.mask or 1)[0]
        self.link.jcap(chip)
        self.link.jtms(3, 0x01)                 # RTI -> Select-DR -> Capture-DR -> Shift-DR
        cap = self.link.jsi(nbits, value, True)
        self.link.jtms(2, 0x01)                 # Exit1-DR -> Update-DR -> RTI
        return cap

    def dr_scan_long(self, nbits, data, expect=None, mask=None):
        """DR scan of any length from bytes; optional TDO compare. Returns
        {chip: [mismatches, first bad bit]} for chips that mismatched."""
        self.link.jtms(3, 0x01)
        bad = self._shift_long(nbits, data, expect, mask)
        self.link.jtms(2, 0x01)
        self.idle(self.cfg["RUNTEST_CLOCKS"])
        return bad

    # -- what the menu / CLI use ------------------------------------------------
    def read_idcodes(self, chips="all"):
        """{chip: IDCODE} for each chip."""
        name = self.cfg["IDCODE_INSTR"]
        if not name:
            raise ValueError("no IDCODE instruction configured (cfg_jtag IDCODE_INSTR)")
        mask = self.select(chips)
        self.ir_scan(self.instr(name))
        return {c: self.dr_scan_short(32, 0, c) for c in chips_of(mask)}

    def send_data(self, data, chips="all", instr=None, nbits=None, expect=None,
                  mask=None, readback=False):
        """Load `instr` (default LOAD_INSTR), then shift data into its DR on
        every selected chip. Returns a result dict (see summarize())."""
        data = bytes(data)
        nbits = nbits or len(data) * 8
        sel = self.select(chips)
        name = instr or self.cfg["LOAD_INSTR"]
        self.ir_scan(self.instr(name))
        t0 = time.monotonic()
        bad = self.dr_scan_long(nbits, data, expect, mask)
        res = {"chips": chips_of(sel), "bits": nbits, "instr": name,
               "compared": expect is not None, "bad": bad, "readback": None,
               "seconds": time.monotonic() - t0}
        if readback:
            res["readback"] = self.dr_scan_long(nbits, data, data, None)
        return res


def summarize(res):
    """Lines of text and overall pass for a send_data() result."""
    lines, ok = [], True
    lines.append("%s: %d bits to chips %s in %.2f s"
                 % (res["instr"], res["bits"], " ".join(chip_name(c) for c in res["chips"]),
                    res["seconds"]))
    for label, bad, on in (("compare", res["bad"], res["compared"]),
                           ("readback", res["readback"], res["readback"] is not None)):
        if not on:
            continue
        for c in res["chips"]:
            if c in bad:
                ok = False
                cnt, first = bad[c]
                lines.append("  chip %d %s %s: FAIL, %d bits differ, first at bit %s"
                             % (c, chip_name(c), label, cnt, first))
            else:
                lines.append("  chip %d %s %s: match" % (c, chip_name(c), label))
    if not res["compared"] and res["readback"] is None:
        lines.append("  (no compare: sent only)")
    return lines, ok


# ---------------------------------------------------------------------------
# Menu entries
# ---------------------------------------------------------------------------
def _ask(prompt, default=""):
    try:
        ans = input("%s%s: " % (prompt, " [%s]" % default if default else "")).strip()
    except EOFError:
        ans = ""
    return ans or default


def jtag_idcode(serial_number):
    """Read IDCODE from all 9 chips (vs IDCODE_EXPECT if configured)."""
    import newport_tests as nt

    def body(link, sn):
        j = NewportJtag(link)
        j.start("all")
        ids = j.read_idcodes("all")
        want = j.cfg["IDCODE_EXPECT"]
        ok = True
        for c, v in sorted(ids.items()):
            if want is not None:
                good = v == want
            else:
                good = v not in (0, 0xFFFFFFFF)       # something answered
            ok &= good
            print("chip %d %s IDCODE %08X  %s" % (c, chip_name(c), v, "ok" if good else
                  ("expected %08X" % want if want is not None else "no TAP answering?")))
        print("cfg: %s" % CFG_SOURCE)
        return 0 if ok else 1
    return nt._with_link(body, serial_number, power=True)


def jtag_send_file(serial_number):
    """Ask for a file, chips and an optional expected file; send and report."""
    import newport_tests as nt

    def body(link, sn):
        path = _ask("Data file (.bin raw, otherwise hex text)")
        if not path:
            print("no file - nothing sent")
            return 1
        data = read_data_file(path.strip('"'))
        chips = parse_chips(_ask("Chips (all, 0-8, 0-3, (X,Y))", "all"))
        instr = _ask("Instruction", CFG["LOAD_INSTR"])
        exp = _ask("Expected-TDO file (Enter = none, 'readback' = write then verify)")
        expect = mask = None
        readback = exp.lower() == "readback"
        if exp and not readback:
            expect = read_data_file(exp.strip('"'))
            m = _ask("Mask file (Enter = compare every bit)")
            mask = read_data_file(m.strip('"')) if m else None
        j = NewportJtag(link)
        j.start(chips)
        res = j.send_data(data, chips, instr, expect=expect, mask=mask, readback=readback)
        lines, ok = summarize(res)
        for line in lines:
            print(line)
        return 0 if ok else 1
    return nt._with_link(body, serial_number, power=True)


# ---------------------------------------------------------------------------
# Self-test (simulated FPGA with 9 TAP models)
# ---------------------------------------------------------------------------
def selftest():
    import tempfile
    from newport_link import NewportLink
    from newport_sim import SimFPGA
    fails = 0

    def check(what, got, want):
        nonlocal fails
        ok = got == want
        fails += not ok
        short = lambda v: (lambda r: r if len(r) <= 60 else r[:57] + "...")(repr(v))
        print("%s  %-46s %s%s" % ("pass" if ok else "FAIL", what, short(got),
                                  "" if ok else "  (want %s)" % short(want)))

    sim = SimFPGA()
    link = NewportLink("SIM", ser=sim)
    j = NewportJtag(link, dict(_DEFAULTS))

    check("parse all", parse_chips("all"), 0x1FF)
    check("parse ranges", parse_chips("0-2, 7"), 0b010000111)
    check("parse coords", parse_chips("(1,1) 2:2"), (1 << 4) | (1 << 8))
    check("fit_bits trims last byte", fit_bits(b"\xFF\xFF", 12), b"\xFF\x0F")
    check("TCK 1 MHz -> half 125", (round(j.set_tck(1e6)), j.half), (1000000, 125))

    j.start("all")
    check("all TAPs in Run-Test/Idle after start", [t.state for t in sim.taps], ["RTI"] * 9)
    check("IR capture value", j.ir_scan(j.instr("IDCODE")), 0b00001)
    ids = j.read_idcodes("all")
    check("IDCODE from each chip", ids, {c: sim.taps[c].idcode for c in range(9)})

    data = bytes((i * 37 + 11) & 0xFF for i in range(5000))         # 40000 bits: 2 chunks
    res = j.send_data(data, "all")
    check("40000 bits into all 9", [t.mem[:40000] == [(data[k // 8] >> (k % 8)) & 1 for k in range(40000)]
                                     for t in sim.taps], [True] * 9)
    check("summary sent only", summarize(res)[1], True)

    sim.taps[6].mem[123] ^= 1
    res = j.send_data(data, "all", expect=data)                   # second pass returns pass-1 data
    lines, ok = summarize(res)
    check("compare flags chip 6 at bit 123", (ok, res["bad"]), (False, {6: [1, 123]}))

    mask = bytearray(b"\xFF" * len(data)); mask[123 // 8] &= ~(1 << (123 % 8)) & 0xFF
    sim.taps[6].mem[123] ^= 1
    res = j.send_data(data, "all", expect=data, mask=bytes(mask))
    check("masked bit not counted", res["bad"], {})

    other = bytes(b ^ 0xFF for b in data)
    res = j.send_data(other, "0-3", readback=True)
    check("readback pass on chips 0-3", (summarize(res)[1], res["readback"]), (True, {}))
    check("chips 4-8 untouched and idle",
          ([t.mem[:16] == [(data[k // 8] >> (k % 8)) & 1 for k in range(16)] for t in sim.taps[4:]],
           [t.state for t in sim.taps[4:]]), ([True] * 5, ["RTI"] * 5))
    check("no all-ones IR ever loaded", sum(t.bypass_seen for t in sim.taps), 0)

    try:
        j.instr("NOPE"); got = "no error"
    except ValueError:
        got = "ValueError"
    check("unknown instruction", got, "ValueError")

    with tempfile.TemporaryDirectory() as tmp:
        p = os.path.join(tmp, "d.hex")
        with open(p, "w") as f:
            f.write("# test\n0x01 02, ff\n")
        check("hex file", read_data_file(p), b"\x01\x02\xFF")
        check("msb_first", read_data_file(p, msb_first=True), b"\x80\x40\xFF")

    print("SELFTEST", "PASS" if fails == 0 else "FAIL (%d)" % fails)
    return fails == 0


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------
def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="JTAG to the Newport DUTs through the FPGA")
    ap.add_argument("--file", help="data for the DR (.bin raw, otherwise hex text)")
    ap.add_argument("--chips", default="all", help="all, 0-8, 0-3,7, (X,Y) or X:Y (default all)")
    ap.add_argument("--instr", default=None, help="instruction name from cfg_jtag (default LOAD_INSTR)")
    ap.add_argument("--bits", type=int, default=None, help="DR length (default: 8 x file bytes)")
    ap.add_argument("--expect", help="expected TDO file to compare against")
    ap.add_argument("--mask", help="compare mask file (1 = compare)")
    ap.add_argument("--readback", action="store_true", help="shift again and expect what was written")
    ap.add_argument("--msb-first", action="store_true", help="bit 7 of each byte goes in first")
    ap.add_argument("--tck", type=float, default=None, help="TCK in Hz (default cfg_jtag TCK_HZ)")
    ap.add_argument("--idcode", action="store_true", help="read IDCODE from the chips")
    ap.add_argument("--port", default=None, help="serial port (default: cfg_com.NEWPORT_port or COM3)")
    ap.add_argument("--debug", action="store_true", help="use the simulated FPGA")
    ap.add_argument("--selftest", action="store_true", help="offline checks of this module")
    a = ap.parse_args(argv)
    if a.selftest:
        return 0 if selftest() else 1
    if not (a.file or a.idcode):
        ap.error("nothing to do: give --file and/or --idcode")
    if a.debug:
        os.environ["K_DEBUG"] = "1"
        try:
            import K_Sim
            K_Sim.set_debug(True)
        except ImportError:
            pass
    import newport_tests as nt
    if a.port:
        nt.set_port(a.port)
    link = nt.get_link()
    if link is None:
        print("Newport FPGA not connected")
        return 1
    rc = 0
    try:
        cfg = dict(CFG)
        if a.tck:
            cfg["TCK_HZ"] = a.tck
        j = NewportJtag(link, cfg)
        j.start(a.chips)
        print("TCK %.0f Hz, IR length %d, cfg: %s" % (CLK_HZ / (2 * j.half), cfg["IR_LEN"], CFG_SOURCE))
        if a.idcode:
            for c, v in sorted(j.read_idcodes(a.chips).items()):
                print("chip %d %s IDCODE %08X" % (c, chip_name(c), v))
        if a.file:
            data = read_data_file(a.file, a.msb_first)
            exp = read_data_file(a.expect, a.msb_first) if a.expect else None
            msk = read_data_file(a.mask, a.msb_first) if a.mask else None
            res = j.send_data(data, a.chips, a.instr, a.bits, exp, msk, a.readback)
            lines, ok = summarize(res)
            print("\n".join(lines))
            rc = 0 if ok else 1
    except (NewportError, ValueError, OSError) as e:
        print("Error: %s" % e)
        rc = 1
    finally:
        nt.close_link()
    return rc


if __name__ == "__main__":
    sys.exit(main())
