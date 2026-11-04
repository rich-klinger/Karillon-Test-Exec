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
#  Purpose: Simulated Newport FPGA for debug mode.
#
#------------------------------------------------------------------------------
"""
newport_sim.py -- a stand-in for the Newport FPGA's UART, used in debug mode
(Karillon_TestExec --debug, or K_DEBUG=1) and by the offline self-tests.

SimFPGA looks like a pyserial port (write / read / in_waiting / close) and
behaves like fpga/newport_eval/cmd_interp.v plus regfile_tests.v and
done_notifier.v:

  - every character is echoed; a CR or LF ends the line; an empty line is
    only echoed (no response), anything unrecognised gets ERR
  - SET / GET TESTn_CFG / GET TESTn_RES / START / GET AFECFG / I2C, with the
    same exact-format rules (case-sensitive keywords, single spaces, 1-2
    digit test numbers 0-19, hex digits either case)
  - a 0->1 edge on config bit 15 starts the test: it finishes at once, its
    result register is loaded and "DONE TESTn" is sent after the reply
  - the I2C command talks to simulated MPQ8785s at the addresses in
    fpga_pmbus.REGULATORS
  - TIME HHHHHHHHHHHH (12 hex digits, ms since 1970 UTC) sets time_ms and
    answers OK; with time_supported = False it answers ERR like a bitstream
    that predates the command
  - PKL p / PKR p oo against 8 x 256-byte packet slots (self.pkt): after
    PKL's OK the next 256 bytes are taken without echo and answered with
    their CRC (binascii.crc_hqx(data, 0xFFFF)); pkt_busy = True makes PKL
    answer BSY, pkl_corrupt = True flips a bit in the next load (the CRC
    then disagrees), pkl_stall = n takes n bytes then answers TO
  - TEST9 (packet send) records each run's config in test9_runs and the
    bytes it would put on the wire (slots in order, all passes) in
    test9_sent; cfg bit 13 picks the GPIO path, bits 12:6 the extra passes
  - JTAG (JSEL/JCAP/JCLK/JRST/JTMS/JSI/JLD/JSHF/JERR) driving 9 TAP models
    (self.taps, SimTap) with no BYPASS, IR length and IDCODE/LOAD codes from
    newport_jtag.CFG; the LOAD register gives back on the next scan what the
    last one wrote (self.taps[c].mem, a list of bits)
  - with ask_time_on_start, starting a test sends "?TIME" before its DONE
    (the FPGA asks for the time when a test starts); request_time() sends
    one at any moment, and inject_before_echo puts text in front of the
    next echoed character (an unsolicited line arriving mid-command)

Default results are what a healthy board would report (TEST0-6 pass, TEST7
a plausible readback, TEST8 done, TEST9 pass, reserved slots 0xBFFF). To see how a test
script handles a failure, override a slot before running it:

    sim.set_result(3, 0xA00B)        # TEST3: done, timeout, mismatches...
    sim.set_result(3, None)          # back to the default
    sim.no_done.add(0)               # TEST0 never sends DONE (host must time out)
    sim.ask_time_on_start = True     # every START also sends ?TIME
    sim.time_supported = False       # TIME answers ERR (older bitstream)
"""

import binascii
import collections
import re

# Default result registers, per the layouts in fpga/newport_eval/README.md.
#   TEST0-5: done | PASS | SOF ok | EOF ok | mismatches=0 | id
#   TEST6:   done | PASS | mismatches=0 | id 6
#   TEST7:   done | sample nibble 1100b | 9 samples | id 7
#   TEST8:   done | id 8
#   TEST9:   done | PASS | id 9 (packet send, 4-bit id)
#   TEST10-19 reserved: 0xBFFF ("not implemented")
DEFAULT_RESULTS = {n: 0x8000 | 0x4000 | 0x1000 | 0x0800 | n for n in range(6)}
DEFAULT_RESULTS[6] = 0x8000 | 0x4000 | 6
DEFAULT_RESULTS[7] = 0x8000 | (0b1100 << 9) | (9 << 3) | 7
DEFAULT_RESULTS[8] = 0x8000 | 8
DEFAULT_RESULTS[9] = 0x8000 | 0x4000 | 9
for _n in range(10, 20):
    DEFAULT_RESULTS[_n] = 0xBFFF

SIM_AFECFG = 0x000000000000

_TESTN = r"TEST(1[0-9]|[0-9])"       # 0-19, 1 or 2 digits, no leading zero
_HEX = r"[0-9A-Fa-f]"


class SimTap:
    """IEEE 1149.1 TAP without a BYPASS register. TRST active low unless
    trst_active_low=False. Instructions: IDCODE (32-bit), LOAD (returns on
    the next scan what the last one wrote, any length); other codes select
    a 1-bit register. An all-ones IR update counts in bypass_seen."""
    NEXT = {"TLR": ("RTI", "TLR"), "RTI": ("RTI", "SELDR"), "SELDR": ("CAPDR", "SELIR"),
            "CAPDR": ("SHDR", "EX1DR"), "SHDR": ("SHDR", "EX1DR"), "EX1DR": ("PAUDR", "UPDDR"),
            "PAUDR": ("PAUDR", "EX2DR"), "EX2DR": ("SHDR", "UPDDR"), "UPDDR": ("RTI", "SELDR"),
            "SELIR": ("CAPIR", "TLR"), "CAPIR": ("SHIR", "EX1IR"), "SHIR": ("SHIR", "EX1IR"),
            "EX1IR": ("PAUIR", "UPDIR"), "PAUIR": ("PAUIR", "EX2IR"), "EX2IR": ("SHIR", "UPDIR"),
            "UPDIR": ("RTI", "SELDR")}

    def __init__(self, irlen, idcode, idcode_ir, load_ir):
        self.irlen, self.idcode = irlen, idcode
        self.idcode_ir, self.load_ir = idcode_ir, load_ir
        self.state = "TLR"
        self.ir = idcode_ir if idcode_ir is not None else 0
        self.ir_sh = collections.deque()
        self.dr_sh = collections.deque()
        self.mem = []
        self.tdo = 1
        self.bypass_seen = 0
        self.updates_dr = 0
        self.old_left = 0

    def reset(self):
        self.state = "TLR"
        self.ir = self.idcode_ir if self.idcode_ir is not None else 0

    def clock(self, tms, tdi):
        """One TCK: returns TDO as sampled before the rising edge."""
        out = self.tdo
        s = self.state
        if s == "CAPIR":
            self.ir_sh = collections.deque([1, 0] + [0] * (self.irlen - 2))
        elif s == "SHIR":
            self.ir_sh.popleft(); self.ir_sh.append(tdi)
        elif s == "CAPDR":
            if self.ir == self.idcode_ir:
                self.dr_sh = collections.deque((self.idcode >> k) & 1 for k in range(32))
            elif self.ir == self.load_ir:
                self.dr_sh = collections.deque(self.mem)
                self.old_left = len(self.mem)   # captured bits still to come out
            else:
                self.dr_sh = collections.deque([0])
        elif s == "SHDR":
            if self.ir == self.load_ir:      # grows to the longest scan so far
                if self.old_left:
                    self.dr_sh.popleft()
                    self.old_left -= 1
                self.dr_sh.append(tdi)
            else:
                self.dr_sh.popleft(); self.dr_sh.append(tdi)
        self.state = self.NEXT[s][tms]
        if self.state == "TLR":
            self.reset()
        # falling edge
        n = self.state
        if n == "SHIR":
            self.tdo = self.ir_sh[0]
        elif n == "SHDR":
            if self.ir == self.load_ir:
                self.tdo = self.dr_sh[0] if self.old_left else 0
            else:
                self.tdo = self.dr_sh[0]
        else:
            self.tdo = 1
            if n == "UPDIR":
                self.ir = sum(b << k for k, b in enumerate(self.ir_sh))
                if self.ir == (1 << self.irlen) - 1:
                    self.bypass_seen += 1
            elif n == "UPDDR" and self.ir == self.load_ir:
                self.mem = list(self.dr_sh)
                self.updates_dr += 1
        return out


class SimFPGA:
    """Byte-level emulation of newport_eval's UART side."""

    def __init__(self, port="SIM", regulators=None, power_wiring=True):
        self.port = port
        self.name = str(port) + " (SIM)"
        self.is_open = True
        self.cfg = [0] * 20
        self.res = [0] * 20
        self.overrides = {}
        self.no_done = set()
        self.lines_seen = []
        self._line = ""
        self._out = bytearray()
        self._pending_done = []
        self.time_supported = True
        self.time_ms = None               # last TIME received
        self.time_set_count = 0
        self.ask_time_on_start = False
        self.time_requests_sent = 0
        self.inject_before_echo = ""
        self.inject_on_char = None        # only before this character's echo
        self._pending_time_req = False
        # Packet RAM (pkt_ram.v) and the PKL binary receive
        self.pkt = [bytearray(256) for _ in range(8)]
        self.pkt_busy = False             # True: PKL answers BSY (TEST9 running)
        self.pkl_corrupt = False          # True: flip a bit in the next load
        self.pkl_stall = None             # n: take n bytes of the next load, then TO
        self._bin_slot = None             # slot being loaded (no echo while set)
        self._bin_buf = bytearray()
        self._pending_bin = None
        self._lost = False
        self.test9_runs = []              # config of every TEST9 start
        self.test9_sent = []              # (path, bytes on the wire) of every TEST9 start
        # JTAG engine + 9 TAPs (codes from newport_jtag's settings)
        from newport_jtag import CFG as _JCFG
        ins = _JCFG["INSTRUCTIONS"]
        idc = ins.get(_JCFG["IDCODE_INSTR"]) if _JCFG["IDCODE_INSTR"] else None
        self.trst_active_low = _JCFG["TRST_ACTIVE_LOW"]
        self.taps = [SimTap(_JCFG["IR_LEN"], 0x1000A001 | (c << 16), idc,
                            ins.get(_JCFG["LOAD_INSTR"])) for c in range(9)]
        self.j_sel, self.j_cap, self.j_half, self.j_trst = 0, 0, 125, 0
        self.jbuf = {b: bytearray(4096) for b in "DEM"}
        self.j_err = [(0, 0xFFFF)] * 9
        self.j_clocks = 0                 # total TCK clocks issued
        # Simulated PMBus regulators (from fpga_pmbus's offline model).
        from fpga_pmbus import _MockMPQ, REGULATORS
        addrs = REGULATORS.values() if regulators is None else regulators
        self.i2c_devices = {a: _MockMPQ(a) for a in addrs}
        # DUT power control (PWR / GET PWR, 2026-10-09; wiring 2026-10-11):
        # bit0 VDDC_ON, bit1 VDDC_RST, bit2 VDDIO_ON, bit3 VDDIO_RST, all 0
        # out of FPGA reset. ON drives the regulator's CTRL, RST its ALT/RST
        # (low = reset). VDDC is the regulator at REGULATORS["vddcore"],
        # VDDIO the one at REGULATORS["vddio"].
        self.pwr_out = 0
        self.power_wiring = power_wiring
        self.jtag_cable = False           # USB-Blaster on the JTAG header (proposed detect)
        self.cable_detect = True          # False: a bitstream without the detect pin
        self.fpga_interlock = True        # FPGA powers down + refuses ON while cable present
        self.pwr_log = []                 # (PWR line, POWER_GOOD bits before it)
        self._rails = {}
        if power_wiring:
            self._rails = {"vddcore": (REGULATORS.get("vddcore"), 0, 1),
                           "vddio": (REGULATORS.get("vddio"), 2, 3)}
        self._apply_pwr()

    # -- test hooks ------------------------------------------------------
    def set_result(self, n, value):
        """Force TESTn's result register (None = default)."""
        if value is None:
            self.overrides.pop(n, None)
        else:
            self.overrides[n] = value & 0xFFFF

    def _apply_pwr(self):
        for addr, on_bit, rst_bit in self._rails.values():
            dev = self.i2c_devices.get(addr)
            if dev is not None:
                dev.ctrl = bool(self.pwr_out >> on_bit & 1)
                dev.rst_low = not self.pwr_out >> rst_bit & 1

    def _pg_bits(self):
        bits = 0
        for k, name in enumerate(("vddcore", "vddio")):
            addr = self._rails.get(name, (None,))[0]
            dev = self.i2c_devices.get(addr)
            if dev is not None and dev.pg():
                bits |= 1 << k
        return bits

    def set_jtag_cable(self, present):
        """Plug a USB-Blaster into (True) / unplug it from (False) the
        FPGA's JTAG header, as the proposed detect pin would see it: the
        FPGA sends !JTAG 1 / !JTAG 0 and, on connect, turns VDDCORE then
        VDDIO off itself."""
        present = bool(present)
        if not self.cable_detect or present == self.jtag_cable:
            self.jtag_cable = present
            return
        self.jtag_cable = present
        if present and self.fpga_interlock:
            self.pwr_out &= ~0b0001                   # VDDC_ON off first
            self._apply_pwr()
            self.pwr_out &= ~0b0100                   # then VDDIO_ON
            self._apply_pwr()
        self._out += b"!JTAG 1\r\n" if present else b"!JTAG 0\r\n"

    def request_time(self):
        """Send an unsolicited ?TIME now."""
        self._out += b"?TIME\r\n"
        self.time_requests_sent += 1

    # -- pyserial-like interface -----------------------------------------
    @property
    def in_waiting(self):
        return len(self._out)

    def read(self, n=1):
        r = bytes(self._out[:n])
        del self._out[:n]
        return r

    def write(self, data):
        if isinstance(data, str):
            data = data.encode()
        self._lost = False
        for b in data:
            if self._lost:                                # rest of a stalled load never arrived
                continue
            if self._bin_slot is not None:                # PKL payload: no echo
                self._bin_byte(b)
                continue
            ch = chr(b)
            if self.inject_before_echo and self.inject_on_char in (None, ch):
                self._out += self.inject_before_echo.encode()   # unsolicited text first
                self.inject_before_echo = ""
            self._out += bytes([b])                       # echo
            if ch in "\r\n":
                if self._line:
                    self.lines_seen.append(self._line)
                    self._out += (self._exec(self._line) + "\r\n").encode()
                    self._flush_done()
                    if self._pending_bin is not None:     # PKL ... OK: binary from here
                        self._bin_slot, self._pending_bin = self._pending_bin, None
                        self._bin_buf = bytearray()
                self._line = ""
            elif len(self._line) < 32:                    # BUF_LEN
                self._line += ch
        return len(data)

    def _bin_store(self, data):
        tgt = self._bin_slot
        if isinstance(tgt, int):
            self.pkt[tgt][:len(data)] = data              # PKL slot
        else:
            self.jbuf[tgt[1]][:len(data)] = data          # JLD buffer

    def _bin_byte(self, b):
        self._bin_buf.append(b)
        tgt = self._bin_slot
        need = 256 if isinstance(tgt, int) else tgt[2]
        if self.pkl_stall is not None and len(self._bin_buf) >= self.pkl_stall:
            self.pkl_stall = None                         # FPGA gives up after 100 ms
            self._lost = True
            self._out += b"TO\r\n"
            self._bin_store(self._bin_buf)
            self._bin_slot = None
            return
        if len(self._bin_buf) == need:
            data = bytearray(self._bin_buf)
            if self.pkl_corrupt:
                self.pkl_corrupt = False
                data[min(17, need - 1)] ^= 0x04
            self._bin_store(data)
            self._out += ("%04X\r\n" % binascii.crc_hqx(bytes(data), 0xFFFF)).encode()
            self._bin_slot = None

    def reset_input_buffer(self):
        self._out.clear()

    def flush(self):
        pass

    def close(self):
        self.is_open = False

    # -- command interpreter -----------------------------------------------
    def _write_cfg(self, n, value):
        rising = (value & 0x8000) and not (self.cfg[n] & 0x8000)
        self.cfg[n] = value
        if rising:
            if n == 9:
                self.test9_runs.append(value & 0x7FFF)
                first, count = value & 7, ((value >> 3) & 7) + 1
                passes = ((value >> 6) & 0x7F) + 1
                data = b"".join(bytes(self.pkt[(first + i) % 8])
                                for _ in range(passes) for i in range(count))
                self.test9_sent.append(("GPIO" if value & 0x2000 else "LVDS", data))
            self.res[n] = self.overrides.get(n, DEFAULT_RESULTS[n])
            if self.ask_time_on_start:
                self._pending_time_req = True
            if n not in self.no_done:
                self._pending_done.append(n)

    def _flush_done(self):
        if self._pending_time_req:
            self._pending_time_req = False
            self.request_time()
        for n in self._pending_done:
            self._out += ("DONE TEST%d\r\n" % n).encode()
        self._pending_done = []

    def _exec(self, line):
        self._pending_done = []
        m = re.fullmatch(r"SET " + _TESTN + " (" + _HEX + "{4})", line)
        if m:
            self._write_cfg(int(m.group(1)), int(m.group(2), 16))
            return "OK"
        m = re.fullmatch(r"GET " + _TESTN + "_(CFG|RES)", line)
        if m:
            n = int(m.group(1))
            return "%04X" % (self.cfg[n] if m.group(2) == "CFG" else self.res[n])
        if line == "GET AFECFG":
            return "%012X" % SIM_AFECFG
        m = re.fullmatch(r"START " + _TESTN, line)
        if m:
            n = int(m.group(1))
            self._write_cfg(n, self.cfg[n] | 0x8000)
            return "OK"
        m = re.fullmatch(r"TIME (" + _HEX + "{12})", line)
        if m and self.time_supported:
            self.time_ms = int(m.group(1), 16)
            self.time_set_count += 1
            return "OK"
        m = re.fullmatch(r"PKL ([0-7])", line)
        if m:
            if self.pkt_busy:
                return "BSY"
            self._pending_bin = int(m.group(1))
            return "OK"
        if line.startswith("J"):
            return self._jtag(line)
        m = re.fullmatch(r"PKR ([0-7]) (" + _HEX + "{2})", line)
        if m:
            p, off = int(m.group(1)), int(m.group(2), 16)
            return bytes(self.pkt[p][(off + i) % 256] for i in range(8)).hex().upper()
        m = re.fullmatch(r"I2C ([0-7]" + _HEX + ") ([0-8]) ([0-8])(?: (" + _HEX + "+))?", line)
        if m:
            return self._i2c(m)
        m = re.fullmatch(r"PWR ([0-3]) ([01])", line)
        if m:
            sel, b = int(m.group(1)), int(m.group(2))
            if (b and sel in (0, 2) and self.jtag_cable and self.cable_detect
                    and self.fpga_interlock):
                return "ERR"                          # proposed FPGA interlock
            self.pwr_log.append((line, self._pg_bits()))   # POWER_GOOD when it landed
            self.pwr_out = (self.pwr_out & ~(1 << sel)) | (b << sel)
            self._apply_pwr()
            return "OK"
        if line == "GET PWR":
            # first digit: the 4 outputs; second: bit0 VDDC_GOOD, bit1
            # VDDIO_GOOD, bit2 JTAG cable (proposed) -- as cmd_interp.v
            cable = 4 if (self.jtag_cable and self.cable_detect) else 0
            return "%X%X" % (self.pwr_out, self._pg_bits() | cable)
        return "ERR"

    # -- JTAG engine (same commands and answers as cmd_interp + jtag_engine) --
    def _jstep(self, tms, tdi):
        trst_on = (self.j_trst == 0) == self.trst_active_low
        outs = []
        for c, t in enumerate(self.taps):
            if trst_on:
                t.reset(); outs.append(1); continue
            if self.j_sel >> c & 1:
                outs.append(t.clock(tms, tdi))
            else:
                outs.append(t.clock(0, 1))
        self.j_clocks += 1
        return outs

    def _jtag(self, line):
        H = _HEX
        m = re.fullmatch(r"JSEL ([01]" + H + "{2})", line)
        if m:
            self.j_sel = int(m.group(1), 16); return "OK"
        m = re.fullmatch(r"JCAP ([0-8])", line)
        if m:
            self.j_cap = int(m.group(1)); return "OK"
        m = re.fullmatch(r"JCLK (" + H + "{4})", line)
        if m and int(m.group(1), 16) >= 4:
            self.j_half = int(m.group(1), 16); return "OK"
        m = re.fullmatch(r"JRST ([01])", line)
        if m:
            self.j_trst = int(m.group(1))
            if (self.j_trst == 0) == self.trst_active_low:
                for t in self.taps:
                    t.reset()
            return "OK"
        m = re.fullmatch(r"JTMS ([1-8]) (" + H + "{2})", line)
        if m:
            n, v = int(m.group(1)), int(m.group(2), 16)
            for k in range(n):
                self._jstep(v >> k & 1, 1)
            return "OK"
        m = re.fullmatch(r"JSI (" + H + "{2}) (" + H + "{8}) ([01])", line)
        if m and 1 <= int(m.group(1), 16) <= 32:
            n, v, x = int(m.group(1), 16), int(m.group(2), 16), int(m.group(3))
            cap = 0
            for k in range(n):
                outs = self._jstep(1 if (x and k == n - 1) else 0, v >> k & 1)
                cap |= outs[self.j_cap] << k
            return "%08X" % cap
        m = re.fullmatch(r"JLD ([DEM]) (" + H + "{4})", line)
        if m and 1 <= int(m.group(2), 16) <= 0x1000:
            self._pending_bin = ("J", m.group(1), int(m.group(2), 16))
            return "OK"
        m = re.fullmatch(r"JSHF (" + H + "{4}) ([0-3])", line)
        if m and 1 <= int(m.group(1), 16) <= 0x8000:
            n, f = int(m.group(1), 16), int(m.group(2))
            d, e, mk = self.jbuf["D"], self.jbuf["E"], self.jbuf["M"]
            cnt, first = [0] * 9, [0xFFFF] * 9
            for k in range(n):
                byte, bit = k >> 3, k & 7
                outs = self._jstep(1 if (f & 1 and k == n - 1) else 0, d[byte] >> bit & 1)
                if f & 2 and mk[byte] >> bit & 1:
                    for c in range(9):
                        if self.j_sel >> c & 1 and outs[c] != (e[byte] >> bit & 1):
                            if cnt[c] == 0:
                                first[c] = k
                            cnt[c] = min(cnt[c] + 1, 0xFFFF)
            self.j_err = list(zip(cnt, first))
            return "%03X" % sum(1 << c for c in range(9) if cnt[c])
        m = re.fullmatch(r"JERR ([0-8])", line)
        if m:
            return "%04X %04X" % self.j_err[int(m.group(1))]
        return "ERR"

    def _i2c(self, m):
        addr, w, r = int(m.group(1), 16), int(m.group(2)), int(m.group(3))
        data = bytes.fromhex(m.group(4) or "")
        if w + r == 0 or len(data) != w or (w == 0) != (m.group(4) is None):
            return "ERR"
        dev = self.i2c_devices.get(addr)
        if dev is None:
            return "NAK"
        rd = dev.xfer(data, r)
        return rd.hex().upper() if r else "OK"
