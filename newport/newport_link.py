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
#  Purpose: UART link to the Newport FPGA test controller (newport_eval).
#
#------------------------------------------------------------------------------
"""
newport_link.py -- the host side of newport_eval's UART command protocol.

The FPGA's command interpreter (fpga/newport_eval/cmd_interp.v) speaks
line-oriented ASCII at 1 Mbaud 8-N-1:

    SET TESTn HHHH    GET TESTn_CFG    GET TESTn_RES    START TESTn
    GET AFECFG        I2C AA W R [HH..]      TIME HHHHHHHHHHHH
    PKL p             PKR p oo
    JSEL MMM  JCAP c  JCLK HHHH  JRST b  JTMS n HH  JSI nn HHHHHHHH x
    JLD b HHHH  JSHF HHHH f  JERR c       (JTAG, see newport_jtag.py)
    PWR s b           GET PWR     (DUT power-sequencing control, added
                                   2026-10-09 -- see set_pwr()/get_pwr()
                                   below; GET PWR widened from 1 to 2 hex
                                   digits 2026-10-09 part 2 for the 2 new
                                   DUT regulator POWER_GOOD inputs)
    (unsolicited)     DONE TESTn             ?TIME
                      !JTAG 1 / !JTAG 0      (USB-Blaster cable detect --
                                              proposed 2026-10-11, needs the
                                              board's detect pin + RTL)

Every character is echoed, and the FPGA only accepts the next character
after the echo has gone out (no receive buffer), so command() sends one
character at a time and waits for its echo. The response line follows the
echoed CR. Unsolicited "DONE TESTn" lines can arrive at any time between
commands; they are pulled out of the byte stream and kept in done_messages,
where wait_done() looks for them.

Time of day (replaces the testbenches' $time, which has no synthesizable
equivalent): the FPGA has no wall clock, so the host gives it one.

    TIME HHHHHHHHHHHH    host -> FPGA: milliseconds since 1970-01-01 UTC,
                         exactly 12 hex digits. Reply OK (ERR if malformed,
                         or from a bitstream without the command). The
                         FPGA loads it as an anchor and counts on from it
                         with its own clock, stamping its log lines.
    ?TIME                FPGA -> host, unsolicited: "send me the time".
                         Starts with "?", which the host never sends, so
                         it can't be mistaken for an echo. The FPGA sends
                         the whole line without letting an echo in
                         mid-line, waits TIME_REQ_TIMEOUT (100 ms, FPGA
                         side), and logs without a timestamp if no answer.

The host sends TIME when the link opens, and again before a START if the
last one is older than TIME_RESYNC_S. A ?TIME request is pulled out of the
byte stream like a DONE line and answered as soon as the current command
has finished, or straight away while wait_done() is waiting on a test --
the usual case, since the FPGA asks when a test starts. A ?TIME sent while
the host is idle is answered at the next command (the FPGA will have timed
out by then). A bitstream without TIME answers ERR: the link then stops
sending it (time_supported = False). fpga_time() turns an FPGA timestamp
back into local date/time.

Packet RAM (added 2026-10-01): the FPGA holds 8 packets of 256 bytes
(fpga/newport_eval/pkt_ram.v) that TEST9 sends through the DFE.

    PKL p        load slot p (0-7): the FPGA answers OK, then takes exactly
                 256 raw bytes with no echo, then answers the CRC-16/CCITT-
                 FALSE of what it got (binascii.crc_hqx(data, 0xFFFF)) as 4
                 hex digits. TO if the bytes stop for 100 ms, BSY if TEST9
                 is running. pkt_load() does the whole exchange and checks
                 the CRC.
    PKR p oo     8 bytes of slot p from offset oo, as 16 hex digits.
                 pkt_read() / pkt_read_all().

DUT power (2026-10-11, newport_power.py does the sequencing):

    !JTAG 1 / !JTAG 0    FPGA -> host, unsolicited (PROPOSED -- not in the
                         RTL yet): a USB-Blaster was plugged into / removed
                         from the FPGA's JTAG header. Starts with "!", which
                         the host never sends, for the same reason ?TIME
                         starts with "?". GET PWR's second digit bit 2
                         carries the same state (reserved/0 on bitstreams
                         without it). Kept in jtag_cable.

The link has a lock (self.lock, re-entrant) around every exchange, so
newport_power's background POWER_GOOD monitor can poll GET PWR between a
test's commands without splitting one (PKL/JLD hold it for the whole binary
load). When the monitor finds a power fault it sets self.fault; while
self.testing is set, the next command or wait_done() then raises
PowerFaultError, which stops the test.

The PKL line ends with CR only (as every command here does); an LF after it
would become data byte 0.

Opening a link: use open_link(). It returns a simulated FPGA in debug mode
(Karillon_TestExec --debug, or K_DEBUG=1), otherwise it opens the requested
port and checks that a Newport FPGA answers there. If the port can't be
opened or what comes back isn't the Newport protocol (wrong port, wrong
baud, no bitstream loaded), it lists the serial ports on the machine and
asks which one to use. A port picked that way is saved as NEWPORT_port in
cfg\cfg_com.py once the FPGA answers on it, so it's the default next time
(save_port(); only that one line of the file changes).
"""

import binascii
import datetime
import os
import re
import threading
import time

BAUD = 1_000_000
TIME_RESYNC_S = 600.0       # re-send TIME before a START if older than this
PKT_SLOTS = 8               # packet slots in the FPGA's pkt_ram.v
PKT_LEN = 256               # bytes per packet (fixed)
JTAG_BUF_BYTES = 4096       # each JTAG buffer (D/E/M) in jtag_engine.v

# PWR command signal-select values (added 2026-10-09) -- see cmd_interp.v's
# "PWR" command and newport_eval_top.v's DUT_VDDC_PSU_*/DUT_VDDIO_PSU_*
# ports. NOT yet confirmed vs. the real schematic -- see
# NPT100-EVB-FPGA-Connections-001 Rev 1.9 Section 1.6.
PWR_VDDC_ON   = 0
PWR_VDDC_RST  = 1
PWR_VDDIO_ON  = 2
PWR_VDDIO_RST = 3

# GET PWR readback-only bit positions for the 2 DUT regulator POWER_GOOD
# inputs (added 2026-10-09 part 2 -- DUT_VDDC_PWR_GOOD/DUT_VDDIO_PWR_GOOD
# on the FPGA; see newport_eval_top.v). Read-only: there is no SET for
# these, only GET PWR via get_pwr()'s return value, same bit positions.
PWR_VDDC_GOOD  = 4
PWR_VDDIO_GOOD = 5
# USB-Blaster cable present on the FPGA's JTAG header (PROPOSED 2026-10-11:
# GET PWR second digit bit 2, reserved/0 until the board's detect pin and
# the RTL exist). Read-only.
PWR_JTAG_CABLE = 6


# ---------------------------------------------------------------------------
# Errors
# ---------------------------------------------------------------------------
class NewportError(Exception):
    """Base class."""


class NakError(NewportError):
    """A slave didn't ACK (absent address, or a byte was refused)."""


class BusTimeoutError(NewportError):
    """FPGA reported TO: the I2C bus hung (SCL held low > 35 ms)."""


class CommandError(NewportError):
    """FPGA reported ERR (malformed command line) or an unexpected reply,
    or nothing sensible came back at all."""


class PowerFaultError(NewportError):
    """A DUT power fault (POWER_GOOD changed unexpectedly, USB-Blaster
    connected, ...) stopped the test; the DUT supplies have been powered
    down. See newport_power.py."""


class BusyError(NewportError):
    """PKL refused with BSY: TEST9 is running and using the packet RAM."""


class PacketError(NewportError):
    """A PKL load failed: CRC mismatch, or TO (the data stopped arriving)."""


def pkt_crc(data):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), as the FPGA computes
    it over a PKL load."""
    return binascii.crc_hqx(bytes(data), 0xFFFF)


# ---------------------------------------------------------------------------
# Debug mode (shared with the instrument drivers through K_Sim when present)
# ---------------------------------------------------------------------------
def debug_mode():
    """True when the FPGA should be simulated."""
    try:
        import K_Sim
        return K_Sim.is_debug()
    except ImportError:
        return os.environ.get('K_DEBUG', '').strip().lower() in ('1', 'true', 'yes', 'on')


# ---------------------------------------------------------------------------
# The link
# ---------------------------------------------------------------------------
_DONE_RE = re.compile(r"DONE TEST(\d+)\r\n")
_TIME_REQ_RE = re.compile(r"\?TIME\r\n")
_JTAG_RE = re.compile(r"!JTAG ([01])\r\n")
# A possibly-incomplete DONE or ?TIME line at the end of the received text.
# The request starts with "?" because the host never sends one, so a "?"
# can't be an echo and holding it back costs nothing. (A request starting
# with a letter the host does send, e.g. "T", would either cost a read
# timeout on every echoed T or let the host mistake it for its echo and send
# the next character before cmd_interp can take it.)
_DONE_PREFIX = re.compile(
    r"(?:D(?:O(?:N(?:E(?: (?:T(?:E(?:S(?:T\d{0,2}\r?)?)?)?)?)?)?)?)?"
    r"|\?(?:T(?:I(?:M(?:E\r?)?)?)?)?"
    r"|!(?:J(?:T(?:A(?:G(?: (?:[01]\r?)?)?)?)?)?)?)\Z")


def time_ms(t=None):
    """Milliseconds since 1970-01-01 UTC (now, or time.time() value t)."""
    return int(round((time.time() if t is None else t) * 1000))


def fpga_time(value):
    """An FPGA timestamp (ms since 1970 UTC, int or hex string) as a local
    datetime, for printing its log lines."""
    ms = int(value, 16) if isinstance(value, str) else int(value)
    return datetime.datetime.fromtimestamp(ms / 1000.0)


class NewportLink:
    """Talks to newport_eval's cmd_interp over the host UART."""

    MAX_W = 8
    MAX_R = 8

    def __init__(self, port, baud=BAUD, timeout=1.0, ser=None):
        if ser is None:
            import serial  # pyserial
            # Short read timeout: at 1 Mbaud a DONE line arrives as one
            # burst, so 5 ms of silence means nothing more is coming.
            ser = serial.Serial(port, baud, timeout=0.005)
        self.ser = ser
        self.port = port
        self.timeout = timeout
        self.done_messages = []   # unsolicited "DONE TESTn" lines not yet consumed
        self._rx = ""             # received text not yet matched to anything
        self.time_requests = 0    # ?TIME lines not yet answered
        self.time_answered = 0    # ?TIME lines answered (for the log / tests)
        self.time_supported = None  # None = not tried yet; False = bitstream answers ERR
        self.time_sent_at = None  # time.monotonic() of the last TIME that got OK
        self._in_command = False
        self.lock = threading.RLock()   # one exchange at a time (power monitor thread)
        self.jtag_cable = None    # USB-Blaster on the JTAG header: None = unknown
        self.jtag_events = []     # "!JTAG 1"/"!JTAG 0" seen, oldest first
        self.on_unsolicited = None  # callback(kind, value): the power monitor's wake-up
        self.fault = None         # DUT power fault text, set by newport_power
        self.testing = 0          # >0 while a DUT test runs (fault -> abort)
        self._fault_bypass = 0    # >0 while newport_power itself is talking
        self.pwr_changed_at = None  # time.monotonic() of the last PWR command

    # -- low level -------------------------------------------------------
    def _pump(self):
        """Read whatever the port has (or wait briefly for one byte)."""
        with self.lock:
            n = getattr(self.ser, 'in_waiting', 0) or 1
            chunk = self.ser.read(n)
            if chunk:
                self._rx += chunk.decode("ascii", errors="replace")
                self._rx = _DONE_RE.sub(self._collect_done, self._rx)
                self._rx = _TIME_REQ_RE.sub(self._collect_time_req, self._rx)
                self._rx = _JTAG_RE.sub(self._collect_jtag, self._rx)
            return bool(chunk)

    def _collect_jtag(self, m):
        present = m.group(1) == "1"
        self.jtag_cable = present
        self.jtag_events.append("!JTAG " + m.group(1))
        if self.on_unsolicited:
            try:
                self.on_unsolicited("jtag", present)
            except Exception:
                pass
        return ""

    def _check_fault(self):
        """Inside a DUT test, a power fault stops it at the next exchange."""
        if self.fault and self.testing and not self._fault_bypass:
            raise PowerFaultError(self.fault)

    def _collect_time_req(self, m):
        self.time_requests += 1
        return ""

    def _collect_done(self, m):
        self.done_messages.append("DONE TEST" + m.group(1))
        return ""

    def _take_until(self, text, deadline):
        """Consume received text up to and including `text`.

        A DONE line can start arriving while we wait for an echo, so a tail
        that could be the start of one ("D", "DON", "DONE TE"...) is held
        back until more bytes show whether it is. If nothing more arrives
        within one read timeout it was ordinary data (e.g. an echoed hex D).
        """
        while True:
            m = _DONE_PREFIX.search(self._rx)
            limit = m.start() if m else len(self._rx)
            i = self._rx.find(text, 0, limit)
            if i < 0 and m and not self._pump():
                i = self._rx.find(text)          # quiet: the tail was data
            if i >= 0:
                got = self._rx[:i + len(text)]
                self._rx = self._rx[i + len(text):]
                return got
            if time.monotonic() >= deadline:
                break
            self._pump()
        raise CommandError("no valid response from the FPGA on %s (got %r)"
                           % (self.port, self._rx[-60:]))

    def drain(self, quiet_time=0.0):
        """Pull in anything pending (collecting DONE lines) and discard the
        rest -- stale echoes or replies from an interrupted command. With
        quiet_time, keep reading until the line has been idle that long."""
        with self.lock:
            while getattr(self.ser, 'in_waiting', 0):
                self._pump()
            end = time.monotonic() + quiet_time
            while time.monotonic() < end:
                if self._pump():
                    end = time.monotonic() + quiet_time
            self._rx = ""

    # -- commands --------------------------------------------------------
    def command(self, line):
        """Send one command line, return the response line (without CR/LF).
        Answers any ?TIME request that arrived meanwhile once it's done."""
        with self.lock:
            self._check_fault()
            resp = self._command(line)
            if line.startswith("PWR "):
                self.pwr_changed_at = time.monotonic()
            self.service_time()
            return resp

    def _command(self, line):
        deadline = time.monotonic() + self.timeout
        self._in_command = True
        try:
            self.drain(0)
            # one character at a time, each echoed before the next is sent
            for ch in line + "\r":
                self.ser.write(ch.encode("ascii"))
                self._take_until(ch, deadline)
            return self._take_until("\r\n", deadline)[:-2]
        finally:
            self._in_command = False

    # -- time of day ---------------------------------------------------------
    def send_time(self, t=None):
        """TIME HHHHHHHHHHHH: give the FPGA the time of day (now, or
        time.time() value t). True on OK; False (and time_supported =
        False) if the bitstream answers ERR."""
        resp = self._command("TIME %012X" % (time_ms(t) & 0xFFFFFFFFFFFF))
        if resp == "OK":
            self.time_supported = True
            self.time_sent_at = time.monotonic()
            return True
        if resp == "ERR":
            self.time_supported = False
            return False
        raise CommandError("TIME -> %r" % resp)

    def service_time(self):
        """Answer pending ?TIME requests (one TIME covers them all). Does
        nothing inside a command; command() calls it again afterwards."""
        if self._in_command or not self.time_requests:
            return
        self._pump_waiting()
        n = self.time_requests
        self.time_requests = 0
        self.send_time()
        self.time_answered += n

    def sync_time(self, max_age=None):
        """Send TIME if the bitstream takes it and the last one is older
        than max_age seconds (None = always)."""
        if self.time_supported is False:
            return False
        if (max_age is not None and self.time_sent_at is not None
                and time.monotonic() - self.time_sent_at < max_age):
            return True
        return self.send_time()

    def _pump_waiting(self):
        while getattr(self.ser, 'in_waiting', 0):
            self._pump()

    def resync(self):
        """Send a bare CR: with an empty line buffer the FPGA only echoes it;
        with leftover characters it answers ERR. Either way the parser is
        back at the start of a line afterwards."""
        with self.lock:
            self.drain(0)
            self.ser.write(b"\r")
            self._take_until("\r", time.monotonic() + self.timeout)
            self.drain(0.05)

    def identify(self):
        """Check that a Newport FPGA is answering: GET AFECFG must return 12
        hex digits. Returns the AFE config string, raises CommandError."""
        resp = self.command("GET AFECFG")
        if not re.fullmatch(r"[0-9A-F]{12}", resp):
            raise CommandError("not a Newport FPGA reply to GET AFECFG: %r" % resp)
        return resp

    # -- test registers --------------------------------------------------
    @staticmethod
    def _check_n(n):
        if not 0 <= int(n) <= 19:
            raise ValueError("test number must be 0..19, got %r" % n)
        return int(n)

    def _ok(self, line):
        resp = self.command(line)
        if resp != "OK":
            raise CommandError("%r -> %r" % (line, resp))

    def set_cfg(self, n, value):
        """SET TESTn HHHH -- writes the whole config register (bit 15 = start)."""
        self._ok("SET TEST%d %04X" % (self._check_n(n), value & 0xFFFF))

    def start(self, n):
        """START TESTn -- sets bit 15, keeps bits 14:0. Re-sends TIME first
        if the last one is older than TIME_RESYNC_S."""
        n = self._check_n(n)
        self.sync_time(TIME_RESYNC_S)
        self._ok("START TEST%d" % n)

    def _get4(self, line):
        resp = self.command(line)
        if not re.fullmatch(r"[0-9A-F]{4}", resp):
            raise CommandError("%r -> %r" % (line, resp))
        return int(resp, 16)

    def get_cfg(self, n):
        return self._get4("GET TEST%d_CFG" % self._check_n(n))

    def get_res(self, n):
        return self._get4("GET TEST%d_RES" % self._check_n(n))

    def get_afecfg(self):
        """GET AFECFG as an int (48 bits)."""
        return int(self.identify(), 16)

    # -- packet RAM (TEST9) --------------------------------------------------
    @staticmethod
    def _check_slot(slot):
        if not 0 <= int(slot) < PKT_SLOTS:
            raise ValueError("packet slot must be 0..%d, got %r" % (PKT_SLOTS - 1, slot))
        return int(slot)

    def pkt_load(self, slot, data):
        """PKL: load exactly PKT_LEN bytes into packet slot 0-7 and check the
        FPGA's CRC. Returns the CRC. Raises BusyError (TEST9 running),
        PacketError (CRC mismatch or TO -- the slot then holds a mix of old
        and new bytes, so load it again) or CommandError."""
        slot = self._check_slot(slot)
        data = bytes(data)
        if len(data) != PKT_LEN:
            raise ValueError("a packet is exactly %d bytes, got %d" % (PKT_LEN, len(data)))
        return self._binary_load("PKL %d" % slot, data)

    def _binary_load(self, line, data):
        """The PKL / JLD exchange: line -> OK, raw bytes (no echo), CRC.
        Returns the CRC; raises BusyError / PacketError / CommandError."""
        with self.lock:                  # nothing may come between line and bytes
            self._check_fault()
            return self._binary_load_locked(line, data)

    def _binary_load_locked(self, line, data):
        self._in_command = True          # no TIME may go out mid-load
        try:
            resp = self._command(line)
            self._in_command = True      # _command() cleared it
            if resp == "BSY":
                raise BusyError("%s: TEST9 is running" % line)
            if resp != "OK":
                raise CommandError("%r -> %r" % (line, resp))
            # raw bytes, no echo: one write (~10 us per byte at 1 Mbaud)
            self.ser.write(data)
            deadline = time.monotonic() + self.timeout + len(data) * 20e-6
            resp = self._take_until("\r\n", deadline)[:-2]
        finally:
            self._in_command = False
        if resp == "TO":
            # Bytes that turn up late would now be read as a command line:
            # let them land, then put the parser back at the start of a line.
            self.drain(0.05)
            self.resync()
            raise PacketError("%s: FPGA timed out waiting for data" % line)
        want = pkt_crc(data)
        if not re.fullmatch(r"[0-9A-F]{4}", resp):
            raise CommandError("%s: unexpected reply %r" % (line, resp))
        if int(resp, 16) != want:
            raise PacketError("%s: CRC %s from the FPGA, %04X sent" % (line, resp, want))
        self.service_time()
        return want

    def pkt_read(self, slot, offset=0):
        """PKR: 8 bytes of slot from offset (wraps within the packet)."""
        slot = self._check_slot(slot)
        line = "PKR %d %02X" % (slot, offset & 0xFF)
        resp = self.command(line)
        if not re.fullmatch(r"[0-9A-F]{16}", resp):
            raise CommandError("%r -> %r" % (line, resp))
        return bytes.fromhex(resp)

    def pkt_read_all(self, slot):
        """The whole slot via 32 PKR commands (slow; for debugging -- the
        PKL CRC already confirms a load)."""
        return b"".join(self.pkt_read(slot, off) for off in range(0, PKT_LEN, 8))

    # -- JTAG (raw J* commands; newport_jtag.py does the TAP work) -----------
    def _jok(self, line):
        resp = self.command(line)
        if resp != "OK":
            raise CommandError("%r -> %r" % (line, resp))

    def jsel(self, mask):
        """JSEL: chips to drive, bit i = chip i (i = 3*Y + X)."""
        if not 0 <= mask <= 0x1FF:
            raise ValueError("chip mask must be 0..1FF")
        self._jok("JSEL %03X" % mask)

    def jcap(self, chip):
        """JCAP: chip whose TDO jsi() returns."""
        if not 0 <= chip <= 8:
            raise ValueError("chip must be 0..8")
        self._jok("JCAP %d" % chip)

    def jclk(self, half):
        """JCLK: TCK half period in 250 MHz clocks (4..65535)."""
        if not 4 <= half <= 0xFFFF:
            raise ValueError("TCK half period must be 4..65535 clocks")
        self._jok("JCLK %04X" % half)
        self.j_half = half

    def jrst(self, level):
        """JRST: drive the shared TRST pin to level 0/1."""
        self._jok("JRST %d" % (1 if level else 0))

    # -- DUT power-sequencing control (added 2026-10-09) --------------------
    def set_pwr(self, sel, level):
        """PWR s b -- drive one of the 4 DUT power-sequencing control
        outputs (DUT_VDDC_PSU_ON/_RST, DUT_VDDIO_PSU_ON/_RST on the FPGA)
        to level 0/1. sel is one of the PWR_VDDC_ON/PWR_VDDC_RST/
        PWR_VDDIO_ON/PWR_VDDIO_RST constants above. This drives the pin to
        exactly the level given -- it does NOT assume active-high vs.
        active-low for either ON or RST, same as jrst() above; that
        polarity is the eval board's convention and is not yet confirmed
        against the real schematic (see NPT100-EVB-FPGA-Connections-001
        Rev 1.9 Section 1.6). All 4 default to 0 out of FPGA reset.

        2026-10-11: the Rev A0 schematic + MPQ8785 datasheet settle the
        polarity -- ON drives the regulators' CTRL pins (high = on), RST
        drives ALT/RST in its RST function (low = VOUT back to VBOOT); see
        newport_power.py. Turning a rail ON (sel 0 or 2, level 1) is
        refused here while a USB-Blaster is known to be on the JTAG header
        (jtag_cable) -- the interlock at the lowest level."""
        if not 0 <= int(sel) <= 3:
            raise ValueError("pwr signal select must be 0..3, got %r" % (sel,))
        if level and int(sel) in (PWR_VDDC_ON, PWR_VDDIO_ON) and self.jtag_cable:
            raise PowerFaultError("USB-Blaster connected to the FPGA's JTAG header: "
                                  "DUT power is blocked until it's removed")
        self._jok("PWR %d %d" % (int(sel), 1 if level else 0))

    def get_pwr(self):
        """GET PWR -- the current level of all 4 PWR outputs, plus the 2
        DUT regulator POWER_GOOD inputs (added 2026-10-09 part 2), packed
        into one int: bit0 = VDDC_ON, bit1 = VDDC_RST, bit2 = VDDIO_ON,
        bit3 = VDDIO_RST (same bit order set_pwr()'s sel uses), bit4 =
        VDDC_GOOD, bit5 = VDDIO_GOOD (PWR_VDDC_GOOD/PWR_VDDIO_GOOD above;
        read-only, no corresponding SET). The wire response widened from
        1 to 2 hex digits for this (2026-10-09 part 2) -- old firmware
        answering with 1 digit will fail the regex below rather than
        silently returning stale/wrong GOOD bits.

        Fixed 2026-10-11: the FPGA sends the 4 outputs as the FIRST digit
        and the POWER_GOOD nibble as the second (cmd_interp.v respbuf[0] /
        respbuf[1]); this used to be int(resp, 16), which put the outputs
        in bits 4-7 and the GOOD bits in bits 0-3 -- the opposite of the
        bit numbers above. Now each digit goes where the constants say.
        Bit 6 (PWR_JTAG_CABLE, second digit bit 2, proposed) also updates
        jtag_cable."""
        resp = self.command("GET PWR")
        if not re.fullmatch(r"[0-9A-Fa-f]{2}", resp):
            raise CommandError("%r -> %r" % ("GET PWR", resp))
        value = int(resp[0], 16) | (int(resp[1], 16) << 4)
        self.jtag_cable = bool(value >> PWR_JTAG_CABLE & 1)
        return value

    def jtms(self, n, bits):
        """JTMS: n (1-8) TCK clocks with TMS = bits, LSB first."""
        if not 1 <= n <= 8:
            raise ValueError("JTMS takes 1..8 bits")
        self._jok("JTMS %d %02X" % (n, bits & 0xFF))

    def jsi(self, n, value, exit_last=True):
        """JSI: shift n (1-32) bits of value (LSB first); returns the JCAP
        chip's TDO bits as an int (bit 0 = first bit out)."""
        if not 1 <= n <= 32:
            raise ValueError("JSI takes 1..32 bits")
        line = "JSI %02X %08X %d" % (n, value & 0xFFFFFFFF, 1 if exit_last else 0)
        resp = self.command(line)
        if not re.fullmatch(r"[0-9A-F]{8}", resp):
            raise CommandError("%r -> %r" % (line, resp))
        return int(resp, 16) & ((1 << n) - 1)

    def jld(self, buf, data):
        """JLD: load JTAG buffer 'D' (TDI), 'E' (expected TDO) or 'M'
        (compare mask) with 1-4096 bytes. Returns the CRC."""
        data = bytes(data)
        if buf not in ("D", "E", "M"):
            raise ValueError("buffer must be D, E or M")
        if not 1 <= len(data) <= JTAG_BUF_BYTES:
            raise ValueError("JLD takes 1..%d bytes, got %d" % (JTAG_BUF_BYTES, len(data)))
        return self._binary_load("JLD %s %04X" % (buf, len(data)), data)

    def jshf(self, nbits, exit_last, compare):
        """JSHF: shift nbits (1-32768) from buffer D. Returns the 9-bit mask
        of chips whose TDO mismatched E (where M = 1); 0 without compare."""
        if not 1 <= nbits <= JTAG_BUF_BYTES * 8:
            raise ValueError("JSHF takes 1..%d bits" % (JTAG_BUF_BYTES * 8))
        line = "JSHF %04X %d" % (nbits, (2 if compare else 0) | (1 if exit_last else 0))
        # the FPGA answers when the shift is done: allow for slow TCK
        secs = nbits * 2 * getattr(self, "j_half", 125) * 4e-9
        saved, self.timeout = self.timeout, self.timeout + 2 * secs
        try:
            resp = self.command(line)
        finally:
            self.timeout = saved
        if not re.fullmatch(r"[0-9A-F]{3}", resp):
            raise CommandError("%r -> %r" % (line, resp))
        return int(resp, 16)

    def jerr(self, chip):
        """JERR: (mismatched bits, first mismatched bit or None) for a chip
        from the last JSHF."""
        resp = self.command("JERR %d" % chip)
        m = re.fullmatch(r"([0-9A-F]{4}) ([0-9A-F]{4})", resp)
        if not m:
            raise CommandError("JERR %d -> %r" % (chip, resp))
        first = int(m.group(2), 16)
        return int(m.group(1), 16), (None if first == 0xFFFF else first)

    # -- DONE events -----------------------------------------------------
    def clear_done(self, n=None):
        """Forget DONE TESTn (or every DONE) seen so far."""
        self.drain(0)
        if n is None:
            self.done_messages.clear()
        else:
            tag = "DONE TEST%d" % n
            self.done_messages = [m for m in self.done_messages if m != tag]

    def wait_done(self, n, timeout=1.0):
        """Wait for the unsolicited DONE TESTn. True if it arrived."""
        tag = "DONE TEST%d" % n
        deadline = time.monotonic() + timeout
        while True:
            with self.lock:                # one step at a time: the power
                self._check_fault()        # monitor can poll in between
                self.service_time()        # the FPGA asks when a test starts
                if tag in self.done_messages:
                    self.done_messages.remove(tag)
                    return True
                if time.monotonic() >= deadline:
                    return False
                self._pump()

    # -- raw I2C -----------------------------------------------------------
    @staticmethod
    def i2c_line(addr, wbytes, rlen):
        """Encode one I2C transaction as the FPGA command line."""
        wbytes = bytes(wbytes)
        if not 0 <= addr <= 0x7F:
            raise ValueError("7-bit address out of range: %r" % addr)
        if len(wbytes) > NewportLink.MAX_W or not 0 <= rlen <= NewportLink.MAX_R:
            raise ValueError("W and R must each be 0..8")
        if len(wbytes) == 0 and rlen == 0:
            raise ValueError("empty transaction")
        line = "I2C %02X %d %d" % (addr, len(wbytes), rlen)
        if wbytes:
            line += " " + wbytes.hex().upper()
        return line

    def i2c(self, addr, wbytes=b"", rlen=0):
        """Raw transaction. Returns the bytes read (b"" for write-only)."""
        line = self.i2c_line(addr, wbytes, rlen)
        resp = self.command(line)
        if resp == "NAK":
            raise NakError("address 0x%02X: missed ACK" % addr)
        if resp == "TO":
            raise BusTimeoutError("address 0x%02X: I2C bus timeout" % addr)
        if resp == "ERR":
            raise CommandError("FPGA rejected %r" % line)
        if rlen == 0:
            if resp != "OK":
                raise CommandError("unexpected reply %r" % resp)
            return b""
        if len(resp) != 2 * rlen or not re.fullmatch(r"[0-9A-F]+", resp):
            raise CommandError("unexpected reply %r" % resp)
        return bytes.fromhex(resp)

    def probe(self, addr):
        """True if something ACKs at addr (1-byte read, no command)."""
        try:
            self.i2c(addr, b"", 1)
            return True
        except NakError:
            return False

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Opening a link, with port selection when the default doesn't answer
# ---------------------------------------------------------------------------
def list_ports():
    """[(device, description), ...] for the serial ports on this machine."""
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        return []
    return [(p.device, p.description) for p in sorted(lp.comports(), key=lambda p: p.device)]


def try_port(port, baud=BAUD, timeout=1.0):
    """Open port and check that a Newport FPGA answers.
    Returns (link, None) on success or (None, reason) on failure."""
    try:
        link = NewportLink(port, baud, timeout)
    except Exception as e:                       # SerialException, OSError...
        return None, "can't open %s: %s" % (port, e)
    try:
        link.resync()
        link.identify()
        link.sync_time()                  # ERR from an older bitstream is fine
    except NewportError as e:
        link.close()
        return None, "%s doesn't answer like the Newport FPGA (%s)" % (port, e)
    return link, None


def find_cfg_com():
    """Path of the Test Exec's cfg_com.py: the imported module if it's on
    sys.path, otherwise ..\\cfg\\cfg_com.py next to this folder. None if
    neither exists."""
    try:
        import cfg_com
        path = getattr(cfg_com, "__file__", None)
        if path and path.endswith(".pyc"):
            path = path[:-1]
        if path and os.path.isfile(path):
            return path
    except ImportError:
        pass
    path = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         os.pardir, "cfg", "cfg_com.py"))
    return path if os.path.isfile(path) else None


_PORT_LINE = re.compile(r"^(NEWPORT_port[ \t]*=[ \t]*)(['\"])[^'\"\r\n]*\2", re.M)


def save_port(port, path=None):
    """Write port into cfg_com.py as NEWPORT_port. Only that assignment
    changes (quotes, spacing, a trailing comment and the file's CRLF/LF line
    endings are kept); if there's no such line, one is added at the end.
    Returns the path written, or None if there was nothing to do or it
    couldn't be written (reported, never raised)."""
    path = path or find_cfg_com()
    if path is None:
        print("Newport FPGA: cfg_com.py not found - %s not saved as the default port" % port)
        return None
    try:
        with open(path, "rb") as f:
            raw = f.read()
        text = raw.decode("utf-8")
        eol = "\r\n" if "\r\n" in text else "\n"
        m = _PORT_LINE.search(text)
        if m:
            if text[m.end(2):m.end() - 1].lower() == port.lower():
                return None                                  # already the default
            new = text[:m.start()] + m.group(1) + m.group(2) + port + m.group(2) + text[m.end():]
        else:
            new = text if text.endswith(("\n", "\r")) or not text else text + eol
            new += "NEWPORT_port = '%s'%s" % (port, eol)
        tmp = path + ".tmp"
        with open(tmp, "wb") as f:
            f.write(new.encode("utf-8"))
        os.replace(tmp, path)
    except (OSError, UnicodeDecodeError) as e:
        print("Newport FPGA: couldn't save %s to %s (%s)" % (port, path, e))
        return None
    try:                                  # keep an imported cfg_com in step
        import sys
        mod = sys.modules.get("cfg_com")
        if mod is not None and os.path.normcase(os.path.abspath(getattr(mod, "__file__", ""))) \
                == os.path.normcase(os.path.abspath(path)):
            mod.NEWPORT_port = port
    except Exception:
        pass
    print("Newport FPGA: %s saved as NEWPORT_port in %s" % (port, path))
    return path


def open_link(port, baud=BAUD, timeout=1.0, interactive=True, debug=None, remember=True):
    """
    Return a connected NewportLink, or None if the user gives up.

    Debug mode returns a link to the simulated FPGA (newport_sim.SimFPGA).
    Otherwise port is tried first. If it can't be opened or doesn't answer
    like the Newport FPGA, and interactive is True, the serial ports on this
    machine are listed and the user picks another one (Enter to cancel).
    With remember, a port picked that way is saved in cfg_com.py once the
    FPGA answers on it.
    """
    if debug is None:
        debug = debug_mode()
    if debug:
        from newport_sim import SimFPGA
        print("SIM Newport FPGA: simulated (no FPGA on %s)" % port)
        link = NewportLink(port, baud, timeout, ser=SimFPGA(port))
        link.sync_time()
        return link

    requested = port
    link, why = try_port(port, baud, timeout)
    while link is None:
        print("\nNewport FPGA: " + why)
        if not interactive:
            return None
        ports = list_ports()
        if ports:
            print("Serial ports on this computer:")
            for i, (dev, desc) in enumerate(ports):
                print("  %d: %s  %s" % (i, dev, desc))
            prompt = "Choose a port by number or name (e.g. COM5), <Enter> to cancel: "
        else:
            print("No serial ports found (is the USB cable plugged in?)")
            prompt = "Type a port name to try (e.g. COM5), <Enter> to cancel: "
        choice = input(prompt).strip()
        if not choice:
            return None
        if choice.isdigit() and int(choice) < len(ports):
            choice = ports[int(choice)][0]
        port = choice
        link, why = try_port(port, baud, timeout)
    print("Newport FPGA answering on %s" % link.port)
    if remember and link.port.lower() != str(requested).lower():
        save_port(link.port)
    return link
