#!/usr/bin/env python3
"""
fpga_pmbus.py -- host-side PMBus control of the NPT100 eval board's
MPQ8785 buck regulators through the Newport FPGA (newport_eval).
(Renamed from newport_pmbus.py 2026-10-11.)

The FPGA knows nothing about PMBus. It exposes one raw I2C transaction over
its UART command interpreter (cmd_interp.v / i2c_txn.v):

    I2C AA W R [HH..]      AA = 7-bit address (hex), W/R = bytes to write /
                           read (0-8 each), HH.. = 2*W hex digits
    -> "hhhh..." (R>0), "OK" (R=0), "NAK", "TO" (bus timeout), or "ERR"

Everything PMBus -- command codes, SMBus protocol shapes (send byte, read/
write byte/word, block read), little-endian word order, PEC (CRC-8), and the
MPQ8785's data formats -- is encoded/decoded here, so no PMBus strings are
stored in the FPGA.

The UART transport (1 Mbaud, echo pacing, DONE TESTn filtering, port
selection) is newport_link.NewportLink, shared with the test runner
(newport_tests.py). This module adds only the PMBus layer on top of it.

Usage (CLI):
    python fpga_pmbus.py COM3 scan
    python fpga_pmbus.py COM3 --reg vddcore telemetry
    python fpga_pmbus.py COM3 --addr 0x20 read READ_VOUT
    python fpga_pmbus.py COM3 --addr 0x20 vout 0.8
    python fpga_pmbus.py COM3 --reg vddio get TON_RISE      (engineering units)
    python fpga_pmbus.py COM3 --reg vddio set TON_RISE 2.0
    python fpga_pmbus.py COM3 rails nom         (both rails to nominal; also low / high)
    python fpga_pmbus.py COM3 --addr 0x20 --pec write OPERATION 0x80
    python fpga_pmbus.py COM3 raw 20 8B --rlen 2
    python fpga_pmbus.py --selftest          (no hardware: simulated FPGA)

As a library:
    link = open_link("COM3")
    buck = MPQ8785(link, 0x20, pec=True)
    buck.set_vout(0.8); print(buck.telemetry())
    set_rails(link, "low")                      # both DUT rails to -3%

Requires pyserial for real hardware (pip install pyserial).

The command table and scaling below were checked against the MPQ8785
datasheet Rev. 1.0 (10/10/2024), K_Env\\doc\\datasheets\\Regulators\\
Monolithic_MPQ8785.pdf, on 2026-09-30. Still to confirm on the bench:
MFR_ID byte order, the VOUT LSB with MFR_VDIFF_GAIN_SEL = 0.5 (VDDIO), and
which phase answers reads on the 2-phase VDDCORE pair (both are at 0x20).
"""

import argparse
import math
import os
import sys
import time

from newport_link import (NewportLink, open_link, NewportError, NakError,
                          BusTimeoutError, CommandError)

# ---------------------------------------------------------------------------
# Board configuration: the DUT supply rails on the NPT100-EVB-array (Grinn
# schematic Rev A0, 25/09/2026, sheets 15-16).
#
# Address = MFR_ADDR_PMBUS (D2h) bits[6:4] (default 2) plus 4 LSBs from the
# ADDR/VBOOT divider (datasheet Table 3, RADDR1 = 3.32k):
#   VDDCORE  U10 + U11, 2-phase, 3.32k / 47R  -> X0h -> 0x20 (both phases;
#            the datasheet recommends one address for master and slave)
#   VDDIO    U12, 1 phase,       3.32k / 124R -> X2h -> 0x22
#
# The working values come from cfg\cfg_regulators.py (the Test Exec's cfg
# folder, found on sys.path or at ..\cfg next to this folder). The values
# below are the fallback if that file isn't there; CFG_SOURCE says which.
#
# nominal/tol: DUT supply spec. The margin corners are nominal*(1 -/+ tol),
# rounded to a VOUT code toward nominal so they never land outside the spec.
# VDDIO (1.8 V, no output divider) needs MFR_VDIFF_GAIN_SEL = 0.5 (EBh[10]);
# set_rail() sets it while the output is off (datasheet Table 1).
# ---------------------------------------------------------------------------
_DEFAULTS = {
    "RAILS": {
        "vddcore": {"addr": 0x20, "nominal": 0.800, "tol": 0.03, "settings": {}},
        "vddio":   {"addr": 0x22, "nominal": 1.800, "tol": 0.03, "settings": {}},
    },
    "RAIL_ORDER": ["vddio", "vddcore"],
    "STARTUP_RAILS": "nom",
    "PEC": False,
    "VERIFY_TOL": 0.01,
    "SETTLE_S": 0.05,
    "SWEEP_LIMIT": 0.10,
}
VDIFF_GAIN1_MAX = 1.55                 # V; above this EBh[10] must be 1


def _find_cfg():
    try:
        import cfg_regulators
        return cfg_regulators
    except ImportError:
        pass
    d = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "cfg"))
    if os.path.isfile(os.path.join(d, "cfg_regulators.py")):
        sys.path.insert(0, d)
        import cfg_regulators
        return cfg_regulators
    return None


def apply_cfg(module=None):
    """Load the rail configuration from `module` (default: cfg_regulators
    if it can be found, otherwise the built-in _DEFAULTS). Returns the
    source description, also kept in CFG_SOURCE."""
    global RAILS, RAIL_ORDER, STARTUP_RAILS, PEC, VERIFY_TOL, SETTLE_S, SWEEP_LIMIT
    global REGULATORS, CFG_SOURCE
    vals = {k: getattr(module, k, v) for k, v in _DEFAULTS.items()} if module else dict(_DEFAULTS)
    RAILS = {n: dict(r, settings=dict(r.get("settings") or {})) for n, r in vals["RAILS"].items()}
    RAIL_ORDER = [n for n in vals["RAIL_ORDER"] if n in RAILS] + \
                 [n for n in RAILS if n not in vals["RAIL_ORDER"]]
    STARTUP_RAILS = vals["STARTUP_RAILS"]
    PEC = bool(vals["PEC"])
    VERIFY_TOL = vals["VERIFY_TOL"]
    SETTLE_S = vals["SETTLE_S"]
    SWEEP_LIMIT = vals["SWEEP_LIMIT"]
    REGULATORS = {name: r["addr"] for name, r in RAILS.items()}
    CFG_SOURCE = getattr(module, "__file__", "cfg_regulators") if module else "built-in defaults"
    return CFG_SOURCE


apply_cfg(_find_cfg())


class PecError(NewportError):
    """PEC byte returned by the device didn't match the computed CRC-8."""


# ---------------------------------------------------------------------------
# SMBus PEC: CRC-8, polynomial x^8 + x^2 + x + 1 (0x07), init 0
# ---------------------------------------------------------------------------
def crc8(data, crc=0):
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


# ---------------------------------------------------------------------------
# Generic PMBus/SMBus device
# ---------------------------------------------------------------------------
class PMBusDevice:
    def __init__(self, link, addr, pec=False):
        self.link = link
        self.addr = addr
        self.pec = pec

    def _wpec(self, payload):
        return crc8(bytes([self.addr << 1]) + payload) if self.pec else None

    def _write(self, payload):
        payload = bytes(payload)
        p = self._wpec(payload)
        if p is not None:
            payload += bytes([p])
        self.link.i2c(self.addr, payload, 0)

    def _read(self, code, n):
        rlen = n + (1 if self.pec else 0)
        data = self.link.i2c(self.addr, bytes([code]), rlen)
        if self.pec:
            want = crc8(bytes([self.addr << 1, code, (self.addr << 1) | 1]) + data[:-1])
            if data[-1] != want:
                raise PecError("0x%02X cmd %02Xh: PEC %02X != %02X" % (self.addr, code, data[-1], want))
            data = data[:-1]
        return data

    # SMBus protocols
    def send_byte(self, code):
        self._write([code])

    def write_byte(self, code, value):
        self._write([code, value & 0xFF])

    def write_word(self, code, value):
        self._write([code, value & 0xFF, (value >> 8) & 0xFF])

    def read_byte(self, code):
        return self._read(code, 1)[0]

    def read_word(self, code):
        d = self._read(code, 2)
        return d[0] | (d[1] << 8)

    def block_read(self, code, max_len):
        """Block read: count byte + up to max_len data bytes. The FPGA reads
        at most 8 bytes, so max_len <= 6 with PEC, <= 7 without."""
        d = self._read(code, 1 + max_len)
        count = d[0]
        if count > max_len:
            raise CommandError("block count %d > expected %d" % (count, max_len))
        return d[1:1 + count]


# ---------------------------------------------------------------------------
# MPQ8785 command table: name -> (code, access, protocol, size)
#   access:   "rw" | "r" | "w"
#   protocol: "send" (no data), "byte", "word", "block"
#   size:     data bytes (block: max count)
# Transcribed from the MPQ8785 datasheet command summary -- verify.
# ---------------------------------------------------------------------------
MPQ8785_COMMANDS = {
    "PAGE":                    (0x00, "rw", "byte", 1),
    "OPERATION":               (0x01, "rw", "byte", 1),
    "ON_OFF_CONFIG":           (0x02, "rw", "byte", 1),
    "CLEAR_FAULTS":            (0x03, "w",  "send", 0),
    "CLEAR_LAST_FAULT":        (0x08, "w",  "send", 0),
    "LAST_FAULT_RESTORE":      (0x0C, "w",  "send", 0),
    "WRITE_PROTECT":           (0x10, "rw", "byte", 1),
    "STORE_ALL":               (0x15, "w",  "send", 0),
    "RESTORE_ALL":             (0x16, "w",  "send", 0),
    "CAPABILITY":              (0x19, "r",  "byte", 1),
    "PMBUS_PS_NUM":            (0x1C, "rw", "byte", 1),
    "VOUT_MODE":               (0x20, "rw", "byte", 1),
    "VOUT_COMMAND":            (0x21, "rw", "word", 2),
    "VOUT_MAX":                (0x24, "rw", "word", 2),
    "VOUT_MARGIN_HIGH":        (0x25, "rw", "word", 2),
    "VOUT_MARGIN_LOW":         (0x26, "rw", "word", 2),
    "VOUT_SCALE_LOOP":         (0x29, "rw", "word", 2),
    "VOUT_MIN":                (0x2B, "rw", "word", 2),
    "COEFFICIENT":             (0x30, "r",  "block", 5),
    "VIN_ON":                  (0x35, "rw", "word", 2),
    "VIN_OFF":                 (0x36, "rw", "word", 2),
    "IOUT_CAL_GAIN":           (0x38, "rw", "word", 2),
    "IOUT_CAL_OFFSET":         (0x39, "rw", "word", 2),
    "IOUT_OC_FAULT_LIMIT":     (0x46, "rw", "word", 2),
    "IOUT_OC_WARN_LIMIT":      (0x4A, "rw", "word", 2),
    "VBOOT_SET_FOR_X0H_ADDR":  (0x4D, "rw", "word", 2),
    "OT_FAULT_LIMIT":          (0x4F, "rw", "word", 2),
    "OT_WARN_LIMIT":           (0x51, "rw", "word", 2),
    "VIN_OV_FAULT_LIMIT":      (0x55, "rw", "word", 2),
    "VIN_OV_WARN_LIMIT":       (0x57, "rw", "word", 2),
    "VBOOT_SET_FOR_X4H_ADDR":  (0x5E, "rw", "word", 2),
    "VBOOT_SET_FOR_X8H_ADDR":  (0x5F, "rw", "word", 2),
    "TON_DELAY":               (0x60, "rw", "word", 2),
    "TON_RISE":                (0x61, "rw", "word", 2),
    "TOFF_DELAY":              (0x64, "rw", "word", 2),
    "TOFF_FALL":               (0x65, "rw", "word", 2),
    "VBOOT_SET_FOR_XEH_ADDR":  (0x6A, "rw", "word", 2),
    "STATUS_WORD":             (0x79, "r",  "word", 2),
    "STATUS_VOUT":             (0x7A, "r",  "byte", 1),
    "STATUS_IOUT":             (0x7B, "r",  "byte", 1),
    "STATUS_INPUT":            (0x7C, "r",  "byte", 1),
    "STATUS_TEMPERATURE":      (0x7D, "r",  "byte", 1),
    "STATUS_CML":              (0x7E, "r",  "byte", 1),
    "REV_ID":                  (0x80, "r",  "byte", 1),
    "READ_VIN":                (0x88, "r",  "word", 2),
    "READ_VOUT":               (0x8B, "r",  "word", 2),
    "READ_IOUT":               (0x8C, "r",  "word", 2),
    "READ_TEMPERATURE":        (0x8D, "r",  "word", 2),
    "PMBUS_REVISION":          (0x98, "r",  "byte", 1),
    "MFR_ID":                  (0x99, "r",  "block", 3),
    "MFR_REVISION":            (0x9B, "r",  "block", 1),
    "MFR_CONFIG_ID":           (0xC0, "rw", "word", 2),
    "MFR_CONFIG_CODE_REV":     (0xC1, "rw", "word", 2),
    "MFR_PRODUCT_REV_USER":    (0xC2, "rw", "word", 2),
    "MFR_SILICON_REV":         (0xC3, "rw", "byte", 1),
    "MFR_APS_LEVEL":           (0xC5, "rw", "word", 2),
    "MFR_CONFIG_A":            (0xD0, "rw", "word", 2),
    "MFR_FS_CFG":              (0xD1, "rw", "word", 2),
    "MFR_ADDR_PMBUS":          (0xD2, "rw", "word", 2),
    "MFR_VOUT_RATE":           (0xD3, "rw", "word", 2),
    "MFR_PWM_TIME_CFG":        (0xD4, "rw", "word", 2),
    "MFR_PWM_TIME_CFG2":       (0xD5, "rw", "word", 2),
    "MFR_PHASE_BLANK_TIME":    (0xD6, "rw", "word", 2),
    "MFR_PHASE_SLOPE_BLANK_TIME": (0xD7, "rw", "word", 2),
    "MFR_SLOPE_BLANK_TIME":    (0xD8, "rw", "word", 2),
    "MFR_BLANK_TIME_LV":       (0xD9, "rw", "word", 2),
    "MFR_SLOPE_CNT_DCM":       (0xDA, "rw", "word", 2),
    "MFR_SLOPE_SR_DCM":        (0xDB, "rw", "word", 2),
    "MFR_SW_BLOCK_LIMIT":      (0xDC, "rw", "word", 2),
    "MFR_VCOMP":               (0xDD, "rw", "word", 2),
    "MFR_DROOP_CFG":           (0xDE, "rw", "word", 2),
    "MFR_CONFIG_B":            (0xDF, "rw", "word", 2),
    "MFR_DC_LOOP_CTRL":        (0xE0, "rw", "word", 2),
    "MFR_CB_LOOP_CTRL":        (0xE1, "rw", "word", 2),
    "MFR_FS_LOOP_CTRL":        (0xE2, "rw", "word", 2),
    "MFR_VIN_CFG":             (0xE3, "rw", "word", 2),
    "MFR_VIN_SCALE":           (0xE4, "rw", "word", 2),
    "MFR_TEMP_TUNE":           (0xE5, "rw", "word", 2),
    "MFR_PROTECT_CFG":         (0xE6, "rw", "word", 2),
    "MFR_PROTECT_LEVEL":       (0xE7, "rw", "word", 2),
    "MFR_PRT_DELAY":           (0xE8, "rw", "word", 2),
    "SMBALERT_MASK":           (0xE9, "rw", "word", 2),
    "MFR_NOCP_OCP_SET":        (0xEA, "rw", "word", 2),
    "MFR_LEVEL_SEL2":          (0xEB, "rw", "word", 2),
    "MFR_PG_CFG":              (0xEC, "rw", "word", 2),
    "MFR_PS_CTRL":             (0xED, "rw", "word", 2),
    "MFR_PMBUS_LOCK":          (0xEE, "rw", "word", 2),
    "MFR_SET_SYNC_CFG":        (0xEF, "rw", "word", 2),
    "MFR_SLAVE_PROTECT":       (0xF0, "rw", "word", 2),
    "MFR_CTRL":                (0xF1, "rw", "word", 2),
    "MFR_AUTO_SLOPE_CFG":      (0xF2, "rw", "word", 2),
    "MFR_SLOPE_DELTA_LIMIT":   (0xF3, "rw", "word", 2),
    "MFR_RETRY_TIMES":         (0xF4, "rw", "word", 2),
    "MFR_CFG_EXT":             (0xF5, "rw", "word", 2),
    "MFR_CDROOP_SET":          (0xF6, "rw", "word", 2),
    "MFR_CFG_BACKUP":          (0xF7, "rw", "word", 2),
    "CHECK_SUM_FUNC":          (0xF8, "r",  "word", 2),
    "PROTECTION":              (0xFA, "r",  "word", 2),
    "PROTECTION_LAST":         (0xFB, "r",  "word", 2),
    "MFR_VBOOT_CFG":           (0xFC, "rw", "word", 2),
    "CLEAR_NVM_FAULT":         (0xFE, "w",  "send", 0),
}

# MPQ8785 STATUS_WORD bits (datasheet p.40-41; bits 12, 10:9 and 3 reserved).
STATUS_WORD_BITS = {
    15: "VOUT", 14: "IOUT_POUT", 13: "INPUT", 11: "PG_STATUS",
    8: "WATCH_DOG", 7: "BUSY", 6: "OFF", 5: "VOUT_OV_FAULT",
    4: "IOUT_OC_FAULT", 2: "TEMPERATURE", 1: "CML", 0: "DRMOS_FAULT",
}
STATUS_OFF = 1 << 6

# Fields with a fixed scale: name -> (LSB, field bits, unit, signed).
# LSB "vout" = the VOUT_MODE step (1.953125 mV linear, 1.5625 mV VID/direct);
# "vboot" = 4 x that step. Datasheet Rev. 1.0 register map.
SCALED = {
    "VOUT_COMMAND":           ("vout", 12, "V", False),
    "VOUT_MAX":               ("vout", 12, "V", False),
    "VOUT_MIN":               ("vout", 12, "V", False),
    "VOUT_MARGIN_HIGH":       ("vout", 12, "V", False),
    "VOUT_MARGIN_LOW":        ("vout", 12, "V", False),
    "MFR_VBOOT_CFG":          ("vout", 12, "V", False),
    "VBOOT_SET_FOR_X0H_ADDR": ("vboot", 10, "V", False),
    "VBOOT_SET_FOR_X4H_ADDR": ("vboot", 10, "V", False),
    "VBOOT_SET_FOR_X8H_ADDR": ("vboot", 10, "V", False),
    "VBOOT_SET_FOR_XEH_ADDR": ("vboot", 10, "V", False),
    "VOUT_SCALE_LOOP":        (1 / 1024, 11, "", False),
    "VIN_ON":                 (0.2, 7, "V", False),
    "VIN_OFF":                (0.2, 7, "V", False),
    "VIN_OV_FAULT_LIMIT":     (0.2, 7, "V", False),
    "VIN_OV_WARN_LIMIT":      (0.2, 7, "V", False),
    "IOUT_OC_FAULT_LIMIT":    (1.0, 10, "A", False),
    "IOUT_OC_WARN_LIMIT":     (1.0, 10, "A", False),
    "IOUT_CAL_OFFSET":        (0.0625, 9, "A", True),
    "OT_FAULT_LIMIT":         (1.0, 8, "C", False),
    "OT_WARN_LIMIT":          (1.0, 8, "C", False),
    "TON_DELAY":              (0.1, 10, "ms", False),
    "TON_RISE":               (0.1, 9, "ms", False),
    "TOFF_DELAY":             (0.1, 10, "ms", False),
    "TOFF_FALL":              (0.1, 9, "ms", False),
}


class MPQ8785(PMBusDevice):
    """One MPQ8785 buck. Scaling per the MPQ8785 datasheet Rev. 1.0."""

    READ_VIN_LSB = 0.025       # V/LSB      READ_VIN[15:0]
    READ_IOUT_LSB = 0.0625     # A/LSB      READ_IOUT[15:0]
    READ_TEMP_LSB = 1.0        # degC/LSB   READ_TEMPERATURE[7:0], unsigned
    VOUT_MASK = 0x0FFF         # VOUT_COMMAND/MAX/MIN/MARGIN: bits[11:0]
    VDIFF_GAIN_BIT = 1 << 10   # MFR_LEVEL_SEL2 (EBh) bit 10: 1 = VDIFF gain 0.5

    def __init__(self, link, addr, pec=False, name=None):
        super().__init__(link, addr, pec)
        self.name = name or "0x%02X" % addr
        self._vout_lsb = None

    # -- generic by name ---------------------------------------------------
    def read(self, name):
        code, access, proto, size = MPQ8785_COMMANDS[name]
        if "r" not in access:
            raise ValueError("%s is not readable" % name)
        if proto == "byte":
            return self.read_byte(code)
        if proto == "word":
            return self.read_word(code)
        if proto == "block":
            return self.block_read(code, size)
        raise ValueError("%s has no data to read" % name)

    def write(self, name, value=None):
        code, access, proto, size = MPQ8785_COMMANDS[name]
        if "w" not in access:
            raise ValueError("%s is not writable" % name)
        if proto == "send":
            return self.send_byte(code)
        if proto == "byte":
            return self.write_byte(code, value)
        if proto == "word":
            return self.write_word(code, value)
        raise ValueError("%s: block write not supported" % name)

    # -- VOUT (format depends on VOUT_MODE[6:5]) ---------------------------
    def vout_lsb(self, refresh=False):
        """V/LSB for VOUT_COMMAND / READ_VOUT / VOUT_MAX etc.
        VOUT_MODE[6:5] = 00 linear, exponent forced to -9 in [4:0]
        (5'b10111) -> 1.953125 mV; 01 VID or 1x direct -> 1.5625 mV."""
        if self._vout_lsb is None or refresh:
            mode = self.read("VOUT_MODE")
            if (mode >> 5) & 0x3 == 0:
                exp = mode & 0x1F
                exp = exp - 32 if exp & 0x10 else exp
                self._vout_lsb = 2.0 ** exp
            else:
                self._vout_lsb = 0.0015625
        return self._vout_lsb

    def get_vout_setpoint(self):
        return (self.read("VOUT_COMMAND") & self.VOUT_MASK) * self.vout_lsb()

    def vout_limits(self):
        """(VOUT_MIN, VOUT_MAX) in volts."""
        lsb = self.vout_lsb()
        return ((self.read("VOUT_MIN") & self.VOUT_MASK) * lsb,
                (self.read("VOUT_MAX") & self.VOUT_MASK) * lsb)

    def set_vout_code(self, code):
        """Write VOUT_COMMAND after checking VOUT_MIN/VOUT_MAX (the part
        would silently clamp to them). Returns the volts written."""
        lsb = self.vout_lsb()
        if not 0 <= code <= self.VOUT_MASK:
            raise ValueError("VOUT code %d out of range" % code)
        volts = code * lsb
        vmin, vmax = self.vout_limits()
        if vmax and volts > vmax + lsb / 2:
            raise ValueError("%s: %.4f V exceeds VOUT_MAX %.4f V" % (self.name, volts, vmax))
        if volts < vmin - lsb / 2:
            raise ValueError("%s: %.4f V is below VOUT_MIN %.4f V" % (self.name, volts, vmin))
        self.write("VOUT_COMMAND", code)
        return volts

    def set_vout(self, volts):
        return self.set_vout_code(int(round(volts / self.vout_lsb())))

    def vdiff_gain_half(self):
        """True if MFR_LEVEL_SEL2 (EBh) bit 10 selects VDIFF gain 0.5
        (needed for 1.55-3.1 V without an output divider)."""
        return bool(self.read("MFR_LEVEL_SEL2") & self.VDIFF_GAIN_BIT)

    def is_on(self):
        return not self.read("STATUS_WORD") & STATUS_OFF

    # -- scaled fields (engineering units) --------------------------------------
    def _scale(self, name):
        lsb, bits, unit, signed = SCALED[name]
        if lsb == "vout":
            lsb = self.vout_lsb()
        elif lsb == "vboot":
            lsb = 4 * self.vout_lsb()
        return lsb, bits, unit, signed

    def get(self, name):
        """Read a SCALED field in engineering units."""
        lsb, bits, unit, signed = self._scale(name)
        raw = self.read(name) & ((1 << bits) - 1)
        if signed and raw & (1 << (bits - 1)):
            raw -= 1 << bits
        return raw * lsb

    def set(self, name, value):
        """Write a SCALED field in engineering units. Returns the value
        actually written (after rounding to the LSB)."""
        lsb, bits, unit, signed = self._scale(name)
        raw = int(round(value / lsb))
        lo, hi = (-(1 << (bits - 1)), (1 << (bits - 1)) - 1) if signed else (0, (1 << bits) - 1)
        if not lo <= raw <= hi:
            raise ValueError("%s: %g %s out of range (%g..%g)" % (name, value, unit, lo * lsb, hi * lsb))
        if name == "VOUT_COMMAND":
            return self.set_vout_code(raw)
        self.write(name, raw & ((1 << bits) - 1))
        return raw * lsb

    # -- telemetry -----------------------------------------------------------
    def read_vout(self):
        return self.read("READ_VOUT") * self.vout_lsb()     # bits[15:0], up to 6.4 V

    def read_vin(self):
        return self.read("READ_VIN") * self.READ_VIN_LSB

    def read_iout(self):
        return self.read("READ_IOUT") * self.READ_IOUT_LSB

    def read_temperature(self):
        return (self.read("READ_TEMPERATURE") & 0xFF) * self.READ_TEMP_LSB

    def status(self):
        w = self.read("STATUS_WORD")
        return w, [STATUS_WORD_BITS[b] for b in range(15, -1, -1)
                   if w & (1 << b) and b in STATUS_WORD_BITS]

    def telemetry(self):
        return {
            "vin_V": round(self.read_vin(), 4),
            "vout_V": round(self.read_vout(), 5),
            "iout_A": round(self.read_iout(), 4),
            "temp_C": self.read_temperature(),
            "status_word": "0x%04X" % self.read("STATUS_WORD"),
        }

    # -- control -------------------------------------------------------------
    def on(self):
        self.write("OPERATION", self.read("OPERATION") | 0x80)

    def off(self):
        self.write("OPERATION", self.read("OPERATION") & ~0x80 & 0xFF)

    def clear_faults(self):
        self.write("CLEAR_FAULTS")

    def mfr_id(self):
        return self.read("MFR_ID")


# ---------------------------------------------------------------------------
# DUT rails: nominal, spec corners, and arbitrary sweep points
# ---------------------------------------------------------------------------
class RailError(NewportError):
    """A rail setting was refused (out of the allowed range, or it needs a
    configuration change that isn't safe with the output on)."""


def rail_device(link, name, pec=None):
    """MPQ8785 for a RAILS entry; pec=None uses the configured PEC."""
    return MPQ8785(link, RAILS[name]["addr"], pec=PEC if pec is None else pec, name=name)


def apply_settings(link, rails=None, pec=None):
    """Write each rail's `settings` (SCALED field -> value, None = leave
    alone) and read them back. Returns [(rail, field, wanted, readback)]."""
    done = []
    for name in (rails or RAIL_ORDER):
        dev = rail_device(link, name, pec)
        for field, value in RAILS[name].get("settings", {}).items():
            if value is None:
                continue
            if field not in SCALED or field == "VOUT_COMMAND":
                raise RailError("%s: settings[%r] isn't a settable scaled field" % (name, field))
            try:
                dev.set(field, value)
            except ValueError as e:
                raise RailError("%s: %s" % (name, e))
            back = dev.get(field)
            if abs(back - value) > dev._scale(field)[0] / 2 + 1e-9:
                raise RailError("%s: %s wrote %g, read back %g" % (name, field, value, back))
            done.append((name, field, value, back))
    return done


def configure_rails(link, pec=None):
    """Startup: write the configured settings, then set every rail to
    STARTUP_RAILS (if not None). Returns (settings list, set_rails list)."""
    s = apply_settings(link, pec=pec)
    r = set_rails(link, STARTUP_RAILS, pec) if STARTUP_RAILS else []
    return s, r


def corner_code(dev, name, corner):
    """VOUT code for 'nom', 'low' or 'high'. low/high are nominal*(1 -/+ tol)
    rounded toward nominal, so they stay inside the spec."""
    r = RAILS[name]
    lsb = dev.vout_lsb()
    if corner == "nom":
        return int(round(r["nominal"] / lsb))
    if corner == "low":
        return int(math.ceil(r["nominal"] * (1 - r["tol"]) / lsb - 1e-9))
    if corner == "high":
        return int(math.floor(r["nominal"] * (1 + r["tol"]) / lsb + 1e-9))
    raise ValueError("corner must be nom, low or high, not %r" % (corner,))


def set_rail(link, name, target, pec=None, verify=True):
    """Set one rail to 'nom' / 'low' / 'high' or to a voltage.

    Rounds to the nearest VOUT code (corners round toward nominal), refuses
    anything more than SWEEP_LIMIT from nominal, and sets VDIFF gain 0.5 when
    the target needs it (only with the output off). If the output is on,
    reads READ_VOUT after SETTLE_S and checks it is within VERIFY_TOL.

    Returns {rail, addr, set_V, pct, on, meas_V, ok}; pct is vs nominal and
    meas_V is None when the output is off (nothing to verify)."""
    r = RAILS[name]
    dev = rail_device(link, name, pec)
    lsb = dev.vout_lsb(refresh=True)
    code = corner_code(dev, name, target) if isinstance(target, str) else int(round(target / lsb))
    volts = code * lsb
    if abs(volts / r["nominal"] - 1) > SWEEP_LIMIT + 1e-9:
        raise RailError("%s: %.4f V is more than %.0f%% from nominal %.3f V (SWEEP_LIMIT)"
                        % (name, volts, SWEEP_LIMIT * 100, r["nominal"]))
    on = dev.is_on()
    if volts > VDIFF_GAIN1_MAX and not dev.vdiff_gain_half():
        if on:
            raise RailError("%s: %.4f V needs VDIFF gain 0.5 (EBh[10]); turn the output "
                            "off before changing it" % (name, volts))
        dev.write("MFR_LEVEL_SEL2", dev.read("MFR_LEVEL_SEL2") | MPQ8785.VDIFF_GAIN_BIT)
    try:
        dev.set_vout_code(code)
    except ValueError as e:
        raise RailError(str(e))
    res = {"rail": name, "addr": r["addr"], "set_V": volts,
           "pct": 100.0 * (volts / r["nominal"] - 1), "on": on, "meas_V": None, "ok": True}
    if verify and on:
        time.sleep(SETTLE_S)
        meas = dev.read_vout()
        res["meas_V"] = meas
        res["ok"] = abs(meas - volts) <= VERIFY_TOL * volts
    return res


def set_rails(link, target, pec=None, verify=True):
    """Set several rails, in RAIL_ORDER. target is 'nom' / 'low' / 'high'
    for every rail, or a dict {rail: 'nom'|'low'|'high'|volts} (rails not
    in the dict are left alone). Returns the list of set_rail() results."""
    if not isinstance(target, dict):
        target = {n: target for n in RAILS}
    for n in target:
        if n not in RAILS:
            raise RailError("unknown rail %r (RAILS has %s)" % (n, ", ".join(RAILS)))
    return [set_rail(link, n, target[n], pec, verify) for n in RAIL_ORDER if n in target]


def format_rail(res):
    meas = "off" if res["meas_V"] is None else "meas %.4f V" % res["meas_V"]
    return "%-7s 0x%02X  set %.4f V (%+.2f%%)  %s%s" % (
        res["rail"], res["addr"], res["set_V"], res["pct"], meas,
        "" if res["ok"] else "  ** OUT OF TOLERANCE **")


def sweep_points(rail, lo_pct, hi_pct, step_pct, others="nom"):
    """Points for a one-rail sweep: rail from nominal*(1+lo_pct/100) to
    nominal*(1+hi_pct/100) in step_pct steps; the other rails held at
    `others` ('nom'/'low'/'high'). Each point is a set_rails() target dict."""
    if step_pct <= 0:
        raise ValueError("step_pct must be > 0")
    nom = RAILS[rail]["nominal"]
    n = int(math.floor((hi_pct - lo_pct) / step_pct + 1e-9))
    pts = []
    for i in range(n + 1):
        p = {r: others for r in RAILS if r != rail}
        p[rail] = round(nom * (1 + (lo_pct + i * step_pct) / 100.0), 6)
        pts.append(p)
    return pts


def corner_points():
    """Every combination of low/nom/high across the rails (3^N points)."""
    pts = [{}]
    for r in RAIL_ORDER:
        pts = [dict(p, **{r: c}) for p in pts for c in ("low", "nom", "high")]
    return pts


# ---------------------------------------------------------------------------
# Offline self-test: newport_sim.SimFPGA speaks the UART protocol and uses
# this simple MPQ8785 register model (with PEC) for the I2C command --
# exercises this module end to end.
# ---------------------------------------------------------------------------
class _MockMPQ:
    """One MPQ8785 for the simulated FPGA.

    Power (2026-10-11): with ctrl None (no FPGA power wiring, the PMBus
    selftest) the output follows OPERATION bit 7 as before. Once
    newport_sim.SimFPGA wires the CTRL / ALT-RST pins (ctrl, rst_low) it
    follows ON_OFF_CONFIG like the real part: 0xx on whenever biased, 1x0
    OPERATION only, 101 CTRL only, 111 both; CTRL polarity from bit 1.
    While rst_low (ALT/RST in its RST function, DFh bit 0 = 0, pulled low)
    VOUT_COMMAND is held at the boot voltage. PG (power good) rises
    pg_delay after the output turns on; force_pg_low simulates a fault."""

    def __init__(self, addr):
        self.addr = addr
        self.regs = {c: 0 for c, *_ in MPQ8785_COMMANDS.values()}
        self.regs.update({0x01: 0x00, 0x20: 0x17, 0x21: 0x01B3, 0x24: 0x0400,
                          0x88: 480, 0x8B: 0x01B3, 0x8C: 160, 0x8D: 45, 0x79: 0x0040})
        self.last_cmd = 0
        self.cleared = 0
        self.pec_errors = 0
        self.vout_offset = 0             # READ_VOUT error in LSBs (selftest)
        self.ctrl = None                 # CTRL pin level; None = not wired
        self.rst_low = False             # ALT/RST pin held low
        self.vboot_code = 0x01B3         # VOUT_COMMAND while RST is low
        self.pg_delay = 0.003            # s from output on to PG
        self.force_pg_low = False
        self._on_since = None

    def output_on(self):
        if self.ctrl is None:
            return bool(self.regs[0x01] & 0x80)
        cfg = self.regs[0x02]
        mode = (cfg >> 2) & 7
        ctrl_on = bool(self.ctrl) == bool(cfg & 0x02)    # bit 1: 1 = active high
        bus_on = bool(self.regs[0x01] & 0x80)
        if not mode & 0b100:
            on = True                                   # 0xx: on whenever biased
        elif mode & 0b001 == 0:
            on = bus_on                                 # 1x0: OPERATION only
        elif mode == 0b101:
            on = ctrl_on                                # CTRL only
        else:
            on = ctrl_on and bus_on                     # 111: both
        if on and self._on_since is None:
            self._on_since = time.monotonic()
        elif not on:
            self._on_since = None
        return on

    def pg(self):
        on = self.output_on()
        return (on and not self.force_pg_low
                and time.monotonic() - self._on_since >= self.pg_delay)

    def _rst_hold(self):
        if self.rst_low and not self.regs[0xDF] & 1:      # RST function selected
            self.regs[0x21] = self.vboot_code

    def xfer(self, w, rlen):
        self._rst_hold()
        if w:
            self.last_cmd = w[0]
            code = w[0]
            proto = next((p for c, _, p, s in MPQ8785_COMMANDS.values() if c == code), "word")
            size = {"send": 0, "byte": 1, "word": 2, "block": 0}[proto]
            data = w[1:]
            if rlen == 0 and len(data) == size + 1:      # trailing PEC byte
                if crc8(bytes([self.addr << 1]) + w[:-1]) != w[-1]:
                    self.pec_errors += 1
                data = data[:-1]
            if rlen == 0:
                if proto == "send":
                    if code == 0x03:
                        self.cleared += 1
                elif proto == "byte" and data:
                    self.regs[code] = data[0]
                elif proto == "word" and len(data) == 2:
                    self.regs[code] = data[0] | data[1] << 8
                self._rst_hold()
                return b""
        code = self.last_cmd
        proto = next((p for c, _, p, s in MPQ8785_COMMANDS.values() if c == code), "word")
        val = self.regs[code]
        on = self.output_on()
        if code == 0x79:                 # STATUS_WORD: OFF / PG_STATUS follow the output
            pg = self.pg() if self.ctrl is not None else on
            val = (val & ~0x0840) | (0x0800 if pg else 0) | (0 if on else 0x0040)
        elif code == 0x8B:               # READ_VOUT: the setpoint (+ offset) when on
            val = (self.regs[0x21] + self.vout_offset) if on else 0
        if code == 0x99:
            out = bytes([3]) + b"MPS"
        elif proto == "byte":
            out = bytes([val & 0xFF])
        else:
            out = bytes([val & 0xFF, val >> 8 & 0xFF])
        pec = crc8(bytes([self.addr << 1, code, self.addr << 1 | 1]) + out)
        out = (out + bytes([pec]) + b"\xFF" * 16)[:rlen]
        return out


def selftest():
    cfg = _find_cfg()
    apply_cfg(None)                     # test against the built-in values
    try:
        return _selftest()
    finally:
        apply_cfg(cfg)


def _selftest():
    from newport_sim import SimFPGA
    ser = SimFPGA(power_wiring=False)    # PMBus only: outputs follow OPERATION
    devs = [ser.i2c_devices[a] for a in REGULATORS.values()]
    link = NewportLink("SIM", ser=ser, timeout=2.0)
    fails = 0

    def check(what, got, want):
        nonlocal fails
        ok = got == want
        fails += not ok
        print("%s  %-40s %r%s" % ("pass" if ok else "FAIL", what, got, "" if ok else "  (want %r)" % (want,)))

    check("line format READ_VOUT", NewportLink.i2c_line(0x20, b"\x8B", 2), "I2C 20 1 2 8B")
    check("line format write word", NewportLink.i2c_line(0x20, b"\x21\xCD\x01", 0), "I2C 20 3 0 21CD01")
    check("crc8 known vector", crc8(b"123456789"), 0xF4)
    check("scan", [a for a in range(0x18, 0x28) if link.probe(a)], sorted(REGULATORS.values()))
    for pec in (False, True):
        b = MPQ8785(link, 0x20, pec=pec)
        tag = " (PEC)" if pec else ""
        check("MFR_ID" + tag, b.mfr_id(), b"MPS")
        check("VOUT_MODE lsb" + tag, b.vout_lsb(refresh=True), 2 ** -9)
        v = b.set_vout(0.85)
        check("set_vout 0.85 -> code" + tag, devs[0].regs[0x21], round(0.85 / 2 ** -9))
        check("readback setpoint" + tag, round(b.get_vout_setpoint(), 4), round(v, 4))
        check("READ_VIN" + tag, round(b.read_vin(), 4), 12.0)
        check("READ_IOUT" + tag, b.read_iout(), 10.0)
        b.on()
        check("OPERATION on" + tag, devs[0].regs[0x01] & 0x80, 0x80)
        b.clear_faults()
    check("CLEAR_FAULTS sent twice", devs[0].cleared, 2)
    check("device saw no bad PEC", devs[0].pec_errors, 0)
    # corrupted PEC is caught
    b = MPQ8785(link, 0x20, pec=True)
    orig = devs[0].xfer
    devs[0].xfer = lambda w, r: bytes(x ^ (1 if i == r - 1 else 0) for i, x in enumerate(orig(w, r)))
    try:
        b.read("READ_VOUT")
        check("bad PEC detected", False, True)
    except PecError:
        check("bad PEC detected", True, True)
    devs[0].xfer = orig
    try:
        MPQ8785(link, 0x33).read("READ_VOUT")
        check("NAK on absent address", False, True)
    except NakError:
        check("NAK on absent address", True, True)
    try:
        link.i2c(0x20, b"", 9)
        check("R>8 rejected", False, True)
    except ValueError:
        check("R>8 rejected", True, True)

    # -- datasheet scaling -------------------------------------------------
    core, io = devs[0], ser.i2c_devices[REGULATORS["vddio"]]
    b = MPQ8785(link, 0x20)
    check("rails at schematic addresses", REGULATORS, {"vddcore": 0x20, "vddio": 0x22})
    check("TON_RISE 2.0 ms -> raw 20", (round(b.set("TON_RISE", 2.0), 3), core.regs[0x61]), (2.0, 20))
    check("VIN_ON 9.4 V -> raw 47", (round(b.set("VIN_ON", 9.4), 3), core.regs[0x35]), (9.4, 47))
    check("IOUT_CAL_OFFSET -0.125 A", (b.set("IOUT_CAL_OFFSET", -0.125), core.regs[0x39],
                                      b.get("IOUT_CAL_OFFSET")), (-0.125, 0x1FE, -0.125))
    try:
        b.set("TON_RISE", 60.0)
        check("TON_RISE 60 ms (> 9 bits) refused", False, True)
    except ValueError:
        check("TON_RISE 60 ms (> 9 bits) refused", True, True)
    core.regs[0x8D] = 0x1F2D                   # reserved high byte set
    check("READ_TEMPERATURE uses bits[7:0]", b.read_temperature(), 45.0)
    core.regs[0x8D] = 45
    check("STATUS_WORD names", b.status(), (0x0800, ["PG_STATUS"]))

    # -- rails ----------------------------------------------------------------
    core.regs[0x01] = io.regs[0x01] = 0x00     # both outputs off
    lsb = 2 ** -9
    res = set_rails(link, "nom")
    check("nom: order + setpoints (outputs off)",
          [(x["rail"], x["set_V"], x["meas_V"]) for x in res],
          [("vddio", round(1.8 / lsb) * lsb, None), ("vddcore", round(0.8 / lsb) * lsb, None)])
    check("vddio 1.8 V set VDIFF gain 0.5", io.regs[0xEB] & 0x0400, 0x0400)
    check("vddcore VDIFF gain left at 1", core.regs[0xEB] & 0x0400, 0)
    res = {x["rail"]: x for x in set_rails(link, "low")}
    check("low corners inside -3%", [-3.0 <= res[r]["pct"] < 0 for r in RAIL_ORDER], [True, True])
    check("vddcore low code rounds toward nom", core.regs[0x21], math.ceil(0.776 / lsb))
    res = {x["rail"]: x for x in set_rails(link, "high")}
    check("high corners inside +3%", [0 < res[r]["pct"] <= 3.0 for r in RAIL_ORDER], [True, True])
    core.regs[0x01] = 0x80                     # vddcore on: verified
    r = set_rail(link, "vddcore", 0.79)
    check("on: READ_VOUT verified", (r["on"], r["ok"], r["meas_V"] == r["set_V"]), (True, True, True))
    core.vout_offset = 20                      # ~39 mV high: out of tolerance
    check("on: READ_VOUT off by 5% flagged", set_rail(link, "vddcore", 0.79)["ok"], False)
    core.vout_offset = 0
    for what, fn in (("0.9 V on vddcore (>10%) refused", lambda: set_rail(link, "vddcore", 0.9)),
                     ("unknown rail refused", lambda: set_rails(link, {"vdd": "nom"}))):
        try:
            fn()
            check(what, False, True)
        except RailError:
            check(what, True, True)
    io.regs[0xEB] = 0
    io.regs[0x01] = 0x80                       # vddio on with gain 1: refuse
    try:
        set_rail(link, "vddio", "nom")
        check("gain change with output on refused", False, True)
    except RailError:
        check("gain change with output on refused", True, True)
    io.regs[0x01] = 0x00
    pts = sweep_points("vddcore", -5, 5, 2.5)
    check("sweep_points vddcore -5..+5% step 2.5",
          [p["vddcore"] for p in pts], [0.76, 0.78, 0.8, 0.82, 0.84])
    check("sweep_points holds vddio at nom", {p["vddio"] for p in pts}, {"nom"})
    check("corner_points = 9", (len(corner_points()), corner_points()[0], corner_points()[1]),
          (9, {"vddio": "low", "vddcore": "low"}, {"vddio": "low", "vddcore": "nom"}))
    set_rails(link, "nom")

    # -- configured settings (current limits etc.) -------------------------
    RAILS["vddcore"]["settings"] = {"IOUT_OC_FAULT_LIMIT": 45, "IOUT_OC_WARN_LIMIT": 40}
    RAILS["vddio"]["settings"] = {"IOUT_OC_FAULT_LIMIT": 6, "IOUT_OC_WARN_LIMIT": None}
    io.regs[0x4A] = 0x0033
    s, r = configure_rails(link)
    check("settings written + read back", [(n, f, b) for n, f, w, b in s],
          [("vddio", "IOUT_OC_FAULT_LIMIT", 6.0),
           ("vddcore", "IOUT_OC_FAULT_LIMIT", 45.0), ("vddcore", "IOUT_OC_WARN_LIMIT", 40.0)])
    check("None leaves the part's value", io.regs[0x4A], 0x0033)
    check("then STARTUP_RAILS", [x["rail"] for x in r], RAIL_ORDER)
    for what, bad in (("unknown settings field refused", {"IOUT_LIMIT": 5}),
                      ("VOUT_COMMAND in settings refused", {"VOUT_COMMAND": 0.8}),
                      ("OC limit > 1023 A refused", {"IOUT_OC_FAULT_LIMIT": 2000})):
        RAILS["vddio"]["settings"] = bad
        try:
            apply_settings(link)
            check(what, False, True)
        except RailError:
            check(what, True, True)
    RAILS["vddio"]["settings"] = {}
    RAILS["vddcore"]["settings"] = {}

    class _Cfg:                         # a cfg_regulators module
        RAILS = {"vddcore": {"addr": 0x20, "nominal": 0.75, "tol": 0.05}}
        PEC = True
    apply_cfg(_Cfg)
    check("cfg module overrides", (REGULATORS, RAILS["vddcore"]["nominal"], PEC, RAIL_ORDER,
                                   RAILS["vddcore"]["settings"], SETTLE_S),
          ({"vddcore": 0x20}, 0.75, True, ["vddcore"], {}, 0.05))
    apply_cfg(None)
    print("lines sent (sample):", ser.lines_seen[:6])
    print("SELFTEST", "PASS" if fails == 0 else "FAIL (%d)" % fails)
    return fails == 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _int(s):
    return int(s, 0)


def main(argv=None):
    ap = argparse.ArgumentParser(description="PMBus control of MPQ8785 bucks via the Newport FPGA")
    ap.add_argument("port", nargs="?", help="serial port, e.g. COM3 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--addr", type=_int, help="7-bit PMBus address, e.g. 0x20")
    g.add_argument("--reg", choices=sorted(REGULATORS), help="regulator name from REGULATORS")
    ap.add_argument("--pec", action="store_true", help="use SMBus PEC on every transaction")
    ap.add_argument("--selftest", action="store_true", help="run offline self-test (no hardware)")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("scan", help="probe all 7-bit addresses")
    p = sub.add_parser("read", help="read a command by name");  p.add_argument("name")
    p = sub.add_parser("write", help="write a command by name"); p.add_argument("name"); p.add_argument("value", nargs="?", type=_int)
    p = sub.add_parser("vout", help="show or set VOUT (volts)");  p.add_argument("volts", nargs="?", type=float)
    sub.add_parser("telemetry", help="VIN/VOUT/IOUT/TEMP/STATUS_WORD")
    sub.add_parser("status", help="decode STATUS_WORD")
    sub.add_parser("on"); sub.add_parser("off"); sub.add_parser("clear")
    sub.add_parser("list", help="list known MPQ8785 commands")
    p = sub.add_parser("get", help="read a scaled field (V, A, ms, C)"); p.add_argument("name")
    p = sub.add_parser("set", help="write a scaled field (V, A, ms, C)"); p.add_argument("name"); p.add_argument("value", type=float)
    p = sub.add_parser("rails", help="set the DUT rails: nom, low or high (all rails)")
    p.add_argument("corner", choices=["nom", "low", "high"])
    sub.add_parser("configure", help="startup: write cfg_regulators settings, then STARTUP_RAILS")
    sub.add_parser("config", help="show the rail configuration in use")
    p = sub.add_parser("raw", help="raw I2C: ADDR [WRITE_HEX] --rlen N")
    p.add_argument("raddr"); p.add_argument("whex", nargs="?", default=""); p.add_argument("--rlen", type=int, default=0)
    a = ap.parse_args(argv)

    if a.selftest:
        return 0 if selftest() else 1
    if a.cmd == "config":
        print("source: %s" % CFG_SOURCE)
        for n in RAIL_ORDER:
            r = RAILS[n]
            print("%-7s 0x%02X  %.3f V +/-%.1f%%  %s" % (n, r["addr"], r["nominal"], r["tol"] * 100,
                  ", ".join("%s=%s" % kv for kv in r["settings"].items()) or "no settings"))
        print("startup %s, PEC %s, verify %.1f%%, sweep limit %.0f%%"
              % (STARTUP_RAILS, PEC, VERIFY_TOL * 100, SWEEP_LIMIT * 100))
        return 0
    if a.cmd == "list":
        for n, (c, acc, proto, sz) in sorted(MPQ8785_COMMANDS.items(), key=lambda kv: kv[1][0]):
            print("%02Xh  %-28s %-3s %-5s %d" % (c, n, acc, proto, sz))
        return 0
    if not a.port or not a.cmd:
        ap.error("need a serial port and a command (or --selftest)")
    link = open_link(a.port, a.baud)
    if link is None:
        return 1

    if a.cmd == "scan":
        found = [x for x in range(0x08, 0x78) if link.probe(x)]
        print("ACK at:", " ".join("0x%02X" % x for x in found) or "(none)")
        return 0
    if a.cmd == "raw":
        print(link.i2c(int(a.raddr, 16), bytes.fromhex(a.whex), a.rlen).hex().upper() or "OK")
        return 0
    if a.cmd == "rails":
        res = set_rails(link, a.corner, pec=a.pec or None)
        for r in res:
            print(format_rail(r))
        return 0 if all(r["ok"] for r in res) else 1
    if a.cmd == "configure":
        settings, res = configure_rails(link, pec=a.pec or None)
        for name, field, want, back in settings:
            print("%-7s %-22s %g" % (name, field, back))
        for r in res:
            print(format_rail(r))
        return 0 if all(r["ok"] for r in res) else 1

    addr = REGULATORS[a.reg] if a.reg else a.addr
    if addr is None:
        ap.error("need --addr or --reg")
    dev = MPQ8785(link, addr, pec=a.pec, name=a.reg)
    if a.cmd == "read":
        v = dev.read(a.name.upper())
        print(v.hex().upper() if isinstance(v, bytes) else "0x%X (%d)" % (v, v))
    elif a.cmd == "write":
        dev.write(a.name.upper(), a.value); print("OK")
    elif a.cmd == "vout":
        if a.volts is None:
            print("setpoint %.5f V, measured %.5f V" % (dev.get_vout_setpoint(), dev.read_vout()))
        else:
            print("set %.5f V" % dev.set_vout(a.volts))
    elif a.cmd in ("get", "set"):
        name = a.name.upper()
        if name not in SCALED:
            ap.error("%s has no scale; scaled fields: %s" % (name, ", ".join(sorted(SCALED))))
        v = dev.get(name) if a.cmd == "get" else dev.set(name, a.value)
        print("%s %s%g %s" % (name, "" if a.cmd == "get" else "set ", round(v, 6), SCALED[name][2]))
    elif a.cmd == "telemetry":
        print(dev.telemetry())
    elif a.cmd == "status":
        w, bits = dev.status(); print("STATUS_WORD 0x%04X %s" % (w, bits))
    elif a.cmd == "on":
        dev.on(); print("OK")
    elif a.cmd == "off":
        dev.off(); print("OK")
    elif a.cmd == "clear":
        dev.clear_faults(); print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
