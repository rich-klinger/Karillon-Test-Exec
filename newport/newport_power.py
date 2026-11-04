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
#  Purpose: DUT supply power-up / power-down sequencing and monitoring.
#
#------------------------------------------------------------------------------
"""
newport_power.py -- DUT power sequencing for the NPT100 EVB (2026-10-11).

The two DUT supplies are MPQ8785 bucks (fpga_pmbus.py): VDDIO (U12, 0x22)
and VDDCORE (U10+U11, 2-phase, 0x20). The FPGA drives two lines per rail
(PWR s b, newport_link.set_pwr) and reads each rail's POWER_GOOD (GET PWR):

    ON   -> the regulators' CTRL pin. High = on, as long as ON_OFF_CONFIG
            (02h) lets CTRL control the rail (CTRL only, active high, 0x16
            -- checked and set here). The board's CTRL pull-down holds it
            off while the FPGA isn't driving it (4.7k planned, so
            reconfiguration can't turn a rail on).
    RST  -> the regulators' ALT/RST pin in its RST function (DFh bit 0 = 0,
            checked and set here). LOW = VOUT forced back to VBOOT. The
            FPGA drives it low out of reset, so it's released before the
            output voltage is set.
    GOOD <- the regulators' PG pin (open-drain, pulled up to VDD18).

Power-up (power_up): VDDIO first, then VDDCORE (RAIL_ORDER in
cfg_regulators). For each rail: check it answers over PMBus; make sure
ON_OFF_CONFIG / ALT-RST are as above; release RST; write its settings
(current limits, ramp times...) and its voltage (STARTUP_RAILS in
cfg_regulators: nom, or the low/high corner) while still off;
clear faults; ON; wait for POWER_GOOD (PG_TIMEOUT_S); check READ_VOUT and
STATUS_WORD; wait RAIL_DELAY_S; next rail. VDDCORE is never turned on until
VDDIO's POWER_GOOD is up. Any failure powers everything down again.

Power-down (power_down): the reverse, VDDCORE then VDDIO: ON off, wait for
POWER_GOOD to drop and READ_VOUT to fall below OFF_VOLTS (OFF_TIMEOUT_S),
then both RST lines back low (the FPGA's reset state). Best effort: one
rail failing to answer doesn't stop the other from being turned off.

Monitor (start_monitor): a background thread polls GET PWR every PG_POLL_S.
A POWER_GOOD that doesn't match its ON line -- dropped while on, or up
while off -- once PG_GRACE_S has passed since the last PWR command, is a
fault: testing stops (the link raises PowerFaultError in the running test)
and both supplies are powered down. So is a USB-Blaster on the FPGA's JTAG
header (GET PWR bit 6 / !JTAG 1, proposed -- needs the board's detect pin
and the RTL): a present Blaster blocks DUT power entirely. A fault stays
latched until a manual power-up (menu "DUT power up", fpga> pwrup, or
python newport_power.py up), which clears it if its cause is gone.

Tests: newport_tests runs DUT tests through ensure_power_on(); whether the
rails then stay up between tests or are cycled around each one is
POWER_BETWEEN_TESTS ("keep" / "cycle").

Settings: the POWER dict in cfg\\cfg_regulators.py (defaults below).

Bench use:
    python newport_power.py [--port COM3] up | down | status
    python newport_power.py --debug up        (simulated FPGA)
    python newport_power.py --selftest
"""

import logging
import os
import sys
import threading
import time

import fpga_pmbus as pm
from newport_link import (NewportError, PowerFaultError, PWR_VDDC_ON, PWR_VDDC_RST,
                          PWR_VDDIO_ON, PWR_VDDIO_RST, PWR_VDDC_GOOD, PWR_VDDIO_GOOD,
                          PWR_JTAG_CABLE)

log = logging.getLogger("newport")

# rail -> (ON select, RST select, GET PWR POWER_GOOD bit)
LINES = {
    "vddio":   (PWR_VDDIO_ON, PWR_VDDIO_RST, PWR_VDDIO_GOOD),
    "vddcore": (PWR_VDDC_ON,  PWR_VDDC_RST,  PWR_VDDC_GOOD),
}
ON_ACTIVE = 1          # CTRL high = on (MPQ8785 CTRL, active-high config)
RST_ACTIVE = 0         # ALT/RST low = reset (VOUT back to VBOOT)

ON_OFF_CONFIG_CTRL = 0x16   # 02h: CTRL only (bits 4:2 = 101), active high (bit 1)
STATUS_FAULTS = 0xE135      # STATUS_WORD: VOUT, IOUT, INPUT, WATCH_DOG,
                            # VOUT_OV, IOUT_OC, TEMPERATURE, DRMOS
STATUS_PG = 1 << 11         # STATUS_WORD PG_STATUS

_DEFAULTS = {
    "POWER_BETWEEN_TESTS": "keep",  # "keep": rails stay up between tests; "cycle"
    "PG_TIMEOUT_S": 0.5,            # ON -> POWER_GOOD, per rail
    "RAIL_DELAY_S": 0.0,            # extra wait after one rail is good, before the next
    "OFF_TIMEOUT_S": 0.5,           # ON off -> POWER_GOOD low and VOUT < OFF_VOLTS
    "OFF_VOLTS": 0.10,              # READ_VOUT below this counts as off
    "PG_POLL_S": 0.5,               # monitor: GET PWR every this often
    "PG_GRACE_S": 0.5,              # monitor: ignore POWER_GOOD this long after a PWR command
    "PG_SOURCE": "fpga",            # "fpga": GET PWR's POWER_GOOD bits; "pmbus": STATUS_WORD
    "ON_OFF_CONFIG": ON_OFF_CONFIG_CTRL,  # set 02h to this; None = leave the parts alone
    "RST_FUNCTION": True,           # make ALT/RST the RST input (DFh bit 0 = 0)
    "CABLE_INTERLOCK": True,        # USB-Blaster present (GET PWR bit 6 / !JTAG 1) blocks power
}
CFG = dict(_DEFAULTS)
CFG_SOURCE = "built-in defaults"


def reload_cfg(module="find"):
    """Load the POWER dict from cfg_regulators (or `module`; None = built-in
    defaults only)."""
    global CFG, CFG_SOURCE
    if module == "find":
        module = pm._find_cfg()
    CFG = dict(_DEFAULTS)
    extra = getattr(module, "POWER", None) if module is not None else None
    if extra:
        unknown = set(extra) - set(_DEFAULTS)
        if unknown:
            raise ValueError("cfg_regulators.POWER: unknown key(s) %s" % ", ".join(sorted(unknown)))
        CFG.update(extra)
    if CFG["POWER_BETWEEN_TESTS"] not in ("keep", "cycle"):
        raise ValueError("POWER_BETWEEN_TESTS must be 'keep' or 'cycle'")
    if CFG["PG_SOURCE"] not in ("fpga", "pmbus"):
        raise ValueError("PG_SOURCE must be 'fpga' or 'pmbus'")
    CFG_SOURCE = getattr(module, "__file__", "cfg_regulators") if extra else "built-in defaults"
    return CFG


reload_cfg()


class PowerError(NewportError):
    """A power sequence was refused or failed."""


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
class _Talking:
    """Hold the link for a whole sequence, and let its commands through even
    while a fault is latched (the monitor powers down under a fault)."""

    def __init__(self, link):
        self.link = link

    def __enter__(self):
        self.link.lock.acquire()
        self.link._fault_bypass += 1
        return self.link

    def __exit__(self, *exc):
        self.link._fault_bypass -= 1
        self.link.lock.release()


def read_pwr(link):
    """GET PWR decoded: {line levels, POWER_GOOD per rail, cable}."""
    v = link.get_pwr()
    d = {"raw": v, "cable": bool(v >> PWR_JTAG_CABLE & 1)}
    for rail, (on_sel, rst_sel, good) in LINES.items():
        d[rail] = {"on": (v >> on_sel & 1) == ON_ACTIVE,
                   "rst_released": (v >> rst_sel & 1) != RST_ACTIVE,
                   "good": bool(v >> good & 1)}
    return d


def _pg(link, rail, pwr=None):
    if CFG["PG_SOURCE"] == "pmbus":
        return bool(pm.rail_device(link, rail).read("STATUS_WORD") & STATUS_PG)
    return (pwr or read_pwr(link))[rail]["good"]


def _wait(cond, timeout, step=0.002):
    """Poll cond() until true; seconds taken, or None on timeout."""
    t0 = time.monotonic()
    while True:
        if cond():
            return time.monotonic() - t0
        if time.monotonic() - t0 >= timeout:
            return None
        time.sleep(step)


def _rails():
    return [r for r in pm.RAIL_ORDER if r in LINES]


def check_power_allowed(link):
    """Raise PowerError if DUT power is blocked: a USB-Blaster on the FPGA's
    JTAG header (GET PWR bit 6 or an !JTAG 1 seen)."""
    if not CFG["CABLE_INTERLOCK"]:
        return
    cable = read_pwr(link)["cable"] or bool(link.jtag_cable)
    if cable:
        raise PowerError("USB-Blaster connected to the FPGA's JTAG header: DUT power "
                         "is blocked until it's removed")


def _configure_part(link, rail):
    """ON_OFF_CONFIG and the ALT/RST function, as the sequencing needs them.
    Only called with the rail's ON line off. Returns what was changed."""
    dev = pm.rail_device(link, rail)
    changed = []
    want = CFG["ON_OFF_CONFIG"]
    if want is not None:
        cur = dev.read("ON_OFF_CONFIG")
        if cur != want:
            dev.write("ON_OFF_CONFIG", want)
            changed.append("ON_OFF_CONFIG %02X -> %02X" % (cur, want))
    if CFG["RST_FUNCTION"]:
        cur = dev.read("MFR_CONFIG_B")
        if cur & 1:
            dev.write("MFR_CONFIG_B", cur & ~1)
            changed.append("MFR_CONFIG_B bit0 (ALT -> RST)")
    for c in changed:
        log.info("power: %s %s", rail, c)
    return changed


# ---------------------------------------------------------------------------
# Sequences
# ---------------------------------------------------------------------------
def power_down(link, reason=None, configure=False, quiet=False):
    """VDDCORE off, then VDDIO; then both RST lines low. Best effort: keeps
    going past a rail that won't answer. configure=True also sets the parts'
    ON_OFF_CONFIG / ALT-RST (startup: a part that powers up 'always on' only
    turns off once CTRL controls it). Returns one result dict per rail;
    raises PowerError only if the FPGA itself couldn't be told."""
    out = []
    errors = []
    with _Talking(link):
        if reason:
            log.warning("power: down (%s)", reason)
        for rail in reversed(_rails()):
            on_sel, rst_sel, _ = LINES[rail]
            res = {"rail": rail, "off": False, "meas_V": None, "note": ""}
            try:
                link.set_pwr(on_sel, 1 - ON_ACTIVE)
            except NewportError as e:
                errors.append("%s ON: %s" % (rail, e))
                res["note"] = "ON line not changed: %s" % e
                out.append(res)
                continue
            if configure:
                try:
                    _configure_part(link, rail)
                except NewportError as e:
                    res["note"] = "PMBus: %s" % e
            t = _wait(lambda: not _pg(link, rail), CFG["OFF_TIMEOUT_S"])
            res["off"] = t is not None
            try:
                dev = pm.rail_device(link, rail)
                _wait(lambda: dev.read_vout() < CFG["OFF_VOLTS"], CFG["OFF_TIMEOUT_S"])
                res["meas_V"] = dev.read_vout()
                if res["meas_V"] >= CFG["OFF_VOLTS"]:
                    res["note"] = "VOUT still %.3f V" % res["meas_V"]
            except NewportError as e:
                res["note"] = res["note"] or "PMBus: %s" % e
            if not res["off"]:
                res["note"] = ("POWER_GOOD still high after %g s; " % CFG["OFF_TIMEOUT_S"]) + res["note"]
            out.append(res)
            time.sleep(CFG["RAIL_DELAY_S"])
        for rail in _rails():
            try:
                link.set_pwr(LINES[rail][1], RST_ACTIVE)
            except NewportError as e:
                errors.append("%s RST: %s" % (rail, e))
    for r in out:
        line = "power down: %-7s %s%s" % (r["rail"], "off" if r["off"] else "NOT confirmed off",
                                          ("  (" + r["note"] + ")") if r["note"] else "")
        log.info(line)
        if not quiet:
            print(line)
    if errors:
        raise PowerError("power-down incomplete: " + "; ".join(errors))
    return out


def power_up(link, manual=False, quiet=False):
    """VDDIO up, wait for its POWER_GOOD, then VDDCORE. manual=True (menu,
    fpga> pwrup, CLI) also clears a latched fault once its cause is gone;
    automatic callers get PowerFaultError while a fault is latched. Any
    failure powers both rails down and raises PowerError. Returns one
    result dict per rail."""
    out = []
    with _Talking(link):
        if link.fault and not manual:
            raise PowerFaultError("DUT power is off after a fault (%s) -- run 'DUT power up' "
                                  "(menu) or pwrup (fpga>) to restore it" % link.fault)
        check_power_allowed(link)
        if manual:
            link.fault = None
        pwr = read_pwr(link)
        if any(pwr[r]["on"] for r in _rails()):
            power_down(link, reason="power-up from a partial state", quiet=True)
        try:
            for rail in _rails():
                on_sel, rst_sel, _ = LINES[rail]
                dev = pm.rail_device(link, rail)
                dev.read("STATUS_WORD")                    # answers? (NAK -> error)
                changed = _configure_part(link, rail)
                link.set_pwr(rst_sel, 1 - RST_ACTIVE)      # release RST before VOUT
                pm.apply_settings(link, rails=[rail])
                set_v = pm.set_rail(link, rail, pm.STARTUP_RAILS or "nom", verify=False)["set_V"]
                dev.clear_faults()
                if CFG["CABLE_INTERLOCK"] and link.jtag_cable:
                    raise PowerError("USB-Blaster connected")
                link.set_pwr(on_sel, ON_ACTIVE)
                t = _wait(lambda: _pg(link, rail), CFG["PG_TIMEOUT_S"])
                if t is None:
                    raise PowerError("%s: no POWER_GOOD within %g s" % (rail, CFG["PG_TIMEOUT_S"]))
                meas = dev.read_vout()
                sw = dev.read("STATUS_WORD")
                if abs(meas - set_v) > pm.VERIFY_TOL * set_v:
                    raise PowerError("%s: VOUT %.4f V, set %.4f V (more than %.1f%% off)"
                                     % (rail, meas, set_v, pm.VERIFY_TOL * 100))
                if sw & STATUS_FAULTS:
                    raise PowerError("%s: STATUS_WORD %04X after power-up %s"
                                     % (rail, sw, dev.status()[1]))
                note = "" if sw & STATUS_PG else "  (PMBus PG_STATUS not set)"
                out.append({"rail": rail, "set_V": set_v, "meas_V": meas,
                            "pg_ms": t * 1000, "changed": changed, "note": note})
                time.sleep(CFG["RAIL_DELAY_S"])
        except (NewportError, ValueError) as e:
            try:
                power_down(link, reason="power-up failed: %s" % e, quiet=quiet)
            except NewportError as e2:
                log.error("power: power-down after a failed power-up: %s", e2)
            if isinstance(e, PowerFaultError):
                raise
            raise PowerError("power-up failed, DUT supplies powered down: %s" % e)
    for r in out:
        line = "power up:   %-7s %.4f V (set %.4f V), POWER_GOOD after %.0f ms%s%s" % (
            r["rail"], r["meas_V"], r["set_V"], r["pg_ms"],
            ("  [" + ", ".join(r["changed"]) + "]") if r["changed"] else "", r["note"])
        log.info(line)
        if not quiet:
            print(line)
    return out


def is_on(link, pwr=None):
    """Both rails on, out of reset and POWER_GOOD."""
    pwr = pwr or read_pwr(link)
    return all(pwr[r]["on"] and pwr[r]["rst_released"] and pwr[r]["good"] for r in _rails())


def ensure_power_on(link, quiet=False):
    """Before a DUT test: nothing if both rails are already up and good,
    otherwise the power-up sequence. Refuses while a fault is latched or a
    USB-Blaster is connected."""
    with _Talking(link):
        if link.fault:
            raise PowerFaultError("DUT power is off after a fault (%s) -- run 'DUT power up' "
                                  "(menu) or pwrup (fpga>) to restore it" % link.fault)
        pwr = read_pwr(link)
        if is_on(link, pwr) and not (CFG["CABLE_INTERLOCK"] and pwr["cable"]):
            return []
        return power_up(link, quiet=quiet)


def status(link):
    """Line levels, POWER_GOOD, cable, and each rail's telemetry."""
    pwr = read_pwr(link)
    rails = {}
    for rail in _rails():
        try:
            rails[rail] = pm.rail_device(link, rail).telemetry()
        except NewportError as e:
            rails[rail] = {"error": str(e)}
    return {"pwr": pwr, "rails": rails, "fault": link.fault, "on": is_on(link, pwr)}


def print_status(link, out=None):
    out = out or print
    s = status(link)
    p = s["pwr"]
    out("DUT power: %s%s" % ("ON" if s["on"] else "off",
                             ("   FAULT: " + s["fault"]) if s["fault"] else ""))
    out("USB-Blaster on JTAG header: %s" % ("YES (power blocked)" if p["cable"] else "no"))
    for rail in _rails():
        l, t = p[rail], s["rails"][rail]
        tel = ("error: " + t["error"]) if "error" in t else \
            "VOUT %.4f V  IOUT %.3f A  STATUS %s" % (t["vout_V"], t["iout_A"], t["status_word"])
        out("  %-7s ON=%d  RST=%s  POWER_GOOD=%d   %s" % (
            rail, l["on"], "released" if l["rst_released"] else "held", l["good"], tel))
    return s


# ---------------------------------------------------------------------------
# Monitor
# ---------------------------------------------------------------------------
def _fault(link, why, out=None):
    """Latch a fault, power down, tell the user."""
    out = out or print
    first = link.fault is None
    link.fault = link.fault or why
    log.error("power: FAULT: %s -- testing stopped, DUT supplies powered down", why)
    if first:
        out("\n*** DUT POWER FAULT: %s -- testing stopped, DUT supplies powered down ***" % why)
    try:
        power_down(link, reason=why, quiet=True)
    except NewportError as e:
        log.error("power: power-down after fault failed: %s", e)
        out("*** power-down after the fault failed: %s -- check the board ***" % e)


def check_once(link, out=None):
    """One monitor pass (the thread calls this every PG_POLL_S; tests call
    it directly). Returns the fault text if it found one, else None."""
    with _Talking(link):
        pwr = read_pwr(link)
        if CFG["CABLE_INTERLOCK"] and (pwr["cable"] or link.jtag_cable):
            if any(pwr[r]["on"] for r in _rails()) or link.fault is None:
                _fault(link, "USB-Blaster connected to the FPGA's JTAG header", out)
            return link.fault
        changed = link.pwr_changed_at
        if changed is not None and time.monotonic() - changed < CFG["PG_GRACE_S"]:
            return None                                  # still settling after a PWR
        for rail in _rails():
            on = pwr[rail]["on"] and pwr[rail]["rst_released"]
            good = pwr[rail]["good"] if CFG["PG_SOURCE"] == "fpga" else _pg(link, rail)
            if on and not good:
                _fault(link, "%s POWER_GOOD dropped" % rail.upper(), out)
                return link.fault
            if good and not pwr[rail]["on"]:
                _fault(link, "%s POWER_GOOD high while its supply is off" % rail.upper(), out)
                return link.fault
    return None


class PowerMonitor(threading.Thread):
    """Polls GET PWR every PG_POLL_S; see check_once()."""

    def __init__(self, link, out=None):
        super().__init__(name="newport-power-monitor", daemon=True)
        self.link, self.out = link, out or (lambda m: print(m))
        self.stop_event = threading.Event()
        self.wake = threading.Event()
        self.errors = 0

    def _on_unsolicited(self, kind, value):
        if kind == "jtag" and value and CFG["CABLE_INTERLOCK"]:
            self.link.fault = self.link.fault or "USB-Blaster connected to the FPGA's JTAG header"
        if kind == "jtag":
            self.out("\n*** USB-Blaster %s the FPGA's JTAG header%s ***" % (
                "plugged into" if value else "removed from",
                " -- DUT power blocked" if value else
                " -- run 'DUT power up' (menu) or pwrup (fpga>) to power the DUT again"))
        self.wake.set()

    def run(self):
        self.link.on_unsolicited = self._on_unsolicited
        while not self.stop_event.is_set():
            self.wake.wait(CFG["PG_POLL_S"])
            self.wake.clear()
            if self.stop_event.is_set():
                break
            if not self.link.lock.acquire(timeout=CFG["PG_POLL_S"]):
                continue                     # a long exchange is running; next time
            try:
                check_once(self.link, self.out)
                self.errors = 0
            except NewportError as e:
                self.errors += 1
                log.debug("power monitor: %s", e)
            except Exception as e:           # serial port gone etc.: stop quietly
                log.debug("power monitor stopped: %s", e)
                break
            finally:
                self.link.lock.release()

    def stop(self):
        self.stop_event.set()
        self.wake.set()
        if self.is_alive() and threading.current_thread() is not self:
            self.join(timeout=2.0)
        if self.link.on_unsolicited == self._on_unsolicited:
            self.link.on_unsolicited = None


def start_monitor(link, out=None):
    mon = getattr(link, "power_monitor", None)
    if mon is not None and mon.is_alive():
        return mon
    mon = PowerMonitor(link, out)
    link.power_monitor = mon
    mon.start()
    return mon


def stop_monitor(link):
    mon = getattr(link, "power_monitor", None)
    if mon is not None:
        mon.stop()
        link.power_monitor = None


# ---------------------------------------------------------------------------
# Self-test (simulated FPGA + regulators)
# ---------------------------------------------------------------------------
def selftest():
    saved = dict(CFG)
    cfgmod = pm._find_cfg()
    pm.apply_cfg(None)
    reload_cfg(None)
    try:
        return _selftest()
    finally:
        pm.apply_cfg(cfgmod)
        CFG.clear()
        CFG.update(saved)


def _selftest():
    from newport_link import NewportLink
    from newport_sim import SimFPGA
    fails = 0
    msgs = []

    def check(what, got, want):
        nonlocal fails
        ok = got == want
        fails += not ok
        print("%s  %-50s %r%s" % ("pass" if ok else "FAIL", what, got,
                                  "" if ok else "  (want %r)" % (want,)))

    def raises(what, exc, fn):
        try:
            fn()
            check(what, "no exception", exc.__name__)
        except exc:
            check(what, exc.__name__, exc.__name__)

    def board():
        sim = SimFPGA()
        link = NewportLink("SIM", ser=sim)
        io = sim.i2c_devices[pm.REGULATORS["vddio"]]
        core = sim.i2c_devices[pm.REGULATORS["vddcore"]]
        return sim, link, io, core

    lsb = 2 ** -9
    sim, link, io, core = board()
    io.pg_delay = core.pg_delay = 0                   # both 'always on' and good now
    sim.pwr_out = 0b0101
    sim._apply_pwr()
    check("GET PWR digit order: outputs first, GOOD second",
          (sim._exec("GET PWR"), link.get_pwr() & 0x3F), ("53", 0b110101))
    sim.pwr_out = 0
    sim._apply_pwr()

    sim, link, io, core = board()
    check("parts start 'always on' (ON_OFF_CONFIG 00)", (io.output_on(), core.output_on()), (True, True))
    power_down(link, reason="startup", configure=True, quiet=True)
    check("startup safe-off: CTRL control + both off",
          (io.regs[0x02], core.regs[0x02], io.output_on(), core.output_on()), (0x16, 0x16, False, False))
    check("... RST lines back low", sim.pwr_out, 0)

    sim.pwr_log.clear()
    res = power_up(link, quiet=True)
    order = [l for l, _ in sim.pwr_log]
    check("power-up order", order, ["PWR 3 1", "PWR 2 1", "PWR 1 1", "PWR 0 1"])
    check("VDDCORE ON only once VDDIO POWER_GOOD is up",
          dict(sim.pwr_log)["PWR 0 1"] & 0b10, 0b10)
    check("nominal voltages (RST released first)",
          (io.regs[0x21], core.regs[0x21]), (round(1.8 / lsb), round(0.8 / lsb)))
    check("results in sequence order", [r["rail"] for r in res], ["vddio", "vddcore"])
    check("is_on", is_on(link), True)
    n = len(sim.pwr_log)
    ensure_power_on(link, quiet=True)
    check("ensure_power_on: already on -> no PWR", len(sim.pwr_log), n)
    check("VDDIO gain 0.5 set for 1.8 V", io.regs[0xEB] & 0x400, 0x400)

    io.force_pg_low = True                              # VDDIO PG drops
    link.pwr_changed_at = None
    sim.pwr_log.clear()
    f = check_once(link, msgs.append)
    check("PG drop -> fault", f, "VDDIO POWER_GOOD dropped")
    check("... powered down VDDCORE first", [l for l, _ in sim.pwr_log][:2], ["PWR 0 0", "PWR 2 0"])
    check("... both off", (io.output_on(), core.output_on()), (False, False))
    check("... user told", any("DUT POWER FAULT" in m for m in msgs), True)
    raises("fault latched: ensure_power_on refuses", PowerFaultError, lambda: ensure_power_on(link))
    link.testing = 1
    raises("fault + testing: next command stops the test", PowerFaultError,
           lambda: link.command("GET AFECFG"))
    raises("fault + testing: wait_done stops the test", PowerFaultError,
           lambda: link.wait_done(0, 0.05))
    link.testing = 0
    check("not testing: commands still work", link.command("GET AFECFG"), "000000000000")
    io.force_pg_low = False
    power_up(link, manual=True, quiet=True)
    check("manual power-up clears the fault", (link.fault, is_on(link)), (None, True))

    link.pwr_changed_at = None
    core.force_pg_low = True
    io.force_pg_low = False
    check("VDDCORE PG drop -> fault", check_once(link, msgs.append), "VDDCORE POWER_GOOD dropped")
    core.force_pg_low = False
    link.fault = None

    sim.pwr_log.clear()
    io.force_pg_low = True                              # VDDIO never good
    raises("power-up: no VDDIO POWER_GOOD -> PowerError", PowerError, lambda: power_up(link, quiet=True))
    check("... VDDCORE never turned on", "PWR 0 1" in [l for l, _ in sim.pwr_log], False)
    check("... everything off again", sim.pwr_out & 0b0101, 0)
    io.force_pg_low = False

    core.vout_offset = 40                               # ~78 mV high
    raises("power-up: VDDCORE VOUT off -> PowerError", PowerError, lambda: power_up(link, quiet=True))
    check("... both off", (io.output_on(), core.output_on()), (False, False))
    core.vout_offset = 0

    io.regs[0xDF] |= 1                                  # ALT/RST set to ALT
    power_up(link, quiet=True)
    check("ALT/RST put back to RST", io.regs[0xDF] & 1, 0)

    link.set_pwr(PWR_VDDC_ON, 0)                        # partial state: VDDIO only
    sim.pwr_log.clear()
    link.pwr_changed_at = None
    power_up(link, manual=True, quiet=True)
    check("partial state: down first, then in order",
          [l for l, _ in sim.pwr_log if l.endswith(" 1") and l[4] in "02"], ["PWR 2 1", "PWR 0 1"])

    # USB-Blaster
    msgs.clear()
    mon = start_monitor(link, msgs.append)
    sim.set_jtag_cable(True)                            # FPGA: !JTAG 1 + powers down
    link.command("GET AFECFG")                          # host reads the !JTAG line
    time.sleep(0.15)
    check("!JTAG 1 seen", (link.jtag_cable, link.jtag_events[-1:]), (True, ["!JTAG 1"]))
    check("... fault latched", link.fault, "USB-Blaster connected to the FPGA's JTAG header")
    check("... rails off", (io.output_on(), core.output_on()), (False, False))
    check("... user told", any("plugged into" in m for m in msgs), True)
    raises("cable: power-up refused", PowerError, lambda: power_up(link, manual=True, quiet=True))
    raises("cable: raw PWR 2 1 refused", PowerFaultError, lambda: link.set_pwr(PWR_VDDIO_ON, 1))
    sim.set_jtag_cable(False)
    link.command("GET AFECFG")
    check("!JTAG 0 seen", link.jtag_cable, False)
    power_up(link, manual=True, quiet=True)
    check("cable gone: manual power-up works", is_on(link), True)

    # monitor thread stops a running test
    CFG["PG_POLL_S"] = 0.02
    CFG["PG_GRACE_S"] = 0.0
    mon.wake.set()
    link.testing = 1
    t0 = time.monotonic()
    threading.Timer(0.1, lambda: setattr(io, "force_pg_low", True)).start()
    try:
        link.wait_done(0, 2.0)                          # a 'test' waiting on DONE
        check("monitor stops a running test", "no exception", "PowerFaultError")
    except PowerFaultError:
        check("monitor stops a running test", "PowerFaultError", "PowerFaultError")
    check("... within 0.5 s", time.monotonic() - t0 < 0.5, True)
    check("... rails off", (io.output_on(), core.output_on()), (False, False))
    link.testing = 0
    io.force_pg_low = False
    stop_monitor(link)
    check("monitor stopped", getattr(link, "power_monitor", None), None)
    CFG.update(PG_POLL_S=_DEFAULTS["PG_POLL_S"], PG_GRACE_S=_DEFAULTS["PG_GRACE_S"])

    # bitstream without the cable detect: bit 6 stays 0, no interlock
    sim, link, io, core = board()
    sim.cable_detect = False
    sim.set_jtag_cable(True)
    power_down(link, configure=True, quiet=True)
    power_up(link, quiet=True)
    check("no detect pin: power works", is_on(link), True)

    # settings
    class _Cfg:
        POWER = {"POWER_BETWEEN_TESTS": "cycle", "PG_POLL_S": 0.2}
    reload_cfg(_Cfg)
    check("cfg POWER overrides", (CFG["POWER_BETWEEN_TESTS"], CFG["PG_POLL_S"], CFG["PG_TIMEOUT_S"]),
          ("cycle", 0.2, 0.5))

    class _Bad:
        POWER = {"PG_TIMOUT_S": 1}
    raises("cfg: unknown key refused", ValueError, lambda: reload_cfg(_Bad))
    reload_cfg(None)

    print("SELFTEST", "PASS" if fails == 0 else "FAIL (%d)" % fails)
    return fails == 0


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="DUT power sequencing (VDDIO -> VDDCORE)")
    ap.add_argument("action", nargs="?", choices=["up", "down", "status"], help="what to do")
    ap.add_argument("--port", default=None, help="serial port (default: cfg_com.NEWPORT_port or COM3)")
    ap.add_argument("--debug", action="store_true", help="use the simulated FPGA")
    ap.add_argument("--selftest", action="store_true", help="offline checks of this module")
    a = ap.parse_args(argv)
    if a.selftest:
        return 0 if selftest() else 1
    if not a.action:
        ap.error("need up, down or status (or --selftest)")
    logging.basicConfig(level=logging.WARNING, format="%(message)s")
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
        return 1
    rc = 0
    try:
        if a.action == "up":
            power_up(link, manual=True)
        elif a.action == "down":
            power_down(link, reason="command line")
        print_status(link)
    except NewportError as e:
        print("error: %s" % e)
        rc = 1
    finally:
        nt.close_link(power_down=False)
    return rc


if __name__ == "__main__":
    sys.exit(main())
