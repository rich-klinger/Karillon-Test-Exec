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
#  Purpose: Run the Newport FPGA hardware tests and decode their results.
#
#------------------------------------------------------------------------------
"""
newport_tests.py -- runs newport_eval's hardware test slots over the UART
and decodes their result registers.

The tests themselves are in the FPGA (fpga/newport_eval/TB, wired up in
regfile_tests.v). The host's job for each one:

    SET TESTn 0000      clear the start bit (the FPGA starts on a 0->1 edge)
    START TESTn         start it
    wait for DONE TESTn (falling back to polling GET TESTn_RES)
    GET TESTn_RES       read the result register and decode it

Slots (fpga/newport_eval/README.md has the result-register layouts):

    TEST0  basic        SOF + 55 bytes + EOF through the DUT      pass/fail
    TEST1  short        10-byte payload                           pass/fail
    TEST2  break        BREAK fault byte                          pass/fail
    TEST3  error        ERROR fault byte                          pass/fail
    TEST4  pauseErr     PAUSE fault byte                          pass/fail
    TEST5  singleEnded  single-ended / GPIO path                  pass/fail
    TEST6  ioConfig     DFE config write / readback               pass/fail
    TEST7  gpioRx       GPIO-bypass receive     exercise only: passes if it
    TEST8  gpioTx       GPIO-bypass transmit    completes without a timeout
    TEST9  packets      host-loaded 256-byte packets (PKL) through  pass/fail
                        the DUT, loopback checked against the RAM
                        -- see newport_pkt.py; not in run_all (it sends
                        whatever is loaded, all zeros after power-up)
    TEST10              earmarked for part-screening (newport_screen.py);
                        FPGA side not written yet -- still answers 0xBFFF
                        like TEST11-19, decoded as 'not implemented', not
                        pass/fail (see decode()'s 'screen' layout)
    TEST11-19           reserved, always 0xBFFF

TEST0-5 and TEST9 need the real Newport ASIC on Interface 1; on a bare dev kit they
report a timeout.

The menu entries used by Karillon_TestExec are the functions at the bottom
(slot_test(n), run_all, link_check, regulator_telemetry): each takes the DUT
serial number and returns 0 for pass, 1 for fail, like every other test.

Standalone (bench) use:
    python newport_tests.py                 run TEST0-8 on COM3
    python newport_tests.py --port COM5 0 6 run TEST0 and TEST6
    python newport_tests.py --debug         the same against the simulated FPGA
    python newport_tests.py --selftest      offline checks of this module
"""

import csv
import datetime
import logging
import os
import re
import sys
import time

from newport_link import open_link, NewportError, NakError, PowerFaultError, debug_mode

log = logging.getLogger("newport")

DEFAULT_PORT = "COM3"
DONE_TIMEOUT = 1.0          # s; every current test finishes in < 1 ms
RESULTS_CSV = os.path.join("logs", "newport_results.csv")

# slot -> (name, TB source, result-register layout)
TESTS = {
    0: ("basic",       "dfe_basic_test.v",            "stream"),
    1: ("short",       "dfe_stream_test.v (short)",   "stream"),
    2: ("break",       "dfe_stream_test.v (break)",   "stream"),
    3: ("error",       "dfe_stream_test.v (error)",   "stream"),
    4: ("pauseErr",    "dfe_stream_test.v (pauseErr)", "stream"),
    5: ("singleEnded", "dfe_stream_test.v (singleEnded)", "stream"),
    6: ("ioConfig",    "dfe_config_test.v",           "config"),
    7: ("gpioRx",      "dfe_gpio_rx_test.v",          "gpio_rx"),
    8: ("gpioTx",      "dfe_gpio_tx_test.v",          "gpio_tx"),
    9: ("packets",     "pkt_tx_test.v",               "pkt"),
}
for _n in range(10, 20):
    TESTS[_n] = ("reserved", "-", "reserved")

REAL_TESTS = list(range(0, 9))
NOT_IMPLEMENTED = 0xBFFF

# TEST10 is earmarked for the part-screening test (added 2026-10-09; see
# newport_screen.py and fpga/newport_eval/regfile_tests.v). The FPGA side
# isn't written yet -- it's still just the generic g_reserved branch,
# answering 0xBFFF -- so this overrides TESTS[10] from the generic loop
# above with a distinct name/layout. decode()'s 'screen' layout reports
# the 0xBFFF sentinel honestly as 'not implemented' (passed=None), not as
# a vacuous pass the way the generic 'reserved' layout would. Once real
# RTL lands there, it's expected to use the same done/pass/timeout/id
# convention as every other test here (TEST0-9) -- see decode() below.
PARTSCREEN_TEST = 10
TESTS[PARTSCREEN_TEST] = ("partScreen",
                          "TBD -- FPGA screening logic not written yet",
                          "screen")


# ---------------------------------------------------------------------------
# Result decoding
# ---------------------------------------------------------------------------
def _bit(v, b):
    return bool(v >> b & 1)


def decode(n, value):
    """
    Decode TESTn's result register. Returns a dict with the named fields,
    'passed' (bool) and 'summary' (one line of text).

    TEST7/TEST8 have no PASS bit yet (exercise-only); they count as passed
    when they finish without a timeout. See newport/TODO.md.
    """
    name, _, layout = TESTS[n]
    d = {"test": n, "name": name, "raw": value, "done": _bit(value, 15)}
    problems = []

    if layout == "reserved":
        d["passed"] = value == NOT_IMPLEMENTED
        d["summary"] = ("not implemented (0xBFFF, as expected)" if d["passed"]
                        else "unexpected value for a reserved slot")
        return d

    if layout == "screen":
        # TEST10, earmarked for part-screening -- see the PARTSCREEN_TEST
        # comment above. The placeholder FPGA still answers the generic
        # reserved-slot sentinel; report that as 'not implemented' rather
        # than a pass. 'passed' is None here (not True/False) specifically
        # so a caller can't mistake a placeholder result for a real one.
        if value == NOT_IMPLEMENTED:
            d["passed"] = None
            d["summary"] = ("NOT IMPLEMENTED (0xBFFF) -- FPGA screening "
                            "logic not written yet")
            return d
        # A real result, once there is one: same done/pass/timeout/id
        # shape as every other test (4-bit id, like TEST9's 'pkt' layout,
        # since slot numbers go past 9).
        d["pass_bit"] = _bit(value, 14)
        d["timeout"] = _bit(value, 13)
        d["id"] = value & 0xF
        d["passed"] = d["done"] and d["pass_bit"] and not d["timeout"]

    if layout in ("stream", "config"):
        d["pass_bit"] = _bit(value, 14)
        d["timeout"] = _bit(value, 13)
        d["mismatches"] = value >> 3 & 0xFF
        d["id"] = value & 0x7
        if layout == "stream":
            d["sof_ok"] = _bit(value, 12)
            d["eof_ok"] = _bit(value, 11)
            if not d["sof_ok"]:
                problems.append("SOF bad")
            if not d["eof_ok"]:
                problems.append("EOF bad")
            what = "data"
        else:
            what = "CONFIG_RD"
        if d["mismatches"]:
            problems.append("%d %s mismatches" % (d["mismatches"], what))
        d["passed"] = d["done"] and d["pass_bit"] and not d["timeout"]

    elif layout == "pkt":
        # [14] pass [13] timeout [12] framing [11:4] mismatches [3:0] id
        d["pass_bit"] = _bit(value, 14)
        d["timeout"] = _bit(value, 13)
        d["framing"] = _bit(value, 12)
        d["mismatches"] = value >> 4 & 0xFF
        d["id"] = value & 0xF
        if d["framing"]:
            problems.append("SOF/EOF bad")
        if d["mismatches"]:
            problems.append("%s byte mismatches" % ("255+" if d["mismatches"] == 255
                                                    else d["mismatches"]))
        d["passed"] = d["done"] and d["pass_bit"] and not d["timeout"]

    elif layout == "gpio_rx":
        d["timeout"] = _bit(value, 13)
        d["sample"] = value >> 9 & 0xF
        d["samples"] = value >> 3 & 0x3F
        d["id"] = value & 0x7
        d["passed"] = d["done"] and not d["timeout"]
        problems.append("GPIO_SAMPLE last=%s (%d samples)"
                        % (format(d["sample"], "04b"), d["samples"]))

    elif layout == "gpio_tx":
        d["timeout"] = False
        d["id"] = value & 0xF
        d["passed"] = d["done"]

    if d["timeout"]:
        problems.insert(0, "TIMEOUT")
    if d["id"] != n:
        problems.append("ID field %d, expected %d" % (d["id"], n))
        d["passed"] = False
    if not d["done"]:
        problems.insert(0, "not done")

    verdict = "PASS" if d["passed"] else "FAIL"
    if layout in ("gpio_rx", "gpio_tx") and d["passed"]:
        verdict = "DONE (exercise only)"
    d["summary"] = verdict + ("" if not problems else " - " + ", ".join(problems))
    return d


# ---------------------------------------------------------------------------
# Running a test
# ---------------------------------------------------------------------------
def run_test(link, n, timeout=DONE_TIMEOUT, cfg=0):
    """Run TESTn once and return decode()'s dict, plus 'done_msg' (whether
    the unsolicited DONE TESTn arrived) and 'seconds'. cfg = config bits
    14:0 for tests that take any (TEST9); bit 15 is START's."""
    t0 = time.monotonic()
    link.set_cfg(n, cfg & 0x7FFF)       # start bit clear, so the next START is an edge
    link.clear_done(n)
    link.start(n)
    got_done = link.wait_done(n, timeout)
    value = link.get_res(n)
    if not got_done:                     # no DONE line: poll the done bit instead
        end = time.monotonic() + timeout
        while not value & 0x8000 and time.monotonic() < end:
            value = link.get_res(n)
    d = decode(n, value)
    d["done_msg"] = got_done
    d["seconds"] = time.monotonic() - t0
    if not got_done:
        d["summary"] += " (no DONE message)"
    return d


def _report(d, serial_number):
    line = "TEST%-2d %-12s 0x%04X  %s" % (d["test"], d["name"], d["raw"], d["summary"])
    print(line)
    log.info("SN %s  %s", serial_number, line)
    _append_csv(serial_number, d)


def _append_csv(serial_number, d):
    try:
        os.makedirs(os.path.dirname(RESULTS_CSV), exist_ok=True)
        new = not os.path.exists(RESULTS_CSV)
        with open(RESULTS_CSV, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["time", "serial", "test", "name", "result_hex",
                            "passed", "summary", "sim"])
            w.writerow([datetime.datetime.now().isoformat(timespec="seconds"),
                        serial_number, d["test"], d["name"], "%04X" % d["raw"],
                        int(d["passed"]), d["summary"], int(debug_mode())])
    except OSError as e:
        log.warning("couldn't write %s: %s", RESULTS_CSV, e)


# ---------------------------------------------------------------------------
# One link for the whole session
# ---------------------------------------------------------------------------
_link = None
_port = None
_rails_started = False


def default_port():
    """cfg_com.NEWPORT_port if the Test Exec's cfg folder is on the path,
    otherwise COM3."""
    try:
        import cfg_com
        return getattr(cfg_com, "NEWPORT_port", DEFAULT_PORT)
    except ImportError:
        return DEFAULT_PORT


def set_port(port):
    global _port
    _port = port


def get_link():
    """The session's link, opened on first use (asks for another port if the
    configured one doesn't answer). None if the user cancelled. Every time
    it opens, the DUT supplies are put in their safe state and the power
    monitor is started (_startup_rails)."""
    global _link, _port, _rails_started
    if _link is None:
        _link = open_link(_port or default_port())
        if _link is not None:
            _port = _link.port           # remember a port picked by hand
            _rails_started = True
            _startup_rails(_link)
    return _link


def _startup_rails(link):
    """Startup (2026-10-11): DUT supplies off in sequence, with the parts set
    so CTRL controls them (newport_power.power_down(configure=True)); the
    cfg_regulators settings (current limits etc.) written; the POWER_GOOD /
    USB-Blaster monitor started. Power comes on when a DUT test needs it
    (newport_power.ensure_power_on) or from "DUT power up". A regulator
    problem is reported but doesn't stop the link from being used."""
    import fpga_pmbus
    import newport_power
    try:
        newport_power.power_down(link, reason="startup: safe state", configure=True, quiet=True)
        settings = fpga_pmbus.apply_settings(link)
        log.info("startup: rail config from %s", fpga_pmbus.CFG_SOURCE)
        for name, field, want, back in settings:
            log.info("startup: %s %s = %g", name, field, back)
        print("Startup: DUT supplies off (safe state); power-up is VDDIO, then VDDCORE")
    except NewportError as e:
        print("Startup: DUT supplies not configured (%s)" % e)
        log.warning("startup: DUT supplies not configured: %s", e)
    newport_power.start_monitor(link)


def close_link(power_down=True):
    """Close the session's link. By default the DUT supplies are powered
    down in sequence first (leaving the Test Exec turns the DUT off); the
    power monitor is stopped either way."""
    global _link
    if _link is not None:
        import newport_power
        newport_power.stop_monitor(_link)
        if power_down:
            try:
                newport_power.power_down(_link, reason="closing the link", quiet=True)
            except Exception as e:           # FPGA gone: nothing more to do
                log.warning("power-down on close: %s", e)
        _link.close()
        _link = None


def power_fault():
    """The latched DUT power fault (text), or None. The Test Exec stops a
    selected sequence when this is set."""
    return _link.fault if _link is not None else None


def _with_link(fn, serial_number, power=False):
    """Run fn(link, sn) on the session's link. power=True (DUT tests):
    the DUT supplies are brought up first if they aren't (ensure_power_on),
    a power fault during fn stops it (PowerFaultError), and with
    POWER_BETWEEN_TESTS = "cycle" they're powered down afterwards."""
    import newport_power
    link = get_link()
    if link is None:
        print("Newport FPGA not connected - test not run")
        return 1
    try:
        if not power:
            return fn(link, serial_number)
        newport_power.ensure_power_on(link)
        link.testing += 1
        try:
            return fn(link, serial_number)
        finally:
            link.testing -= 1
            if newport_power.CFG["POWER_BETWEEN_TESTS"] == "cycle" and not link.fault:
                newport_power.power_down(link, reason="between tests (cycle)", quiet=True)
    except (PowerFaultError, newport_power.PowerError) as e:
        print("DUT power: %s - test stopped" % e)
        log.error("SN %s  DUT power: %s", serial_number, e)
        return 1
    except NewportError as e:
        print("Newport FPGA error: %s" % e)
        log.error("Newport FPGA error: %s", e)
        close_link()                     # reopen (and re-check) next time
        return 1


# ---------------------------------------------------------------------------
# Menu entries: f(serial_number) -> 0 pass / 1 fail
# ---------------------------------------------------------------------------
def slot_test(n):
    """Menu entry that runs TESTn once."""
    def entry(serial_number):
        def body(link, sn):
            d = run_test(link, n)
            _report(d, sn)
            return 0 if d["passed"] else 1
        return _with_link(body, serial_number, power=True)
    entry.__name__ = "newport_test%d" % n
    return entry


def run_all(serial_number):
    """Run TEST0-8 in order and print a summary."""
    def body(link, sn):
        failed = []
        for n in REAL_TESTS:
            d = run_test(link, n)
            _report(d, sn)
            if not d["passed"]:
                failed.append(n)
        print("Newport tests: %d of %d passed%s"
              % (len(REAL_TESTS) - len(failed), len(REAL_TESTS),
                 "" if not failed else "; failed: " +
                 ", ".join("TEST%d" % n for n in failed)))
        return 0 if not failed else 1
    return _with_link(body, serial_number, power=True)


def link_check(serial_number):
    """Confirm the UART and bitstream: GET AFECFG, a config register
    write/readback, and a reserved slot's 0xBFFF + DONE round trip."""
    def body(link, sn):
        ok = True
        afe = link.get_afecfg()
        print("Port %s: GET AFECFG = %012X" % (link.port, afe))
        link.set_cfg(19, 0x1234)
        cfg = link.get_cfg(19)
        good = cfg == 0x1234
        ok &= good
        print("TEST19 config write/readback 1234 -> %04X  %s"
              % (cfg, "ok" if good else "MISMATCH"))
        d = run_test(link, 19)
        good = d["passed"] and d["done_msg"]
        ok &= good
        print("TEST19 run: result %04X, DONE message %s  %s"
              % (d["raw"], "seen" if d["done_msg"] else "MISSING",
                 "ok" if good else "FAIL"))
        link.set_cfg(19, 0x0000)
        print("Link check %s" % ("passed" if ok else "FAILED"))
        log.info("SN %s  link check %s on %s", sn, "passed" if ok else "FAILED", link.port)
        return 0 if ok else 1
    return _with_link(body, serial_number)


def regulator_telemetry(serial_number):
    """Read VIN/VOUT/IOUT/temperature/STATUS_WORD from each MPQ8785."""
    from fpga_pmbus import MPQ8785, REGULATORS

    def body(link, sn):
        ok = True
        for name, addr in sorted(REGULATORS.items()):
            try:
                t = MPQ8785(link, addr, name=name).telemetry()
                print("%-7s 0x%02X  VIN %6.3f V  VOUT %7.4f V  IOUT %6.3f A  "
                      "TEMP %4.0f C  STATUS %s" % (name, addr, t["vin_V"], t["vout_V"],
                                                   t["iout_A"], t["temp_C"], t["status_word"]))
                log.info("SN %s  %s 0x%02X %s", sn, name, addr, t)
            except NakError:
                ok = False
                print("%-7s 0x%02X  no response (NAK) - check the address in "
                      "fpga_pmbus.REGULATORS" % (name, addr))
        return 0 if ok else 1
    return _with_link(body, serial_number)


# ---------------------------------------------------------------------------
# DUT supply rails: set, and sweep with tests at every point
# ---------------------------------------------------------------------------
SWEEP_CSV = os.path.join("logs", "newport_sweep.csv")


def _rail_cols():
    from fpga_pmbus import RAIL_ORDER
    cols = []
    for r in RAIL_ORDER:
        cols += [r + "_set_V", r + "_pct", r + "_meas_V"]
    return cols


def _append_sweep_csv(serial_number, point_no, rails, d):
    """One row per test per sweep point (d is None when the tests were
    skipped because a rail didn't verify)."""
    try:
        os.makedirs(os.path.dirname(SWEEP_CSV) or ".", exist_ok=True)
        new = not os.path.exists(SWEEP_CSV)
        from fpga_pmbus import RAIL_ORDER
        by_rail = {x["rail"]: x for x in rails}
        vals = []
        for r in RAIL_ORDER:
            x = by_rail.get(r, {})
            vals += ["" if x.get("set_V") is None else "%.5f" % x["set_V"],
                     "" if x.get("pct") is None else "%+.2f" % x["pct"],
                     "" if x.get("meas_V") is None else "%.5f" % x["meas_V"]]
        with open(SWEEP_CSV, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["time", "serial", "point"] + _rail_cols() +
                           ["test", "name", "result_hex", "passed", "summary", "sim"])
            row = [datetime.datetime.now().isoformat(timespec="seconds"), serial_number, point_no] + vals
            if d is None:
                row += ["", "", "", 0, "skipped - rail out of tolerance", int(debug_mode())]
            else:
                row += [d["test"], d["name"], "%04X" % d["raw"], int(d["passed"]),
                        d["summary"], int(debug_mode())]
            w.writerow(row)
    except OSError as e:
        log.warning("couldn't write %s: %s", SWEEP_CSV, e)


def run_sweep(link, points, tests=None, serial_number="bench", quiet=False):
    """For each point (a fpga_pmbus.set_rails() target, e.g.
    {"vddcore": 0.776, "vddio": "nom"}): set the rails, check READ_VOUT,
    run `tests` (default: REAL_TESTS), log one CSV row per test, then step
    to the next point. If a rail doesn't verify, that point's tests are
    skipped and counted as failed. The rails go back to nominal at the end,
    even if the sweep stops on an error.

    Returns a list of {point, rails, results, passed} per point."""
    from fpga_pmbus import set_rails, format_rail
    tests = REAL_TESTS if tests is None else list(tests)
    out = []
    warned_off = False
    try:
        for i, p in enumerate(points, 1):
            rails = set_rails(link, p)
            rails_ok = all(r["ok"] for r in rails)
            if not warned_off and any(r["meas_V"] is None for r in rails):
                warned_off = True
                print("Note: output off on %s - setpoints loaded but not reaching the DUT"
                      % ", ".join(r["rail"] for r in rails if r["meas_V"] is None))
            results = []
            if rails_ok:
                for n in tests:
                    d = run_test(link, n)
                    results.append(d)
                    _append_sweep_csv(serial_number, i, rails, d)
                    if not quiet and not d["passed"]:
                        print("    TEST%-2d %-12s 0x%04X  %s" % (n, d["name"], d["raw"], d["summary"]))
            else:
                _append_sweep_csv(serial_number, i, rails, None)
            passed = rails_ok and all(d["passed"] for d in results)
            npass = sum(d["passed"] for d in results)
            line = "Point %d/%d  %s  %s" % (
                i, len(points), "  ".join("%s %.4f V (%+.2f%%)" % (r["rail"], r["set_V"], r["pct"]) for r in rails),
                ("tests %d/%d passed" % (npass, len(tests))) if rails_ok else "RAIL OUT OF TOLERANCE - tests skipped")
            if not quiet:
                print(line)
                if not rails_ok:
                    for r in rails:
                        print("    " + format_rail(r))
            log.info("SN %s  sweep %s", serial_number, line)
            out.append({"point": p, "rails": rails, "results": results, "passed": passed})
    finally:
        try:
            set_rails(link, "nom")
        except NewportError as e:
            print("Couldn't return the rails to nominal: %s" % e)
            log.error("sweep: couldn't return the rails to nominal: %s", e)
    return out


def _sweep_summary(out):
    bad = [i for i, x in enumerate(out, 1) if not x["passed"]]
    print("Sweep: %d of %d points passed%s" % (len(out) - len(bad), len(out),
          "" if not bad else "; failed points: " + ", ".join(str(i) for i in bad)))
    return 0 if not bad else 1


def rails_set(target):
    """Menu entry: set the DUT rails to target ('nom', 'low', 'high' or a
    {rail: target} dict) and show the result."""
    def entry(serial_number):
        from fpga_pmbus import set_rails, format_rail

        def body(link, sn):
            res = set_rails(link, target)
            for r in res:
                print(format_rail(r))
                log.info("SN %s  %s", sn, format_rail(r))
            return 0 if all(r["ok"] for r in res) else 1
        return _with_link(body, serial_number, power=True)
    entry.__name__ = "rails_%s" % (target if isinstance(target, str) else "custom")
    return entry


# ---------------------------------------------------------------------------
# DUT power-sequencing control (PWR / GET PWR, added 2026-10-09; 2
# regulator POWER_GOOD inputs added to GET PWR 2026-10-09 part 2)
# ---------------------------------------------------------------------------
_PWR_NAMES = {
    0: "DUT_VDDC_PSU_ON",
    1: "DUT_VDDC_PSU_RST",
    2: "DUT_VDDIO_PSU_ON",
    3: "DUT_VDDIO_PSU_RST",
    4: "DUT_VDDC_PWR_GOOD",    # read-only, no SET (2026-10-09 part 2)
    5: "DUT_VDDIO_PWR_GOOD",   # read-only, no SET (2026-10-09 part 2)
}


def dut_power_set(sel, level):
    """Menu entry: PWR sel level -- set/clear one of the 4 DUT power-
    sequencing control outputs (DUT_VDDC_PSU_ON/_RST, DUT_VDDIO_PSU_ON/
    _RST on the FPGA; see newport_eval_top.v). sel is one of
    newport_link.PWR_VDDC_ON/PWR_VDDC_RST/PWR_VDDIO_ON/PWR_VDDIO_RST.
    Drives the pin to exactly the level given -- active-high vs.
    active-low polarity for ON/RST is NOT assumed here, same as jrst();
    that is the eval board's convention and is not yet confirmed against
    the real schematic (see NPT100-EVB-FPGA-Connections-001 Rev 1.9
    Section 1.6)."""
    def entry(serial_number):
        def body(link, sn):
            link.set_pwr(sel, level)
            after = link.get_pwr()
            name = _PWR_NAMES.get(sel, "sel %d" % sel)
            print("%s -> %d  (GET PWR = %X)" % (name, 1 if level else 0, after))
            log.info("SN %s  %s -> %d", sn, name, 1 if level else 0)
            return 0
        return _with_link(body, serial_number)
    entry.__name__ = "dut_power_%s_%d" % (_PWR_NAMES.get(sel, str(sel)).lower(), 1 if level else 0)
    return entry


def dut_power_status(serial_number):
    """Menu entry: DUT power status -- the 4 control lines, the 2 POWER_GOOD
    inputs, the USB-Blaster detect, any latched fault, and each rail's
    VOUT/IOUT/STATUS_WORD (newport_power.print_status)."""
    import newport_power

    def body(link, sn):
        newport_power.print_status(link)
        return 0
    return _with_link(body, serial_number)


def dut_power_up(serial_number):
    """Menu entry: DUT power-up sequence -- VDDIO, wait for its POWER_GOOD,
    then VDDCORE (newport_power.power_up, manual: clears a latched fault
    once its cause is gone)."""
    import newport_power

    def body(link, sn):
        newport_power.power_up(link, manual=True)
        log.info("SN %s  DUT power up", sn)
        return 0
    return _with_link(body, serial_number)


def dut_power_down(serial_number):
    """Menu entry: DUT power-down sequence -- VDDCORE, then VDDIO."""
    import newport_power

    def body(link, sn):
        res = newport_power.power_down(link, reason="menu")
        log.info("SN %s  DUT power down", sn)
        return 0 if all(r["off"] for r in res) else 1
    return _with_link(body, serial_number)


def sweep_corners(tests=None):
    """Menu entry: run `tests` at every low/nom/high combination of the rails."""
    def entry(serial_number):
        from fpga_pmbus import corner_points

        def body(link, sn):
            return _sweep_summary(run_sweep(link, corner_points(), tests, sn))
        return _with_link(body, serial_number, power=True)
    entry.__name__ = "sweep_corners"
    return entry


def sweep_rail(rail, lo_pct, hi_pct, step_pct, tests=None, others="nom"):
    """Menu entry: step one rail from lo_pct to hi_pct (% of nominal) and run
    `tests` at each step, the other rails held at `others`."""
    def entry(serial_number):
        from fpga_pmbus import sweep_points

        def body(link, sn):
            pts = sweep_points(rail, lo_pct, hi_pct, step_pct, others)
            return _sweep_summary(run_sweep(link, pts, tests, sn))
        return _with_link(body, serial_number, power=True)
    entry.__name__ = "sweep_%s" % rail
    return entry


# ---------------------------------------------------------------------------
# Offline self-test against the simulated FPGA
# ---------------------------------------------------------------------------
def selftest():
    from newport_link import NewportLink
    from newport_sim import SimFPGA
    fails = 0

    def check(what, got, want):
        nonlocal fails
        ok = got == want
        fails += not ok
        print("%s  %-44s %r%s" % ("pass" if ok else "FAIL", what, got,
                                  "" if ok else "  (want %r)" % (want,)))

    sim = SimFPGA()
    link = NewportLink("SIM", ser=sim)
    for n in range(20):
        d = run_test(link, n, timeout=0.2)
        if n == PARTSCREEN_TEST:
            # TEST10's placeholder FPGA answers the generic reserved-slot
            # sentinel too, but the "screen" decode layout (added
            # 2026-10-09, see PARTSCREEN_TEST above) reports that honestly
            # as "not implemented" (passed=None) rather than the generic
            # "reserved" layout's vacuous True -- so this one slot doesn't
            # fit the blanket "default passes" check every other slot gets.
            check("TEST%d default (screen placeholder) -> not implemented" % n,
                  d["passed"], None)
        else:
            check("TEST%d default passes" % n, d["passed"], True)
    check("DONE message used", run_test(link, 0)["done_msg"], True)
    check("rerun works (start re-armed)", run_test(link, 0)["passed"], True)
    check("SET 0000 then START sent", sim.lines_seen[-3:-1], ["SET TEST0 0000", "START TEST0"])
    sim.set_result(3, 0x8000 | 0x2000 | 0x1000 | (5 << 3) | 3)
    d = run_test(link, 3)
    check("TEST3 timeout+EOF bad fails", (d["passed"], d["summary"]),
          (False, "FAIL - TIMEOUT, EOF bad, 5 data mismatches"))
    sim.set_result(6, 0xC005)
    check("TEST6 wrong ID fails", run_test(link, 6)["passed"], False)
    sim.set_result(7, 0x8000 | 0x2000 | 7)
    check("TEST7 timeout fails", run_test(link, 7)["passed"], False)
    sim.set_result(10, 0x800A)
    check("TEST10 (screen) non-BFFF, pass bit clear -> fails",
          run_test(link, 10)["passed"], False)
    sim.set_result(10, None)
    check("TEST10 (screen) default 0xBFFF -> not implemented, not pass/fail",
          run_test(link, 10)["passed"], None)
    check("TEST10 (screen) 0xBFFF summary says not implemented",
          "NOT IMPLEMENTED" in decode(10, 0xBFFF)["summary"], True)
    check("TEST10 (screen) pass-shaped value (done+pass+id=10) -> passes",
          decode(10, 0x8000 | 0x4000 | 10)["passed"], True)
    check("TEST10 (screen) wrong id fails",
          decode(10, 0x8000 | 0x4000 | 3)["passed"], False)
    check("decode TEST9 pass", decode(9, 0xC009)["summary"], "PASS")
    check("decode TEST9 one bad byte", decode(9, 0x8019)["summary"], "FAIL - 1 byte mismatches")
    check("decode TEST9 framing", decode(9, 0x9019)["summary"], "FAIL - SOF/EOF bad, 1 byte mismatches")
    check("decode TEST9 timeout", decode(9, 0xA009)["summary"], "FAIL - TIMEOUT")
    check("TEST9 cfg goes out without start, then START",
          (run_test(link, 9, cfg=0x4038)["passed"], sim.lines_seen[-3:-1]),
          (True, ["SET TEST9 4038", "START TEST9"]))
    sim.set_result(0, None)
    sim.no_done.add(0)
    d = run_test(link, 0, timeout=0.05)
    check("no DONE: polled result still passes", (d["passed"], d["done_msg"]), (True, False))
    sim.no_done.clear()
    check("GET AFECFG", link.get_afecfg(), 0)
    check("decode TEST7 sample", decode(7, 0x8000 | (0b1100 << 9) | (9 << 3) | 7)["summary"],
          "DONE (exercise only) - GPIO_SAMPLE last=1100 (9 samples)")
    check("decode TEST8", decode(8, 0x8008)["summary"], "DONE (exercise only)")
    check("decode not done", decode(0, 0x0000)["summary"].split(" - ")[0], "FAIL")
    # DONE lines arriving in the middle of an echo don't confuse the link
    sim.set_result(1, None)
    sim._out += b"DONE TEST5\r\n"
    check("stray DONE before a command", link.get_cfg(1) & 0x7FFF, 0)
    check("stray DONE recorded", "DONE TEST5" in link.done_messages, True)
    _selftest_sweep(check)              # counts its failures through check()
    _selftest_time(check)
    _selftest_port(check)
    print("SELFTEST", "PASS" if fails == 0 else "FAIL (%d)" % fails)
    return fails == 0


def _selftest_port(check):
    """save_port() / open_link() remembering a port picked by hand."""
    import builtins
    import stat
    import tempfile
    import newport_link as nl
    from newport_sim import SimFPGA
    d = tempfile.mkdtemp()
    p = os.path.join(d, "cfg_com.py")
    head = b"# header\r\nDMM1_port = 'com3'\r\n"
    tail = b"\r\n# after\r\nTCB_port = 'com3'\r\n"
    with open(p, "wb") as f:
        f.write(head + b"NEWPORT_port = 'COM3'" + tail)
    nl.save_port("COM7", p)
    with open(p, "rb") as f:
        got = f.read()
    check("save_port: only NEWPORT_port changes (CRLF kept)", got, head + b"NEWPORT_port = 'COM7'" + tail)
    mtime = os.path.getmtime(p)
    check("save_port: same port (any case) -> no write", (nl.save_port("com7", p), os.path.getmtime(p)), (None, mtime))
    with open(p, "wb") as f:
        f.write(b'x = 1\nNEWPORT_port  =  "COM3"   # FT230X\ny = 2\n')
    nl.save_port("COM12", p)
    with open(p, "rb") as f:
        check("save_port: spacing, quotes, comment, LF kept", f.read(),
              b'x = 1\nNEWPORT_port  =  "COM12"   # FT230X\ny = 2\n')
    with open(p, "wb") as f:
        f.write(b"A = 1\r\nB = 2")                  # no NEWPORT_port, no final EOL
    nl.save_port("COM5", p)
    with open(p, "rb") as f:
        check("save_port: line added when missing", f.read(), b"A = 1\r\nB = 2\r\nNEWPORT_port = 'COM5'\r\n")
    check("save_port: no cfg_com.py -> None", nl.save_port("COM5", os.path.join(d, "none.py")), None)
    if os.name != "nt" and os.geteuid() != 0:
        os.chmod(d, stat.S_IRUSR | stat.S_IXUSR)
        check("save_port: read-only folder reported, not raised", nl.save_port("COM9", p), None)
        os.chmod(d, stat.S_IRWXU)

    # open_link: configured port dead, user picks COM8 from the list -> saved
    with open(p, "wb") as f:
        f.write(b"NEWPORT_port = 'COM3'\r\n")
    saved = (nl.try_port, nl.list_ports, nl.find_cfg_com, builtins.input)
    try:
        nl.try_port = lambda port, baud=nl.BAUD, timeout=1.0: (
            (nl.NewportLink(port, ser=SimFPGA(port)), None) if port == "COM8" else (None, "no answer"))
        nl.list_ports = lambda: [("COM3", "USB Serial"), ("COM8", "FT230X")]
        nl.find_cfg_com = lambda: p
        builtins.input = lambda prompt="": "1"
        link = nl.open_link("COM3", debug=False)
        with open(p, "rb") as f:
            check("open_link: port picked by hand is saved", (link.port, f.read()),
                  ("COM8", b"NEWPORT_port = 'COM8'\r\n"))
        m = os.path.getmtime(p)
        nl.open_link("COM8", debug=False)
        check("open_link: configured port answering -> no write", os.path.getmtime(p), m)
        with open(p, "wb") as f:
            f.write(b"NEWPORT_port = 'COM3'\r\n")
        nl.open_link("COM3", debug=False, remember=False)
        with open(p, "rb") as f:
            check("open_link(remember=False) leaves it alone", f.read(), b"NEWPORT_port = 'COM3'\r\n")
    finally:
        nl.try_port, nl.list_ports, nl.find_cfg_com, builtins.input = saved


def _selftest_time(check):
    """TIME / ?TIME against the simulated FPGA."""
    import datetime as dt
    import newport_link as nl
    from newport_sim import SimFPGA
    now = time.time()
    link = nl.open_link("SIM", debug=True)
    sim = link.ser
    check("TIME sent when the link opens", (sim.time_set_count, link.time_supported), (1, True))
    check("TIME is ms since 1970 UTC", abs(sim.time_ms - now * 1000) < 2000, True)
    check("TIME line is 12 hex digits", bool(re.fullmatch(r"TIME [0-9A-F]{12}", sim.lines_seen[-1])), True)

    sim.ask_time_on_start = True                 # FPGA asks when a test starts
    d = run_test(link, 0)
    check("?TIME at START answered during wait_done",
          (d["passed"], sim.time_requests_sent, link.time_answered, sim.time_set_count), (True, 1, 1, 2))
    check("...order START, TIME, GET RES",
          [x.split()[0] for x in sim.lines_seen[-4:]], ["SET", "START", "TIME", "GET"])
    sim.ask_time_on_start = False

    sim.request_time()                           # while the host is idle
    check("idle ?TIME answered after the next command",
          (link.get_cfg(3), sim.lines_seen[-2:][0], sim.lines_seen[-1][:5]), (0, "GET TEST3_CFG", "TIME "))
    for ch in ("T", "S", "3", "\r"):             # ?TIME just before an echo
        sim.inject_before_echo, sim.inject_on_char = "?TIME\r\n", ch
        before = sim.time_set_count
        link.set_cfg(3, 0x0123)
        check("?TIME before echo of %r: command ok + answered" % ch,
              (link.get_cfg(3), sim.time_set_count - before), (0x0123, 1))
    sim.inject_on_char = None
    sim.inject_before_echo = "DONE TEST5\r\n?TIME\r\n"   # both, back to back
    before = sim.time_set_count
    check("DONE + ?TIME together", (link.get_cfg(3), "DONE TEST5" in link.done_messages,
                                    sim.time_set_count - before), (0x0123, True, 1))
    link.clear_done()

    link.time_sent_at -= nl.TIME_RESYNC_S + 1    # anchor getting old
    before = sim.time_set_count
    run_test(link, 1)
    check("stale anchor: TIME re-sent before START", sim.time_set_count - before, 1)
    before = sim.time_set_count
    run_test(link, 1)
    check("fresh anchor: no TIME before START", sim.time_set_count - before, 0)

    old = SimFPGA("OLD")
    old.time_supported = False                   # bitstream without TIME
    link2 = nl.NewportLink("OLD", ser=old)
    check("older bitstream: ERR tolerated", (link2.sync_time(), link2.time_supported), (False, False))
    n_lines = len(old.lines_seen)
    d = run_test(link2, 0)
    check("...then no more TIME sent", (d["passed"], [x for x in old.lines_seen[n_lines:] if x.startswith("TIME")]),
          (True, []))

    t = 1790784000.123
    check("fpga_time() round trip", nl.fpga_time("%012X" % nl.time_ms(t)),
          dt.datetime.fromtimestamp(round(t * 1000) / 1000.0))
    link.close()


def _selftest_sweep(check):
    """Rail sweep against the simulated FPGA + MPQ8785 models."""
    global SWEEP_CSV
    import tempfile
    import fpga_pmbus as pm
    from newport_link import NewportLink
    from newport_sim import SimFPGA
    cfg = pm._find_cfg()
    pm.apply_cfg(None)
    saved_csv = SWEEP_CSV
    tmp = tempfile.mkdtemp()
    SWEEP_CSV = os.path.join(tmp, "sweep.csv")
    try:
        sim = SimFPGA()
        link = NewportLink("SIM", ser=sim)
        core = sim.i2c_devices[pm.REGULATORS["vddcore"]]
        io = sim.i2c_devices[pm.REGULATORS["vddio"]]
        _startup_rails(link)
        import newport_power as npw
        npw.stop_monitor(link)                          # checked in newport_power's selftest
        check("startup: supplies off, CTRL in control, RST low",
              (sim.pwr_out, core.regs[0x02], io.regs[0x02], core.output_on(), io.output_on()),
              (0, 0x16, 0x16, False, False))
        npw.power_up(link, quiet=True)
        check("power-up: rails at nominal", (core.regs[0x21], io.regs[0x21]),
              (round(0.8 / 2 ** -9), round(1.8 / 2 ** -9)))
        pts = pm.sweep_points("vddcore", -3, 3, 1.5)
        out = run_sweep(link, pts, tests=[0, 6], serial_number="st", quiet=True)
        check("sweep: rails set VDDIO first", [r["rail"] for r in out[0]["rails"]], ["vddio", "vddcore"])
        check("sweep: 5 points x 2 tests run", [len(x["results"]) for x in out], [2] * 5)
        check("sweep: all points pass", [x["passed"] for x in out], [True] * 5)
        check("sweep: vddcore stepped", [round(next(r["set_V"] for r in x["rails"] if r["rail"] == "vddcore"), 4) for x in out],
              [round(round(v / 2 ** -9) * 2 ** -9, 4) for v in (0.776, 0.788, 0.8, 0.812, 0.824)])
        check("sweep: back at nominal after", core.regs[0x21], round(0.8 / 2 ** -9))
        with open(SWEEP_CSV) as f:
            rows = list(csv.DictReader(f))
        check("sweep CSV: one row per test per point", len(rows), 10)
        check("sweep CSV: rail columns", (rows[0]["vddcore_pct"], rows[0]["vddio_pct"]), ("-3.08", "+0.04"))
        sim.set_result(6, 0xC005)                       # TEST6 fails at every point
        out = run_sweep(link, pm.corner_points(), tests=[0, 6], quiet=True)
        check("corner sweep: 9 points, failures reported", (len(out), sum(x["passed"] for x in out)), (9, 0))
        sim.set_result(6, None)
        core.vout_offset = 20                            # rail won't verify
        out = run_sweep(link, pts[:2], tests=[0, 6], quiet=True)
        check("rail out of tolerance: tests skipped", [(x["passed"], len(x["results"])) for x in out],
              [(False, 0), (False, 0)])
        core.vout_offset = 0
        try:
            run_sweep(link, [{"vddcore": 0.95}], tests=[0], quiet=True)
            check("sweep point past SWEEP_LIMIT stops the sweep", False, True)
        except pm.RailError:
            check("sweep point past SWEEP_LIMIT stops the sweep", True, True)
        check("...and the rails still go back to nominal", core.regs[0x21], round(0.8 / 2 ** -9))
        _selftest_power_entries(check, sim, link, io, core)
    finally:
        SWEEP_CSV = saved_csv
        pm.apply_cfg(cfg)
    return 0


def _selftest_power_entries(check, sim, link, io, core):
    """Menu entries with DUT power: power-up on demand, a fault stops the
    test and the sequence, keep vs cycle."""
    global _link, get_link, RESULTS_CSV
    import tempfile
    import newport_power as npw
    real_get, real_link, real_csv = get_link, _link, RESULTS_CSV
    get_link = lambda: link
    _link = link
    RESULTS_CSV = os.path.join(tempfile.mkdtemp(), "results.csv")   # not the real log
    saved = dict(npw.CFG)
    try:
        npw.power_down(link, quiet=True)
        check("menu test: supplies off before", npw.is_on(link), False)
        rc = slot_test(0)("st")
        check("menu test powers up first, then runs", (rc, npw.is_on(link)), (0, True))
        n = len(sim.pwr_log)
        slot_test(1)("st")
        check("keep: next test doesn't touch power", len(sim.pwr_log), n)
        npw.CFG["POWER_BETWEEN_TESTS"] = "cycle"
        rc = slot_test(1)("st")
        check("cycle: powered down after the test", (rc, npw.is_on(link), sim.pwr_out & 0b0101), (0, False, 0))
        npw.CFG["POWER_BETWEEN_TESTS"] = "keep"
        npw.ensure_power_on(link, quiet=True)
        real_wait = link.wait_done

        def drop_while_waiting(n, timeout=1.0):         # the test is waiting on DONE
            io.force_pg_low = True                      # when VDDIO PG drops and the
            link.pwr_changed_at = None                  # monitor (here: one pass) sees it
            npw.check_once(link, lambda m: None)
            return real_wait(n, timeout)
        link.wait_done = drop_while_waiting
        try:
            rc = slot_test(2)("st")
        finally:
            link.wait_done = real_wait
        check("PG drop mid-test: test fails, fault latched",
              (rc, power_fault()), (1, "VDDIO POWER_GOOD dropped"))
        check("... supplies off", (io.output_on(), core.output_on()), (False, False))
        io.force_pg_low = False
        rc = slot_test(0)("st")
        check("fault latched: next DUT test refused", (rc, npw.is_on(link)), (1, False))
        rc = dut_power_up("st")
        check("menu 'DUT power up' clears it", (rc, power_fault(), npw.is_on(link)), (0, None, True))
        rc = dut_power_down("st")
        check("menu 'DUT power down'", (rc, npw.is_on(link)), (0, False))
    finally:
        npw.CFG.clear()
        npw.CFG.update(saved)
        get_link, _link, RESULTS_CSV = real_get, real_link, real_csv


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="Run Newport FPGA hardware tests")
    ap.add_argument("tests", nargs="*", type=int, help="test numbers (default 0-8)")
    ap.add_argument("--port", default=None, help="serial port (default: cfg_com.NEWPORT_port or COM3)")
    ap.add_argument("--serial", default="bench", help="DUT serial number for the log")
    ap.add_argument("--debug", action="store_true", help="use the simulated FPGA")
    ap.add_argument("--selftest", action="store_true", help="offline checks of this module")
    a = ap.parse_args(argv)
    if a.selftest:
        return 0 if selftest() else 1
    if a.debug:
        os.environ["K_DEBUG"] = "1"
        try:
            import K_Sim
            K_Sim.set_debug(True)
        except ImportError:
            pass
    logging.basicConfig(level=logging.WARNING, format="%(message)s")
    if a.port:
        set_port(a.port)
    rc = 0
    for n in (a.tests or REAL_TESTS):
        rc |= slot_test(n)(a.serial)
    close_link()
    return rc


if __name__ == "__main__":
    sys.exit(main())
