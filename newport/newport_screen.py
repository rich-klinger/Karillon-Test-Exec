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
#  Purpose: Batch part-screening over TEST10 (added 2026-10-09).
#
#------------------------------------------------------------------------------
"""
newport_screen.py -- batch part-screening loop for TEST10 (added 2026-10-09).

A technician screens a tray/box of parts, one at a time: load a part in the
socket, type its serial number, press GO (ENTER), the FPGA runs TEST10 and
answers pass/fail, the result goes to a dedicated log line, and the loop
asks for the next part. Blank serial number (or "quit"/"done") ends the
batch, closes the log, and returns to the Test Exec menu.

TEST10 is a placeholder -- the FPGA screening logic isn't written yet (see
newport_tests.PARTSCREEN_TEST and fpga/newport_eval/regfile_tests.v's
g_reserved branch). Today every part comes back NOT IMPLEMENTED (0xBFFF),
decoded by newport_tests.decode()'s "screen" layout with passed=None so it
can never be mistaken for a real pass. This module already handles a real
pass/fail/timeout result the same way, so nothing here needs to change once
the FPGA side lands -- only TESTS[10]'s layout stops short-circuiting on
0xBFFF.

    screen_session(serial_number)   Test Exec menu entry (0 pass / 1 fail)
    main(argv)                      standalone CLI

Standalone (bench) use:
    python newport_screen.py                run a screening batch on COM3
    python newport_screen.py --debug        the same against the simulated FPGA
    python newport_screen.py --selftest     offline checks of this module

Each session's results go to their own timestamped CSV under
logs/screening/ -- not newport_tests.RESULTS_CSV -- both because a NOT
IMPLEMENTED result (passed=None) doesn't fit that file's int(d["passed"])
column, and because a screening run is naturally a single closeable
artifact: one file per batch, opened when the technician starts and closed
when they're done with the tray.
"""

import csv
import datetime
import logging
import os
import sys

from newport_link import NewportError

log = logging.getLogger("newport")

LOG_DIR = os.path.join("logs", "screening")

# Typing any of these at a prompt ends the batch (case-insensitive).
_QUIT_WORDS = ("quit", "q", "done", "exit")


def _ask(prompt, default=""):
    try:
        ans = input("%s%s: " % (prompt, " [%s]" % default if default else "")).strip()
    except EOFError:
        ans = ""
    return ans or default


def _classify(passed):
    """passed, as decode()'s 'screen' layout sets it (True/False/None) ->
    the status word used in the log and the on-screen summary."""
    if passed is None:
        return "NOT IMPLEMENTED"
    return "PASS" if passed else "FAIL"


def _open_log(log_dir=None):
    """Start a new timestamped screening log. Returns (file, csv writer,
    path); the caller closes the file when the batch ends."""
    log_dir = log_dir or LOG_DIR
    os.makedirs(log_dir, exist_ok=True)
    path = os.path.join(log_dir, "screen_%s.csv"
                         % datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["time", "serial", "status", "result_hex", "summary", "seconds"])
    return f, w, path


def _run_one(link, writer, serial_number):
    """Run TEST10 once for serial_number, log the result, and return the
    status word ("PASS"/"FAIL"/"NOT IMPLEMENTED"/"ERROR"). A NewportError
    (UART NAK, timeout, busy, ...) is caught here so one bad part doesn't
    raise out of the batch loop -- it's logged as ERROR with the exception
    text as the summary; the caller decides whether/how to reconnect."""
    import newport_tests as nt
    now = datetime.datetime.now().isoformat(timespec="seconds")
    try:
        d = nt.run_test(link, nt.PARTSCREEN_TEST)
    except NewportError as e:
        status = "ERROR"
        print("Newport FPGA error: %s - part logged as ERROR" % e)
        log.error("screen SN %s: %s", serial_number, e)
        writer.writerow([now, serial_number, status, "", str(e), ""])
        return status
    status = _classify(d["passed"])
    print("TEST10 0x%04X  %-5s %s" % (d["raw"], status, d["summary"]))
    log.info("screen SN %s  TEST10 0x%04X  %s  %s",
             serial_number, d["raw"], status, d["summary"])
    writer.writerow([now, serial_number, status, "%04X" % d["raw"],
                      d["summary"], "%.3f" % d["seconds"]])
    return status


def _run_batch(link, writer, first_serial=None):
    """The technician-facing loop: prompt for a serial number and a GO,
    run TEST10, repeat. Returns a dict of status -> count. first_serial
    (the serial number the Test Exec menu already asked for, if any) is
    offered as the first part's serial number so the technician doesn't
    have to retype it; <Enter> accepts it, like every other reused-SN
    prompt in this codebase (see ktest.get_serial_number's --eng mode)."""
    import newport_tests as nt
    counts = {}
    next_default = first_serial or ""
    while True:
        sn = _ask("\nSerial number (blank or 'done' to finish)", next_default)
        next_default = ""                      # only offered once
        if not sn or sn.strip().lower() in _QUIT_WORDS:
            break
        go = _ask("Part loaded in socket - press ENTER (or type GO) to "
                   "start, or 'skip' to skip this part")
        if go.strip().lower() in _QUIT_WORDS:
            break
        if go.strip().lower() == "skip":
            print("SN %s: skipped" % sn)
            counts["SKIPPED"] = counts.get("SKIPPED", 0) + 1
            continue
        if link is None:
            link = nt.get_link()
        if link is None:
            print("Newport FPGA not connected - part not tested")
            counts["ERROR"] = counts.get("ERROR", 0) + 1
            continue
        # DUT power (2026-10-11): up for this part only, down again after it
        # so the part can be changed with the supplies off.
        import newport_power
        try:
            newport_power.ensure_power_on(link, quiet=True)
        except NewportError as e:
            print("SN %s: DUT power: %s - part not tested" % (sn, e))
            log.error("screen: SN %s DUT power: %s", sn, e)
            counts["ERROR"] = counts.get("ERROR", 0) + 1
            continue
        link.testing += 1
        try:
            status = _run_one(link, writer, sn)
        finally:
            link.testing -= 1
            try:
                newport_power.power_down(link, reason="screening: part done", quiet=True)
                print("DUT supplies off - safe to change the part")
            except NewportError as e:
                print("DUT power-down after SN %s failed: %s - check the board before "
                      "changing the part" % (sn, e))
                log.error("screen: power-down after SN %s: %s", sn, e)
        counts[status] = counts.get(status, 0) + 1
        if status == "ERROR":
            # Force a fresh link next time rather than keep hammering a
            # UART that just NAK'd/timed out -- one bad part shouldn't
            # abort the rest of the tray. If the FPGA doesn't come back,
            # every later part logs ERROR too but the batch (and its log)
            # stays open until the technician ends it.
            nt.close_link()
            link = nt.get_link()
    return counts


def screen_session(serial_number, log_dir=None):
    """Test Exec menu entry: run a batch of TEST10 screenings and return
    0 (session completed -- see the log for each part's result) or 1 (the
    FPGA couldn't be reached at all, nothing was tested). log_dir is for
    selftest(); the menu always uses the default (logs/screening/)."""
    import newport_tests as nt
    link = nt.get_link()
    if link is None:
        print("Newport FPGA not connected - screening session not started")
        return 1

    f, writer, path = _open_log(log_dir)
    print("Part screening: TEST10 is a placeholder (FPGA side not written "
          "yet) -- every part will log NOT IMPLEMENTED until real RTL "
          "lands. See newport_tests.PARTSCREEN_TEST.")
    print("Screening log: %s" % path)
    log.info("screen: session start, log %s", path)
    try:
        counts = _run_batch(link, writer, serial_number)
    finally:
        f.close()

    total = sum(counts.values())
    summary = "Screening done: %d part%s (%s)" % (
        total, "" if total == 1 else "s",
        ", ".join("%d %s" % (n, s) for s, n in sorted(counts.items())) or "none tested")
    print(summary)
    print("Log closed: %s" % path)
    log.info("screen: session end, %s", summary)
    return 0


# ---------------------------------------------------------------------------
# Offline self-test against the simulated FPGA
# ---------------------------------------------------------------------------
def selftest():
    import builtins
    import io
    import tempfile

    from newport_link import NewportLink, BusTimeoutError
    from newport_sim import SimFPGA
    import newport_tests as nt

    fails = 0

    def check(what, got, want):
        nonlocal fails
        ok = got == want
        fails += not ok
        print("%s  %-50s %r%s" % ("pass" if ok else "FAIL", what, got,
                                  "" if ok else "  (want %r)" % (want,)))

    check("_classify True", _classify(True), "PASS")
    check("_classify False", _classify(False), "FAIL")
    check("_classify None (placeholder)", _classify(None), "NOT IMPLEMENTED")

    sim = SimFPGA()
    link = NewportLink("SIM", ser=sim)

    # _run_one against a csv.writer backed by a StringIO, independent of
    # the interactive loop.
    def one(sn="SELFTEST"):
        buf = io.StringIO()
        w = csv.writer(buf)
        status = _run_one(link, w, sn)
        row = buf.getvalue().strip("\r\n").split(",")
        return status, row

    status, row = one()
    check("placeholder (0xBFFF default) -> NOT IMPLEMENTED", status, "NOT IMPLEMENTED")
    check("placeholder row: serial, status", row[1:3], ["SELFTEST", "NOT IMPLEMENTED"])

    sim.set_result(10, 0x8000 | 0x4000 | 10)
    status, row = one()
    check("pass-shaped result -> PASS", status, "PASS")
    check("pass row: result_hex", row[3], "%04X" % (0x8000 | 0x4000 | 10))

    sim.set_result(10, 0x8000 | 0x4000 | 3)       # wrong id field
    status, row = one()
    check("wrong-id result -> FAIL", status, "FAIL")

    sim.set_result(10, None)                      # back to the 0xBFFF default

    real_run_test = nt.run_test
    nt.run_test = lambda link, n, **kw: (_ for _ in ()).throw(BusTimeoutError("sim NAK"))
    try:
        status, row = one("BADPART")
    finally:
        nt.run_test = real_run_test
    check("NewportError during run_test -> ERROR, doesn't raise", status, "ERROR")
    check("ERROR row carries the exception text", "sim NAK" in row[4], True)

    # Full scripted batch: part 1 reuses the Test-Exec-supplied serial
    # number (blank at the SN prompt), part 2 types its own, then a blank
    # serial number ends the session.
    answers = iter(["", "", "SN-002", "", ""])
    real_input, real_get_link, real_close_link = builtins.input, nt.get_link, nt.close_link
    builtins.input = lambda prompt="": next(answers)
    nt.get_link = lambda: link
    nt.close_link = lambda: None
    try:
        with tempfile.TemporaryDirectory() as tmp:
            rc = screen_session("SN-001", log_dir=tmp)
            logged = sorted(os.listdir(tmp))
            check("screen_session returns 0 (FPGA reachable)", rc, 0)
            check("one log file written", len(logged), 1)
            with open(os.path.join(tmp, logged[0]), newline="") as lf:
                rows = list(csv.reader(lf))
            check("header + 2 parts logged", len(rows), 3)
            check("part 1 reused the Test Exec serial number", rows[1][1], "SN-001")
            check("part 2 used its own typed serial number", rows[2][1], "SN-002")
            check("both parts NOT IMPLEMENTED (placeholder)",
                  [rows[1][2], rows[2][2]], ["NOT IMPLEMENTED", "NOT IMPLEMENTED"])
    finally:
        builtins.input, nt.get_link, nt.close_link = real_input, real_get_link, real_close_link

    # No FPGA at all: session reports 1 and writes no log.
    nt.get_link = lambda: None
    try:
        with tempfile.TemporaryDirectory() as tmp:
            rc = screen_session("SN-003", log_dir=tmp)
            check("no FPGA -> returns 1", rc, 1)
            check("no FPGA -> no log file written", os.listdir(tmp), [])
    finally:
        nt.get_link = real_get_link

    print("SELFTEST", "PASS" if fails == 0 else "FAIL (%d)" % fails)
    return fails == 0


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------
def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="Batch part-screening over TEST10 (placeholder)")
    ap.add_argument("--port", default=None, help="serial port (default: cfg_com.NEWPORT_port or COM3)")
    ap.add_argument("--serial", default=None, help="first part's serial number (offered, not required)")
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
    import newport_tests as nt
    if a.port:
        nt.set_port(a.port)
    try:
        return screen_session(a.serial or "")
    finally:
        nt.close_link()


if __name__ == "__main__":
    sys.exit(main())
