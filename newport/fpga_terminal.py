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
#  Purpose: fpga> terminal -- raw Newport FPGA (cmd_interp) commands.
#
#------------------------------------------------------------------------------
"""
fpga_terminal.py -- the fpga> terminal: type cmd_interp commands at the
Newport FPGA directly. (Renamed from newport_terminal.py 2026-10-11; that
name now belongs to the newport> terminal for DUT-level commands, see
newport_terminal.py. "newport" here switches to it, "fpga" there switches
back.)

Anything typed is sent as-is (one character at a time, each echo awaited,
as NewportLink.command() always does) and the FPGA's response line is
printed. Unsolicited DONE TESTn lines are shown as they arrive; ?TIME
requests are answered automatically, as everywhere else.

FPGA commands are uppercase and case-sensitive (see
fpga/newport_eval/README.md):

    SET TESTn HHHH     GET TESTn_CFG     GET TESTn_RES     START TESTn
    GET AFECFG         I2C AA W R [HH..] TIME HHHHHHHHHHHH
    PKL p              PKR p oo
    JSEL MMM  JCAP c  JCLK HHHH  JRST b  JTMS n HH  JSI nn HHHHHHHH x
    JLD b HHHH  JSHF HHHH f  JERR c            (JTAG -- see newport_jtag.py)

Terminal commands are lowercase, so they can never be mistaken for one:

    help                 this list
    quit / exit / q      back to the menu (also Ctrl-C, or Ctrl-Z Enter)
    pkl <slot> <file>    load a packet from a file (.bin raw, otherwise hex)
    pkl <slot> demo      load that slot's demo pattern (newport_pkt.DEMO)
    pkl demo             load all 8 demo patterns
    dump <slot>          read a whole packet slot back (32 x PKR)
    jload <D|E|M> <file> load a JTAG buffer from a file (.bin raw, otherwise hex)
    wait <n> [seconds]   wait for DONE TESTn (default 2 s)
    time                 send the host's time of day (TIME)
    pwrup                DUT power-up sequence: VDDIO, wait for its
                         POWER_GOOD, then VDDCORE (newport_power.py)
    pwrdown              DUT power-down sequence: VDDCORE, then VDDIO
    pwr                  DUT power status (control lines, POWER_GOOD,
                         USB-Blaster, each rail's VOUT/IOUT/STATUS)
    newport              switch to the newport> terminal (DUT commands)

A bare "PKL p" or "JLD b HHHH" is not sent: after its OK the FPGA expects
raw bytes, which can't be typed. Use the lowercase pkl / jload above.
A raw "PWR 0 1" or "PWR 2 1" (rail ON) is refused while a USB-Blaster is
on the FPGA's JTAG header; raw PWR skips the sequencing, so prefer pwrup.

Menu entry for Karillon_TestExec: terminal(serial_number).
Standalone: python fpga_terminal.py [--port COM5] [--debug]
"""

import os
import re
import sys
import time

from newport_link import NewportError, PKT_SLOTS

PROMPT = "fpga> "
# Shown in blue (newport> is green) so the two modes are easy to tell apart.
# Plain ANSI codes, the same ones termcolor uses for "blue".
PROMPT_SHOWN = "\033[34m" + PROMPT + "\033[0m"
LOCAL_CMDS = {"help", "?", "quit", "exit", "q", "pkl", "dump", "wait", "time", "jload",
              "pwrup", "pwrdown", "pwr", "newport"}
LOCAL_HELP = __doc__[__doc__.index("    help"):__doc__.index("A bare")].rstrip()
FPGA_HELP = __doc__[__doc__.index("    SET TESTn"):__doc__.index("Terminal commands")].rstrip()


def _print_done(link):
    """Show (and consume) DONE lines collected so far."""
    while link.done_messages:
        print("  << " + link.done_messages.pop(0))


def _settle(link, quiet_time=0.03):
    """Give an unsolicited line that's on its way (e.g. the DONE right after
    a START) time to arrive, then show it."""
    end = time.monotonic() + quiet_time
    while time.monotonic() < end:
        if link._pump():
            end = time.monotonic() + quiet_time
    _print_done(link)


def _slot(text):
    if not text.isdigit() or not 0 <= int(text) < PKT_SLOTS:
        raise ValueError("slot must be 0-%d" % (PKT_SLOTS - 1))
    return int(text)


def _local(link, words):
    """Run a lowercase terminal command. Returns False to leave, "newport"
    to switch to the newport> terminal, True to stay."""
    import newport_pkt as npk
    cmd, args = words[0], words[1:]

    if cmd in ("quit", "exit", "q"):
        return False

    if cmd == "newport":
        return "newport"

    if cmd in ("pwrup", "pwrdown", "pwr"):
        import newport_power as npw
        if cmd == "pwrup":
            npw.power_up(link, manual=True)
        elif cmd == "pwrdown":
            npw.power_down(link, reason="fpga> pwrdown")
        else:
            npw.print_status(link)
        return True

    if cmd in ("help", "?"):
        print("FPGA commands (sent as typed, case-sensitive):")
        print(FPGA_HELP)
        print("Terminal commands:")
        print(LOCAL_HELP)

    elif cmd == "pkl":
        if args == ["demo"]:
            packets = dict(npk.DEMO)
        elif len(args) == 2:
            slot = _slot(args[0])
            packets = {slot: npk.DEMO[slot] if args[1] == "demo"
                       else npk.read_packet_file(args[1])}
        else:
            raise ValueError("usage: pkl <slot> <file|demo>   or   pkl demo")
        for slot in sorted(packets):
            crc = link.pkt_load(slot, packets[slot])
            print("  slot %d loaded, CRC %04X" % (slot, crc))

    elif cmd == "dump":
        if len(args) != 1:
            raise ValueError("usage: dump <slot>")
        slot = _slot(args[0])
        print(npk._hexdump(link.pkt_read_all(slot)))

    elif cmd == "wait":
        if not 1 <= len(args) <= 2 or not args[0].isdigit():
            raise ValueError("usage: wait <test n> [seconds]")
        n, secs = int(args[0]), float(args[1]) if len(args) > 1 else 2.0
        if link.wait_done(n, secs):
            print("  << DONE TEST%d" % n)
        else:
            print("  no DONE TEST%d within %g s" % (n, secs))

    elif cmd == "jload":
        import newport_jtag as njt
        if len(args) != 2 or args[0].upper() not in ("D", "E", "M"):
            raise ValueError("usage: jload <D|E|M> <file>")
        data = njt.read_data_file(args[1])
        crc = link.jld(args[0].upper(), data)
        print("  buffer %s loaded: %d bytes (%d bits), CRC %04X"
              % (args[0].upper(), len(data), 8 * len(data), crc))

    elif cmd == "time":
        print("  OK" if link.send_time() else "  ERR (bitstream has no TIME command)")

    return True


_PWR_ON_RE = re.compile(r"PWR [02] 1")


def run(link, inp=input, out=print):
    """The fpga> loop on an open link. inp/out are swappable for tests.
    Returns "newport" to switch to the newport> terminal, None to leave."""
    out("FPGA terminal (fpga>) on %s. Type help for commands, newport for the "
        "DUT terminal, quit to go back." % link.port)
    link.drain(0)
    while True:
        _print_done(link)
        try:
            line = inp(PROMPT_SHOWN)
        except (EOFError, KeyboardInterrupt):
            out("")
            return None
        line = line.strip("\r\n")
        if not line.strip():
            continue
        words = line.split()
        try:
            if words[0] in LOCAL_CMDS:                         # terminal command
                r = _local(link, words)
                if r is False:
                    return None
                if r == "newport":
                    return "newport"
                continue
            if _PWR_ON_RE.fullmatch(line):
                import newport_power as npw
                npw.check_power_allowed(link)                  # USB-Blaster interlock
            if re.fullmatch(r"PKL [0-7]", line):
                out("  not sent: PKL needs 256 raw bytes after its OK - "
                    "use  pkl %s <file|demo>" % line[4])
                continue
            if re.fullmatch(r"JLD [DEM] [0-9A-Fa-f]{4}", line):
                out("  not sent: JLD needs raw bytes after its OK - "
                    "use  jload %s <file>" % line[4])
                continue
            t0 = time.monotonic()
            resp = link.command(line)
            ms = (time.monotonic() - t0) * 1000
            hint = ""
            if resp == "ERR" and line != line.upper():
                hint = "   (FPGA commands are uppercase and case-sensitive)"
            out("  %s%s    [%.0f ms]" % (resp, hint, ms))
            _settle(link)
        except (NewportError, ValueError, OSError) as e:
            out("  error: %s" % e)
        except KeyboardInterrupt:
            out("  interrupted")
            try:
                link.resync()
            except NewportError:
                pass


def session(link, mode="fpga", inp=input, out=print):
    """Run the fpga> / newport> terminals on one link, switching between
    them when asked, until one of them is quit."""
    import newport_terminal
    while mode:
        mode = (run if mode == "fpga" else newport_terminal.run)(link, inp, out)


def terminal(serial_number):
    """Karillon_TestExec menu entry: the fpga> terminal. Always 'passes'
    (returns 0)."""
    import newport_tests as nt

    def body(link, sn):
        session(link, "fpga")
        return 0
    return nt._with_link(body, serial_number)


# ---------------------------------------------------------------------------
# Self-test (simulated FPGA, scripted input)
# ---------------------------------------------------------------------------
def selftest():
    from newport_link import NewportLink
    from newport_sim import SimFPGA
    import newport_pkt as npk
    fails = 0

    def check(what, got, want):
        nonlocal fails
        ok = got == want
        fails += not ok
        print("%s  %-44s %r%s" % ("pass" if ok else "FAIL", what, got,
                                  "" if ok else "  (want %r)" % (want,)))

    seen_prompts = []

    def session(lines):
        sim = SimFPGA()
        link = NewportLink("SIM", ser=sim)
        feed, outp = list(lines), []
        def inp(prompt):
            seen_prompts.append(prompt)
            if not feed:
                raise EOFError
            return feed.pop(0)
        import builtins
        real_print = builtins.print
        builtins.print = lambda *a, **k: outp.append(" ".join(str(x) for x in a))
        try:
            run(link, inp, lambda s: outp.append(s))
        finally:
            builtins.print = real_print
        return sim, [re.sub(r"\s+\[\d+ ms\]$", "", s) for s in outp]

    sim, o = session(["GET AFECFG", "SET TEST3 0042", "GET TEST3_CFG", "get test3_cfg",
                      "START TEST0", "PKL 2", "pkl 2 demo", "PKR 2 00", "pkl demo",
                      "dump 0", "wait 5 0.01", "bogus", "quit", "GET AFECFG"])
    check("GET AFECFG", o[1], "  000000000000")
    check("SET", o[2], "  OK")
    check("GET", o[3], "  0042")
    check("lowercase -> ERR + hint", o[4], "  ERR   (FPGA commands are uppercase and case-sensitive)")
    check("START shows OK then DONE", o[5:7], ["  OK", "  << DONE TEST0"])
    check("bare PKL not sent", (o[7].startswith("  not sent"), sim.lines_seen.count("PKL 2")), (True, 2))  # 2 = the two pkl loads
    check("pkl slot demo", (o[8], bytes(sim.pkt[2]) == npk.DEMO[2]), ("  slot 2 loaded, CRC %04X" % 0x41E8, True))
    check("PKR after load", o[9], "  " + npk.DEMO[2][:8].hex().upper())
    check("pkl demo loads 8", sum(s.startswith("  slot ") for s in o[10:18]), 8)
    check("dump", (o[18].splitlines()[0], len(o[18].splitlines())),
          ("  00: 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F", 16))
    check("wait timeout", o[-2], "  no DONE TEST5 within 0.01 s")
    check("unknown lowercase word goes to the FPGA", o[-1], "  ERR   (FPGA commands are uppercase and case-sensitive)")
    check("quit stops (later lines not sent)", sim.lines_seen.count("GET AFECFG"), 1)
    check("fpga> prompt, blue", seen_prompts[0], "\033[34mfpga> \033[0m")

    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        f = os.path.join(tmp, "d.bin")
        with open(f, "wb") as fh:
            fh.write(b"\xA5\x5A")
        sim, o = session(["JLD D 0002", "jload D " + f, "JSEL 001", "JRST 1", "JTMS 6 1F",
                          "JTMS 4 03", "JSI 05 00000003 1", "JTMS 2 01",   # IR = LOAD
                          "JTMS 3 01", "JSHF 0010 1", "JCAP 0", "JTMS 2 01", "JTMS 3 01",
                          "JSI 10 00000000 1"])
    check("bare JLD not sent", o[1].startswith("  not sent: JLD"), True)
    check("jload", o[2], "  buffer D loaded: 2 bytes (16 bits), CRC %04X" % __import__("binascii").crc_hqx(b"\xA5\x5A", 0xFFFF))
    check("JTAG raw: write then read back 16 bits", o[-2], "  00005AA5")

    sim, o = session(["pkl 9 demo", "pkl 1 nofile.bin", "dump", "SET TEST3 0001"])
    check("bad slot", o[1], "  error: slot must be 0-7")
    check("missing file reported", o[2].startswith("  error:"), True)
    check("dump usage", o[3], "  error: usage: dump <slot>")
    check("EOF ends, link still fine", o[4], "  OK")

    # DUT power commands (2026-10-11)
    import newport_power as npw
    sim, o = session(["pwr", "pwrup", "pwr", "pwrdown", "PWR 2 1", "PWR 2 0"])
    check("pwr: status, off", (o[1], o[2]), ("DUT power: off", "USB-Blaster on JTAG header: no"))
    check("pwrup: VDDIO first, then VDDCORE",
          [l for l, _ in sim.pwr_log if l in ("PWR 2 1", "PWR 0 1")][:2], ["PWR 2 1", "PWR 0 1"])
    i = o.index("DUT power: ON")
    check("pwr after pwrup: ON", o[i], "DUT power: ON")
    check("pwrdown: both off", (sim.pwr_out & 0b0101), 0)
    check("raw PWR 2 1 sent (no cable)", "PWR 2 1" in sim.lines_seen, True)
    sim2 = SimFPGA()
    sim2.set_jtag_cable(True)
    link2 = NewportLink("SIM", ser=sim2)
    out2 = []
    feed = ["PWR 2 1", "pwrup", "pwr"]
    import builtins
    real_print = builtins.print
    builtins.print = lambda *a, **k: out2.append(" ".join(str(x) for x in a))
    try:
        run(link2, lambda p: feed.pop(0) if feed else (_ for _ in ()).throw(EOFError()), out2.append)
    finally:
        builtins.print = real_print
    check("cable: raw PWR 2 1 refused, not sent",
          (out2[1].startswith("  error: USB-Blaster"), "PWR 2 1" in sim2.lines_seen), (True, False))
    check("cable: pwrup refused", out2[2].startswith("  error: USB-Blaster"), True)
    check("cable: pwr shows it", any("YES (power blocked)" in x for x in out2), True)

    print("SELFTEST", "PASS" if fails == 0 else "FAIL (%d)" % fails)
    return fails == 0


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="fpga> terminal: raw commands for the Newport FPGA")
    ap.add_argument("--port", default=None, help="serial port (default: cfg_com.NEWPORT_port or COM3)")
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
    import newport_tests as nt
    if a.port:
        nt.set_port(a.port)
    rc = terminal("bench")
    nt.close_link()
    return rc


if __name__ == "__main__":
    sys.exit(main())
