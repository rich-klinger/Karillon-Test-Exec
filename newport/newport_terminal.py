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
#  Purpose: newport> terminal -- Newport DUT-level commands (placeholder).
#
#------------------------------------------------------------------------------
"""
newport_terminal.py -- the newport> terminal (PLACEHOLDER, 2026-10-11).

Two command-prompt modes share one link to the FPGA:

    fpga>      raw FPGA (cmd_interp) commands -- fpga_terminal.py
    newport>   Newport DUT-level commands      -- this file

The newport> command set isn't designed yet. Until it is, this terminal only
knows its own housekeeping commands; anything else is reported as not
implemented and nothing is sent to the FPGA.

Terminal commands (lowercase):

    help                 this list
    fpga                 switch to the fpga> terminal
    quit / exit / q      back to the menu (also Ctrl-C, or Ctrl-Z Enter)

To add a DUT command: add its name to DUT_CMDS with a function taking
(link, args, out), and a line to the help text above.

Menu entry for Karillon_TestExec: terminal(serial_number).
Standalone: python newport_terminal.py [--port COM5] [--debug]
"""

import os
import sys

from newport_link import NewportError

PROMPT = "newport> "
# Shown in green (fpga> is blue) so the two modes are easy to tell apart.
# Plain ANSI codes, the same ones termcolor uses for "green".
PROMPT_SHOWN = "\033[32m" + PROMPT + "\033[0m"
HELP = __doc__[__doc__.index("    help"):__doc__.index("To add a DUT")].rstrip()

# DUT-level commands: name -> function(link, args, out). None yet.
DUT_CMDS = {}


def run(link, inp=input, out=print):
    """The newport> loop on an open link. Returns "fpga" to switch to the
    fpga> terminal, None to leave."""
    out("Newport DUT terminal (newport>) on %s -- placeholder, no DUT commands "
        "yet. Type help, fpga for the FPGA terminal, quit to go back." % link.port)
    while True:
        try:
            line = inp(PROMPT_SHOWN)
        except (EOFError, KeyboardInterrupt):
            out("")
            return None
        words = line.strip().split()
        if not words:
            continue
        cmd, args = words[0], words[1:]
        if cmd in ("quit", "exit", "q"):
            return None
        if cmd == "fpga":
            return "fpga"
        if cmd in ("help", "?"):
            out("Terminal commands:")
            out(HELP)
            if DUT_CMDS:
                out("DUT commands: " + ", ".join(sorted(DUT_CMDS)))
            continue
        fn = DUT_CMDS.get(cmd)
        if fn is None:
            out("  %s: not implemented yet (newport> is a placeholder; raw FPGA "
                "commands are in the fpga> terminal -- type fpga)" % cmd)
            continue
        try:
            fn(link, args, out)
        except (NewportError, ValueError, OSError) as e:
            out("  error: %s" % e)


def terminal(serial_number):
    """Karillon_TestExec menu entry: the newport> terminal. Always 'passes'
    (returns 0)."""
    import newport_tests as nt
    import fpga_terminal

    def body(link, sn):
        fpga_terminal.session(link, "newport")
        return 0
    return nt._with_link(body, serial_number)


# ---------------------------------------------------------------------------
# Self-test (simulated FPGA, scripted input)
# ---------------------------------------------------------------------------
def selftest():
    from newport_link import NewportLink
    from newport_sim import SimFPGA
    import fpga_terminal
    fails = 0

    def check(what, got, want):
        nonlocal fails
        ok = got == want
        fails += not ok
        print("%s  %-44s %r%s" % ("pass" if ok else "FAIL", what, got,
                                  "" if ok else "  (want %r)" % (want,)))

    def session(lines, mode):
        sim = SimFPGA()
        link = NewportLink("SIM", ser=sim)
        feed, outp, prompts = list(lines), [], []

        def inp(prompt):
            prompts.append(prompt)
            if not feed:
                raise EOFError
            return feed.pop(0)
        fpga_terminal.session(link, mode, inp, outp.append)
        return sim, outp, prompts

    NP, FP = PROMPT_SHOWN, fpga_terminal.PROMPT_SHOWN   # green, blue
    sim, o, p = session(["help", "frob 1 2", "", "quit"], "newport")
    check("newport> prompt, green", p[0], "\033[32mnewport> \033[0m")
    check("help lists fpga + quit", ("fpga " in o[2] and "quit" in o[2]), True)
    check("unknown -> not implemented, nothing sent",
          (o[3].startswith("  frob: not implemented yet"), sim.lines_seen), (True, []))
    check("quit leaves", len(p), 4)

    sim, o, p = session(["fpga", "GET AFECFG", "newport", "fpga", "quit"], "newport")
    check("switch newport> -> fpga> -> newport> -> fpga>", p,
          [NP, FP, FP, NP, FP])
    check("fpga> sends raw commands", sim.lines_seen, ["GET AFECFG"])

    sim, o, p = session(["newport", "q"], "fpga")
    check("fpga> 'newport' switches", p, [FP, NP])

    print("SELFTEST", "PASS" if fails == 0 else "FAIL (%d)" % fails)
    return fails == 0


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="newport> terminal: Newport DUT commands (placeholder)")
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
