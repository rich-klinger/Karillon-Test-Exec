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
#  Purpose: Load 256-byte packets into the Newport FPGA and send them (TEST9).
#
#------------------------------------------------------------------------------
"""
newport_pkt.py -- packet RAM and TEST9 on the Newport FPGA (2026-10-01).

The FPGA keeps 8 packets of 256 bytes in on-chip RAM
(fpga/newport_eval/pkt_ram.v). TEST9 (TB/pkt_tx_test.v) sends a run of them
through the DFE as SOF + 256 bytes + EOF each and, unless TX-only is set,
checks every byte the DUT loops back against the same RAM.

    load_packets(link, {slot: bytes, ...})   PKL each, CRC checked
    send(link, first, count, repeat, tx_only, se)  run TEST9, decoded result
    send_file(link, path, se, check)          a whole file, 8 packets a batch
    tx_cfg(...)                               TEST9's config register value
    make_packet(payload, pad)                 pad a short payload to 256
    read_packet_file(path)                    .bin (raw) or text hex
    DEMO                                      8 test patterns, one per slot

Packets are always exactly 256 bytes on the FPGA side: there is no
end-of-packet marker, because the data can hold any byte value. Anything
the packet needs to carry -- a header, a length, a sequence number -- goes
inside those 256 bytes, built here; shorter payloads are padded.

Path (2026-10-02): se=False sends over LVDS (NSCIp/NSDIp), se=True over
the DFE's single-ended GPIO fallback (GPIO15 clock / GPIO14_DUT_0_0 data
out, GPIO13/GPIO12 back). The FPGA holds the other path's outputs low for
the run. For GPIO the Newport (chip 0,0) must have its own north DFE's
ioConfig[63] (single_end_bypass) set, or it won't hear anything.

Menu entries for Karillon_TestExec (0 pass / 1 fail, like the others):
    packet_test           DEMO into slots 0-7, send once over LVDS
    packet_test_tx_only   the same without the loopback check
    send_file_lvds        ask for a file, send it over LVDS
    send_file_gpio        ask for a file, send it over GPIO

Bench use:
    python newport_pkt.py                         load DEMO, send 0-7 once
    python newport_pkt.py --load 0=a.bin 1=b.hex  load files into slots 0, 1
    python newport_pkt.py --send 0-1 --repeat 9   send slots 0,1 ten times
    python newport_pkt.py --send 6-1              send 6,7,0,1 (wraps)
    python newport_pkt.py --demo --send 0-7 --tx-only   no loopback check
    python newport_pkt.py --dump 3                read slot 3 back (PKR)
    python newport_pkt.py --file data.bin         send a whole file over LVDS
    python newport_pkt.py --file data.bin --gpio --tx-only   ... over GPIO
    python newport_pkt.py --debug ...             against the simulated FPGA
    python newport_pkt.py --selftest              offline checks
"""

import os
import re
import sys

from newport_link import (PKT_LEN, PKT_SLOTS, NewportError, BusyError,
                          PacketError, pkt_crc)

TEST9 = 9
LINE_BYTES_PER_S = 12.5e6       # 125 Mb/s 8b10b on Interface 1


# ---------------------------------------------------------------------------
# Building packets
# ---------------------------------------------------------------------------
def make_packet(payload, pad=0x00):
    """payload (bytes, up to 256) padded to exactly PKT_LEN with pad."""
    payload = bytes(payload)
    if len(payload) > PKT_LEN:
        raise ValueError("payload is %d bytes; a packet holds %d" % (len(payload), PKT_LEN))
    return payload + bytes([pad & 0xFF]) * (PKT_LEN - len(payload))


def _lfsr_bytes(seed, n):
    out, v = bytearray(), seed & 0xFF or 1
    for _ in range(n):
        out.append(v)
        v = ((v << 1) | ((v >> 7) ^ (v >> 5) ^ (v >> 4) ^ (v >> 3)) & 1) & 0xFF
    return bytes(out)


# One per slot: easy to recognise on a scope or in a PKR dump.
DEMO = {
    0: bytes(range(256)),                                   # 00 01 .. FF
    1: bytes(range(255, -1, -1)),                           # FF FE .. 00
    2: bytes(256),                                          # all 00
    3: b"\xFF" * 256,                                       # all FF
    4: b"\x55\xAA" * 128,                                   # alternating
    5: bytes(1 << (i % 8) for i in range(256)),             # walking one
    6: _lfsr_bytes(0xA5, 256),                              # pseudo-random
    7: b"\x0D\x0A\xFB\xFD\x1C\xBC\xF7\xFE" * 32,            # CR LF + K-char values as data
}


def read_packet_file(path, pad=0x00):
    """A packet from a file: .bin = raw bytes; anything else = hex text
    (bytes as 2-digit hex, any whitespace/commas, '#' comments). Short
    files are padded to 256."""
    with open(path, "rb") as f:
        raw = f.read()
    if os.path.splitext(path)[1].lower() == ".bin":
        return make_packet(raw, pad)
    text = re.sub(r"#[^\n]*", "", raw.decode("ascii", errors="replace"))
    tokens = re.split(r"[\s,]+", text.strip()) if text.strip() else []
    try:
        data = bytes(int(t, 16) for t in tokens)
    except ValueError:
        raise ValueError("%s: not hex bytes" % path)
    return make_packet(data, pad)


# ---------------------------------------------------------------------------
# Talking to the FPGA
# ---------------------------------------------------------------------------
def load_packets(link, packets, quiet=False):
    """PKL each {slot: data} (data padded to 256 if short). Returns
    {slot: crc}. Raises BusyError / PacketError / CommandError."""
    crcs = {}
    for slot in sorted(packets):
        data = make_packet(packets[slot])
        crcs[slot] = link.pkt_load(slot, data)
        if not quiet:
            print("slot %d loaded, CRC %04X" % (slot, crcs[slot]))
    return crcs


MAX_REPEAT = 127


def tx_cfg(first=0, count=PKT_SLOTS, repeat=0, tx_only=False, se=False):
    """TEST9 config bits 14:0: [2:0] first slot, [5:3] count-1 (1-8, slots
    wrap 7->0), [12:6] extra passes (0-127), [13] GPIO path, [14] TX only.
    (Before 2026-10-02 repeat was [13:6], 0-255, and there was no path bit.)"""
    if not 0 <= first < PKT_SLOTS:
        raise ValueError("first slot must be 0..7")
    if not 1 <= count <= PKT_SLOTS:
        raise ValueError("count must be 1..8")
    if not 0 <= repeat <= MAX_REPEAT:
        raise ValueError("repeat must be 0..%d (extra passes)" % MAX_REPEAT)
    return ((bool(tx_only) << 14) | (bool(se) << 13) | (repeat << 6)
            | ((count - 1) << 3) | first)


def path_name(se):
    return "GPIO" if se else "LVDS"


def run_seconds(count, repeat):
    """Rough wire time of a TEST9 run (SOF + 256 + EOF per packet)."""
    return count * (PKT_LEN + 2) * (repeat + 1) / LINE_BYTES_PER_S


def send(link, first=0, count=PKT_SLOTS, repeat=0, tx_only=False, se=False):
    """Run TEST9 and return newport_tests.decode()'s dict for it."""
    from newport_tests import run_test
    cfg = tx_cfg(first, count, repeat, tx_only, se)
    return run_test(link, TEST9, timeout=1.0 + 2 * run_seconds(count, repeat), cfg=cfg)


def slots_of(first, count):
    return [(first + i) % PKT_SLOTS for i in range(count)]


def parse_range(text):
    """'3' -> (3, 1); '0-7' -> (0, 8); '6-1' -> (6, 4) (wraps 7 -> 0)."""
    m = re.fullmatch(r"([0-7])(?:-([0-7]))?", text.strip())
    if not m:
        raise ValueError("slot range like 3, 0-7 or 6-1 expected, got %r" % text)
    first = int(m.group(1))
    last = int(m.group(2)) if m.group(2) is not None else first
    return first, (last - first) % PKT_SLOTS + 1


def split_file(data, pad=0x00):
    """File bytes -> list of 256-byte packets, the last one padded.
    Returns (packets, pad_bytes)."""
    data = bytes(data)
    if not data:
        raise ValueError("file is empty")
    pkts = [make_packet(data[i:i + PKT_LEN], pad) for i in range(0, len(data), PKT_LEN)]
    return pkts, len(pkts) * PKT_LEN - len(data)


def read_file_data(path):
    """A file to send: .bin = raw bytes, anything else = hex text (same rule
    as read_packet_file and the JTAG send-file entry), any length."""
    with open(path, "rb") as f:
        raw = f.read()
    if os.path.splitext(path)[1].lower() == ".bin":
        return raw
    text = re.sub(r"#[^\n]*", "", raw.decode("ascii", errors="replace"))
    tokens = re.split(r"[\s,]+", text.strip()) if text.strip() else []
    try:
        return bytes(int(t, 16) for t in tokens)
    except ValueError:
        raise ValueError("%s: not hex bytes (use a .bin extension for raw data)" % path)


def send_file(link, data, se=False, check=True, quiet=False):
    """Send data (bytes) as 256-byte packets, 8 per batch: PKL them into
    slots 0..n-1, run TEST9 over the chosen path, next batch. Stops at the
    first failing batch. Returns (passed, last decode dict, batches run,
    batches total)."""
    pkts, pad = split_file(data)
    nb = (len(pkts) + PKT_SLOTS - 1) // PKT_SLOTS
    if not quiet:
        print("%d bytes -> %d packet%s (last padded with %d zero bytes), %d batch%s, over %s%s"
              % (len(data), len(pkts), "" if len(pkts) == 1 else "s", pad, nb,
                 "" if nb == 1 else "es", path_name(se), "" if check else ", TX only"))
    d = None
    for b in range(nb):
        batch = pkts[b * PKT_SLOTS:(b + 1) * PKT_SLOTS]
        load_packets(link, dict(enumerate(batch)), quiet=True)
        d = send(link, 0, len(batch), 0, not check, se)
        if not quiet:
            lo = b * PKT_SLOTS * PKT_LEN
            hi = min(lo + len(batch) * PKT_LEN, len(data)) - 1
            print("  batch %d/%d  bytes %06X-%06X  0x%04X  %s"
                  % (b + 1, nb, lo, hi, d["raw"], d["summary"]))
        if not d["passed"]:
            return False, d, b + 1, nb
    return True, d, nb, nb


def _hexdump(data):
    return "\n".join("  %02X: %s" % (i, data[i:i + 16].hex(" ").upper())
                     for i in range(0, len(data), 16))


# ---------------------------------------------------------------------------
# Menu entry
# ---------------------------------------------------------------------------
def packet_test(serial_number, tx_only=False):
    """Load the DEMO patterns into slots 0-7 and send them once: loopback
    checked (needs the Newport ASIC), or with tx_only just transmitted (bare
    dev kit / scope; passes if everything went out)."""
    import newport_tests as nt

    def body(link, sn):
        try:
            crcs = load_packets(link, DEMO, quiet=True)
        except BusyError:
            print("TEST9 is still running - packets not loaded")
            return 1
        print("Loaded slots 0-7 (CRC %s)" % " ".join("%04X" % crcs[s] for s in sorted(crcs)))
        d = send(link, 0, PKT_SLOTS, 0, tx_only)
        nt._report(d, sn)
        return 0 if d["passed"] else 1
    # DUT supplies up first, except TX-only (no DUT on the board)
    return nt._with_link(body, serial_number, power=not tx_only)


def packet_test_tx_only(serial_number):
    """packet_test without the loopback check (menu entry)."""
    return packet_test(serial_number, tx_only=True)


def _ask(prompt, default=""):
    try:
        ans = input("%s%s: " % (prompt, " [%s]" % default if default else "")).strip()
    except EOFError:
        ans = ""
    return ans or default


GPIO_NOTE = ("GPIO path: FPGA GPIO15 (clock) / GPIO14_DUT_0_0 (data) -> Newport (0,0) "
             "gpio_15/gpio_14,\n  back on gpio_13/gpio_12 -> FPGA GPIO13/GPIO12. NSCIp/NSDIp "
             "are held low for the run.\n  The Newport's own north DFE must have "
             "ioConfig[63] (single_end_bypass) set to receive.")


def _send_file_entry(serial_number, se):
    """Menu body: ask for a file and whether to check the loopback, send
    it over the chosen path, report like the other entries."""
    import newport_tests as nt

    def body(link, sn):
        if se:
            print(GPIO_NOTE)
        path = _ask("File to send over %s (.bin raw, otherwise hex text)" % path_name(se)).strip('"')
        if not path:
            print("no file - nothing sent")
            return 1
        try:
            data = read_file_data(path)
            split_file(data)
        except (OSError, ValueError) as e:
            print("Error: %s" % e)
            return 1
        check = _ask("Check the loopback (needs the Newport looping back)? y/n", "y").lower().startswith("y")
        try:
            ok, d, done, nb = send_file(link, data, se, check)
        except BusyError:
            print("TEST9 is still running - file not sent")
            return 1
        except PacketError as e:
            print("Packet load failed: %s" % e)
            return 1
        d = dict(d)
        d["summary"] = ("PASS - %s over %s, %d bytes, %d batch%s%s"
                        % (os.path.basename(path), path_name(se),
                           len(data), nb, "" if nb == 1 else "es", "" if check else ", TX only")
                        if ok else
                        "%s (batch %d/%d of %s over %s)" % (d["summary"], done, nb,
                                                            os.path.basename(path), path_name(se)))
        nt._report(d, sn)
        return 0 if ok else 1
    return nt._with_link(body, serial_number, power=True)


def send_file_lvds(serial_number):
    """Send a file to the Newport over LVDS (menu entry)."""
    return _send_file_entry(serial_number, se=False)


def send_file_gpio(serial_number):
    """Send a file to the Newport over the single-ended GPIO path (menu entry)."""
    return _send_file_entry(serial_number, se=True)


# ---------------------------------------------------------------------------
# Self-test (simulated FPGA)
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

    crcs = load_packets(link, DEMO, quiet=True)
    check("8 slots loaded, CRCs match", crcs, {s: pkt_crc(DEMO[s]) for s in DEMO})
    check("RAM holds the data", [bytes(sim.pkt[s]) for s in range(8)], [DEMO[s] for s in range(8)])
    check("CR/LF/K values loaded as data", bytes(sim.pkt[7][:4]), b"\x0D\x0A\xFB\xFD")
    check("PKR 8 bytes", link.pkt_read(0, 0xF8), bytes(range(0xF8, 0x100)))
    check("PKR wraps", link.pkt_read(1, 0xFC), bytes([3, 2, 1, 0, 255, 254, 253, 252]))
    check("read_all = slot", link.pkt_read_all(6), DEMO[6])
    check("echo mode back after PKL", link.get_cfg(3), 0)
    check("CRC of 00..FF", pkt_crc(bytes(range(256))), 0x3FBD)   # same value the RTL bench gets

    sim.pkt_busy = True
    try:
        link.pkt_load(2, DEMO[2]); got = "no error"
    except BusyError:
        got = "BusyError"
    check("BSY while TEST9 runs", got, "BusyError")
    sim.pkt_busy = False

    sim.pkl_corrupt = True
    try:
        link.pkt_load(2, DEMO[2]); got = "no error"
    except PacketError as e:
        got = "CRC" in str(e)
    check("CRC mismatch -> PacketError", got, True)

    sim.pkl_stall = 100
    try:
        link.pkt_load(2, DEMO[2]); got = "no error"
    except PacketError as e:
        got = "timed out" in str(e)
    check("TO -> PacketError", got, True)
    link.pkt_load(2, DEMO[2])
    check("reload after TO", bytes(sim.pkt[2]), DEMO[2])

    try:
        link.pkt_load(0, b"short"); got = "no error"
    except ValueError:
        got = "ValueError"
    check("pkt_load wants exactly 256", got, "ValueError")
    check("make_packet pads", make_packet(b"\x01\x02", 0xEE)[:4], b"\x01\x02\xEE\xEE")

    check("tx_cfg 0-7 once", tx_cfg(0, 8), 0x0038)
    check("tx_cfg 6-1 x3 TX only", tx_cfg(*parse_range("6-1"), repeat=2, tx_only=True), 0x409E)
    check("tx_cfg GPIO path = bit 13", tx_cfg(0, 8, se=True), 0x2038)
    check("tx_cfg max repeat 127", tx_cfg(1, 1, repeat=127, se=True), 0x3FC1)
    try:
        tx_cfg(repeat=128); got = "no error"
    except ValueError:
        got = "ValueError"
    check("repeat 128 rejected (bit 13 is the path)", got, "ValueError")
    check("parse_range single", parse_range("3"), (3, 1))
    check("slots_of wrap", slots_of(6, 4), [6, 7, 0, 1])

    d = send(link, 3, 2, 4)
    check("send runs TEST9 with the config", (d["passed"], sim.test9_runs[-1]), (True, 0x010B))
    sim.set_result(9, 0x8019)
    d = send(link)
    check("send reports a bad byte", d["summary"], "FAIL - 1 byte mismatches")
    sim.set_result(9, None)

    # whole file: 20 packets + 17 bytes -> 3 batches (8, 8, 5), last padded
    import random
    rnd = random.Random(7)
    fdata = bytes(rnd.randrange(256) for _ in range(20 * 256 + 17))
    sim.test9_runs.clear(); sim.test9_sent.clear()
    ok, d, done, nb = send_file(link, fdata, se=True, check=True, quiet=True)
    check("send_file GPIO: 3 batches pass", (ok, done, nb), (True, 3, 3))
    check("send_file GPIO: configs", sim.test9_runs, [0x2038, 0x2038, 0x2020])
    check("send_file GPIO: every byte on the wire, in order",
          b"".join(x for _, x in sim.test9_sent), fdata + bytes(256 - 17))
    check("send_file GPIO: path", {p for p, _ in sim.test9_sent}, {"GPIO"})
    sim.test9_runs.clear(); sim.test9_sent.clear()
    ok, d, done, nb = send_file(link, fdata[:256], se=False, check=False, quiet=True)
    check("send_file LVDS TX only, exactly 1 packet", (ok, sim.test9_runs, len(sim.test9_sent[0][1])),
          (True, [0x4000], 256))
    sim.set_result(9, 0xA009)
    sim.test9_runs.clear()
    ok, d, done, nb = send_file(link, fdata, quiet=True)
    check("send_file stops at the first failing batch", (ok, done, nb, len(sim.test9_runs)), (False, 1, 3, 1))
    sim.set_result(9, None)
    try:
        split_file(b""); got = "no error"
    except ValueError:
        got = "ValueError"
    check("empty file rejected", got, "ValueError")

    with tempfile.TemporaryDirectory() as tmp:
        pb, ph = os.path.join(tmp, "p.bin"), os.path.join(tmp, "p.hex")
        with open(pb, "wb") as f:
            f.write(b"\xAB" * 10)
        with open(ph, "w") as f:
            f.write("# header\n01 02, 03\nff\n")
        check("read .bin (padded)", read_packet_file(pb)[:12], b"\xAB" * 10 + b"\x00\x00")
        check("read hex text", read_packet_file(ph)[:5], b"\x01\x02\x03\xFF\x00")
        with open(pb, "wb") as f:
            f.write(fdata)
        check("read_file_data .bin, any length", read_file_data(pb), fdata)
        check("read_file_data hex text", read_file_data(ph), b"\x01\x02\x03\xFF")

        # menu entry, scripted answers
        import builtins
        import newport_tests as nt
        answers = iter([pb, "n"])
        real = builtins.input, nt.get_link, nt._append_csv
        builtins.input = lambda prompt="": next(answers)
        nt.get_link = lambda: link
        nt._append_csv = lambda *args: None          # keep the selftest out of the results CSV
        sim.test9_runs.clear()
        try:
            rc = send_file_gpio("SELFTEST")
        finally:
            builtins.input, nt.get_link, nt._append_csv = real
        check("menu send_file_gpio (TX only)", (rc, sim.test9_runs), (0, [0x6038, 0x6038, 0x6020]))

    print("SELFTEST", "PASS" if fails == 0 else "FAIL (%d)" % fails)
    return fails == 0


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------
def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="Load packets into the Newport FPGA and send them (TEST9)")
    ap.add_argument("--load", nargs="+", metavar="SLOT=FILE", default=[],
                    help="load files into slots (.bin raw, otherwise hex text)")
    ap.add_argument("--demo", action="store_true", help="load the 8 built-in test patterns")
    ap.add_argument("--send", metavar="RANGE", help="send slots, e.g. 3, 0-7, 6-1 (wraps)")
    ap.add_argument("--repeat", type=int, default=0, help="extra passes, 0-127 (default 0)")
    ap.add_argument("--tx-only", action="store_true", help="don't check loopback (no DUT / scope only)")
    ap.add_argument("--gpio", action="store_true",
                    help="send over the single-ended GPIO path instead of LVDS")
    ap.add_argument("--file", metavar="FILE",
                    help="send a whole file (.bin raw, otherwise hex text), 8 packets a batch")
    ap.add_argument("--dump", type=int, metavar="SLOT", help="read a slot back and print it")
    ap.add_argument("--port", default=None, help="serial port (default: cfg_com.NEWPORT_port or COM3)")
    ap.add_argument("--serial", default="bench", help="DUT serial number for the log")
    ap.add_argument("--debug", action="store_true", help="use the simulated FPGA")
    ap.add_argument("--selftest", action="store_true", help="offline checks of this module")
    a = ap.parse_args(argv)
    if a.selftest:
        return 0 if selftest() else 1

    packets = dict(DEMO) if a.demo else {}
    for item in a.load:
        slot, _, path = item.partition("=")
        if not slot.isdigit() or not 0 <= int(slot) < PKT_SLOTS or not path:
            ap.error("--load wants SLOT=FILE with SLOT 0-7, got %r" % item)
        packets[int(slot)] = read_packet_file(path)
    if not (packets or a.send or a.file or a.dump is not None):
        packets, a.send = dict(DEMO), "0-7"          # no arguments: demo run

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
        if packets:
            load_packets(link, packets)
        if a.dump is not None:
            print("slot %d:\n%s" % (a.dump, _hexdump(link.pkt_read_all(a.dump))))
        if a.send:
            first, count = parse_range(a.send)
            print("TEST9: slots %s x %d over %s%s  (~%.2f ms on the wire)"
                  % (",".join(map(str, slots_of(first, count))), a.repeat + 1, path_name(a.gpio),
                     ", TX only" if a.tx_only else "", 1000 * run_seconds(count, a.repeat)))
            d = send(link, first, count, a.repeat, a.tx_only, a.gpio)
            nt._report(d, a.serial)
            rc = 0 if d["passed"] else 1
        if a.file:
            if a.gpio:
                print(GPIO_NOTE)
            ok, d, done, nb = send_file(link, read_file_data(a.file), a.gpio, not a.tx_only)
            print("%s: %s" % (a.file, "PASS" if ok else "FAIL at batch %d/%d" % (done, nb)))
            rc = rc or (0 if ok else 1)
    except (NewportError, ValueError, OSError) as e:
        print("Error: %s" % e)
        rc = 1
    finally:
        nt.close_link()
    return rc


if __name__ == "__main__":
    sys.exit(main())
