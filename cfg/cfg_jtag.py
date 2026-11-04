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
#  Purpose: Newport DUT JTAG (TAP) settings for the Newport Test Exec.
#           Read by newport\newport_jtag.py.
#
#-------------------------------------------------------------------------------
#
# *** PLACEHOLDERS (2026-10-01) ***  IR_LEN, the instruction codes, TRST
# polarity and the TCK limit are NOT from the Newport spec yet. Fill in the
# real values before trusting any result. Nothing else needs to change: the
# FPGA only moves bits, all of the Newport-specific knowledge is here.
#
# Newport's TAP is standard IEEE 1149.1 except that it has no BYPASS. On
# this board every chip has its own TDI/TDO, so nothing is daisy-chained and
# BYPASS is never needed. Don't map an instruction to all 1s unless Newport
# really defines one there.
#
# IR_LEN           instruction register length, bits
# INSTRUCTIONS     name -> IR code. Names are free; the two below are used
#                  by the menu entries.
# LOAD_INSTR       instruction whose data register takes the file data
#                  ("Newport JTAG: send file" and newport_jtag.py --file)
# IDCODE_INSTR     instruction that selects the 32-bit IDCODE register, or
#                  None if Newport has none ("Newport JTAG: read IDCODE")
# IDCODE_EXPECT    expected IDCODE (int), or None to only check that each
#                  chip answers with something other than 00000000/FFFFFFFF
# TRST_ACTIVE_LOW  True for the IEEE 1149.1 TRST* (low = reset)
# TCK_HZ           TCK frequency; the FPGA supports 3.8 kHz .. 31.25 MHz in
#                  steps of 250 MHz / (2 * n)
# RUNTEST_CLOCKS   extra TCK clocks in Run-Test/Idle after each data
#                  register update (some memory loads need idle clocks)
#
#-------------------------------------------------------------------------------

IR_LEN = 5

INSTRUCTIONS = {
    "IDCODE": 0x01,
    "LOAD":   0x03,
}

LOAD_INSTR = "LOAD"
IDCODE_INSTR = "IDCODE"
IDCODE_EXPECT = None

TRST_ACTIVE_LOW = True
TCK_HZ = 1_000_000
RUNTEST_CLOCKS = 0
