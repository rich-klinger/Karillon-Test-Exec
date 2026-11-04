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
#  Purpose: DUT supply-rail (MPQ8785 PMBus buck) configuration for the
#           Newport Test Exec. Read by newport\fpga_pmbus.py and
#           newport\newport_power.py (POWER, at the end).
#
#-------------------------------------------------------------------------------
#
# Board: NPT100-EVB-array Rev A0 (Grinn schematic 25/09/2026, sheets 15-16).
#   vddcore  U10 + U11, 2-phase, ADDR 3.32k/47R  -> 0x20 (both phases)
#   vddio    U12,       1 phase, ADDR 3.32k/124R -> 0x22
#
# nominal / tol  DUT supply spec. The "low" and "high" corners are
#                nominal * (1 -/+ tol), rounded toward nominal to a VOUT code.
# settings       Registers written when the Test Exec first connects, before
#                the voltage is set. Engineering units; any field listed in
#                fpga_pmbus.SCALED can go here. None = leave the part's own
#                (MTP) value alone. Writes aren't stored to MTP.
#   IOUT_OC_FAULT_LIMIT  total output over-current fault, A (1 A/LSB, 0-1023).
#                        On vddcore this is the 2-phase total -- confirm.
#   IOUT_OC_WARN_LIMIT   over-current warning, A (1 A/LSB).
#   Also available: OT_FAULT_LIMIT / OT_WARN_LIMIT (C), TON_DELAY / TON_RISE /
#   TOFF_DELAY / TOFF_FALL (ms, 0.1 ms/LSB), VIN_ON / VIN_OFF (V, 0.2 V/LSB).
#
#-------------------------------------------------------------------------------

RAILS = {
    "vddcore": {
        "addr": 0x20,
        "nominal": 0.800,
        "tol": 0.03,
        "settings": {
            "IOUT_OC_FAULT_LIMIT": None,
            "IOUT_OC_WARN_LIMIT": None,
        },
    },
    "vddio": {
        "addr": 0x22,
        "nominal": 1.800,
        "tol": 0.03,
        "settings": {
            "IOUT_OC_FAULT_LIMIT": None,
            "IOUT_OC_WARN_LIMIT": None,
        },
    },
}

RAIL_ORDER = ["vddio", "vddcore"]   # power-up order (down is the reverse): VDDIO first

STARTUP_RAILS = "nom"       # voltage each rail is set to at power-up: nom/low/high
PEC = False                 # SMBus PEC on every regulator transaction
VERIFY_TOL = 0.01           # READ_VOUT must be within 1% of the setpoint
SETTLE_S = 0.05             # s after a VOUT change before READ_VOUT
SWEEP_LIMIT = 0.10          # refuse any setpoint more than 10% from nominal

#-------------------------------------------------------------------------------
# DUT power sequencing (newport_power.py, 2026-10-11)
#
# Power-up: VDDIO, wait for its POWER_GOOD, then VDDCORE (RAIL_ORDER above).
# Power-down: the reverse. A POWER_GOOD that changes on its own, or a
# USB-Blaster on the FPGA's JTAG header, stops testing and powers both down;
# that stays latched until a manual "DUT power up".
#
# POWER_BETWEEN_TESTS  "keep": rails stay up between DUT tests (power comes
#                      up for the first one); "cycle": powered down after
#                      each. Part screening always powers down between parts.
# PG_TIMEOUT_S         ON -> POWER_GOOD, per rail, before power-up fails
# RAIL_DELAY_S         extra wait after a rail is good, before the next one
# OFF_TIMEOUT_S        ON off -> POWER_GOOD low and VOUT < OFF_VOLTS
# OFF_VOLTS            READ_VOUT below this counts as off (V)
# PG_POLL_S            POWER_GOOD monitor: GET PWR this often (each poll is
#                      one short UART command; keep the FT230X latency timer
#                      at 1 ms in Windows)
# PG_GRACE_S           monitor ignores POWER_GOOD this long after a PWR command
# PG_SOURCE            "fpga": the FPGA's POWER_GOOD inputs (GET PWR);
#                      "pmbus": each regulator's STATUS_WORD PG_STATUS
# ON_OFF_CONFIG        written to each regulator (02h) while it's off so CTRL
#                      controls it: 0x16 = CTRL only, active high. None = leave
# RST_FUNCTION         True: make ALT/RST the RST input (DFh bit 0 = 0)
# CABLE_INTERLOCK      True: a USB-Blaster on the JTAG header blocks power
#-------------------------------------------------------------------------------

POWER = {
    "POWER_BETWEEN_TESTS": "keep",
    "PG_TIMEOUT_S": 0.5,
    "RAIL_DELAY_S": 0.0,
    "OFF_TIMEOUT_S": 0.5,
    "OFF_VOLTS": 0.10,
    "PG_POLL_S": 0.5,
    "PG_GRACE_S": 0.5,
    "PG_SOURCE": "fpga",
    "ON_OFF_CONFIG": 0x16,
    "RST_FUNCTION": True,
    "CABLE_INTERLOCK": True,
}
