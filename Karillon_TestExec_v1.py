#-------------------------------------------------------------------------------
# 
#  Karillon Corporation
#  Colorado Springs, Colorado
# 
#  Copyright (c) 2024 by Karillon Corp. 
#  Colorado Springs, Colorado. USA
# 
#  This code is the property of Karillon Corporation (Karillon) and may not
#  be redistributed in any form without prior written permission from Karillon
# 
#  Purpose: Test Executive for automated electronic test & evaluation.
#
#  Usage:
#    python Karillon_TestExec_v1.py [--eng] [--debug] [--logfile FILE]
#                                   [--loglevel LEVEL]
#    python Karillon_TestExec_v1.py --help
#
#  --eng     Engineering mode. The DUT serial number is remembered for the
#            session and offered at the next prompt, so <Enter> reuses it
#            (type a new one to change it). Without --eng the exec runs in
#            technician mode: the serial number must be typed in for every
#            test sequence.
#
#  --debug   Debug mode. Every instrument driver (k_instruments) and Karillon
#            board driver (K_MinimaR4, K_Uno, K_Nucleo, K_TCB) skips its real
#            connection and a simulated instrument answers instead, so a
#            test can be run and debugged with no hardware attached. The
#            simulation lives in K_Env\lib\python\k_lib\K_Sim.py. Also
#            switches the log format to include file and line numbers. Add
#            --loglevel DEBUG to see every command sent to the simulated
#            instruments.
#
#------------------------------------------------------------------------------
#
# REVISION HISTORY:
#
#  $Author: $
#  $Rev: $
#  $Date: $
#  $URL: $
#
#------------------------------------------------------------------------------

import argparse
from contextlib import contextmanager
import datetime
# ngb import fcntl
import json
import logging
import re
import socket
import string
import subprocess
import sys
import time
import winsound
from termcolor import colored

'''Color examples
print(colored("black", "black"))
print(colored("red", "red"))
print(colored("green", "green"))
print(colored("yellow", "yellow"))
print(colored("blue", "blue"))
print(colored("magenta", "magenta"))
print(colored("cyan", "cyan"))
print(colored("light gray", "light_grey"))
print(colored("dark gray", "dark_grey"))
print(colored("bright red", "light_red"))
print(colored("bright green", "light_green"))
print(colored("bright yellow", "light_yellow"))
print(colored("bright blue", "light_blue"))
print(colored("bright magenta", "light_magenta"))
print(colored("bright cyan", "light_cyan"))
print(colored("white", "white"))
'''

sys.path.append('./cfg/')
sys.path.append('./tests/')
import K_cfg
#import K_cfg_tests

#sys.path.append("C:\\python\\K_lib")
sys.path.append("G:\\Shared drives\\K_Env\\lib\\python\\K_lib")
import K_lib
import K_Sim    # debug-mode instrument simulation (see --debug)

from K_WindowSize import *
from K_Logo import *
from K_Menu import *
from K_lib import *
from K_AnsiLogo import *    # ANSI art banner, ASCII K_Logo fallback

#import test_noload_v2
#import test_led1

import instrument_interactive_v1

#  Newport FPGA tests: link, runner/decoder, simulated FPGA and PMBus control
#  live in ./newport/ (see newport_tests.py). The FPGA port is
#  cfg_com.NEWPORT_port; if it doesn't answer, the user is offered the other
#  serial ports on the machine.
sys.path.append('./newport/')
import newport_tests
import newport_pkt
import fpga_terminal       # fpga> raw FPGA commands (was newport_terminal.py)
import newport_terminal    # newport> DUT commands (placeholder)
import newport_jtag
import newport_screen

#===============================================================================
#  Menus
#
#  The main menu (MAIN_MENU, numbered from 0) picks one submenu: Tests, Power
#  or Utilities (2026-10-11; before that every row was on one long menu).
#  Each submenu is numbered from 0 and takes a single number, a list
#  ("1, 3, 12") or a range ("10-12"); the selected rows run in the order
#  typed. <Enter> on a submenu goes back to the main menu; <Enter> on the
#  main menu quits. Only Tests asks for the DUT serial number; Power and
#  Utilities entries are passed an empty one ("").
#
#  To add an entry: import its module above, then add a row to the submenu
#  it belongs in. Each row is (menu label, test function, name used in the
#  start/end and pass/fail messages). The function takes the DUT serial
#  number and returns 0 for pass, anything else for fail. For an older-style
#  test module, use its test1 function (e.g. instrument_interactive_v1.test1).
#===============================================================================
npt = newport_tests
TESTS_MENU = [
    ("Newport TEST0 basic",                npt.slot_test(0),      "TEST0 basic"),
    ("Newport TEST1 short",                npt.slot_test(1),      "TEST1 short"),
    ("Newport TEST2 break",                npt.slot_test(2),      "TEST2 break"),
    ("Newport TEST3 error",                npt.slot_test(3),      "TEST3 error"),
    ("Newport TEST4 pauseErr",             npt.slot_test(4),      "TEST4 pauseErr"),
    ("Newport TEST5 singleEnded",          npt.slot_test(5),      "TEST5 singleEnded"),
    ("Newport TEST6 ioConfig",             npt.slot_test(6),      "TEST6 ioConfig"),
    ("Newport TEST7 gpioRx (exercise only)", npt.slot_test(7),    "TEST7 gpioRx"),
    ("Newport TEST8 gpioTx (exercise only)", npt.slot_test(8),    "TEST8 gpioTx"),
    ("Newport TEST9 packets (8 x 256 B, loopback checked)", newport_pkt.packet_test, "TEST9 packets"),
    ("Newport TEST9 packets, TX only (no DUT / scope)", newport_pkt.packet_test_tx_only, "TEST9 packets TX only"),
    ("Run all Newport tests (TEST0-8)",    npt.run_all,           "Newport TEST0-8"),
    # TEST10 part-screening batch loop (added 2026-10-09) -- placeholder FPGA
    # test, see newport_screen.py and newport_tests.PARTSCREEN_TEST. Unlike
    # every other row here this one runs its own multi-part loop (its own SN
    # prompting, its own log file) rather than a single pass/fail per call.
    ("Newport: Part screening (placeholder FPGA test, TEST10)", newport_screen.screen_session, "Part screening"),
]

POWER_MENU = [
    ("Regulator telemetry (MPQ8785 x3)",   npt.regulator_telemetry, "Regulator telemetry"),
    # DUT power (newport_power.py, 2026-10-11): sequenced -- up is VDDIO,
    # wait for its POWER_GOOD, then VDDCORE; down is the reverse. DUT tests
    # power up by themselves when needed. A POWER_GOOD change, or a
    # USB-Blaster on the FPGA's JTAG header, stops testing and powers both
    # supplies down. Single control lines: PWR s b in the fpga> terminal.
    ("Newport DUT power up (VDDIO -> VDDCORE)",   npt.dut_power_up,     "DUT power up"),
    ("Newport DUT power down (VDDCORE -> VDDIO)", npt.dut_power_down,   "DUT power down"),
    ("Newport DUT power status",                  npt.dut_power_status, "DUT power status"),
]

UTILITIES_MENU = [
    ("Newport link check (UART + bitstream)", npt.link_check,     "Newport link check"),
    ("FPGA terminal (fpga> raw FPGA commands)", fpga_terminal.terminal, "FPGA terminal"),
    ("Newport terminal (newport> DUT commands - placeholder)", newport_terminal.terminal, "Newport terminal"),
    ("Newport JTAG: read IDCODE (9 chips)", newport_jtag.jtag_idcode,   "JTAG IDCODE"),
    ("Newport LVDS: send file to DUT (0,0)", newport_pkt.send_file_lvds, "LVDS send file"),
    ("Newport GPIO: send file to DUT (0,0), single-ended", newport_pkt.send_file_gpio, "GPIO send file"),
    ("Newport JTAG: send file to DUT(s)",   newport_jtag.jtag_send_file, "JTAG send file"),
    ("Instr Interactive",                  instrument_interactive_v1.test1, "Instrument Interactive"),
]

#  (title, rows, ask for serial number) for each main-menu entry, in
#  display order.
MAIN_MENU = [
    ("Tests",     TESTS_MENU,     True),
    ("Power",     POWER_MENU,     False),
    ("Utilities", UTILITIES_MENU, False),
]

#  Every row in one flat list, for code that wants to look a test up by
#  label or function rather than by menu position.
TEST_MENU = TESTS_MENU + POWER_MENU + UTILITIES_MENU

#===============================================================================
#  Karillon Testing
#===============================================================================
class ktest():
    """
    Library of standard Karillon functions and parameters.
    """

    #===========================================================================
    #  __init__
    #===========================================================================
    def __init__(self):
        """
        Def documentation goes here.
        """
        
        self.serial_number = ''
        self.eng = False        # engineering mode (--eng); see get_serial_number


    #===========================================================================
    #  get_serial_number
    #===========================================================================
    def get_serial_number(self):
        """
        Ask for the DUT serial number before a test sequence.

        Technician mode: K_lib.getSerNum(), which makes the technician type
        it in every time.
        Engineering mode (--eng): the last serial number is offered in
        brackets and <Enter> reuses it; typing a new one replaces it. The
        first time (nothing to offer yet) it must be typed in.
        """
        if not self.eng or not self.serial_number:
            if not self.eng:
                return K_lib.getSerNum()
            sn = ''
            while not sn:                  # spaces alone don't count
                sn = K_lib.getSerNum().strip()
            return sn
        reply = input('\nEnter DUT serial number [' + self.serial_number +
                      ']:').strip()
        return reply or self.serial_number


    #===========================================================================
    # bitwise_not
    #===========================================================================
    @staticmethod
    def bitwise_not(theInt):
        """
        This method returns the bitwise not of an integer.  Since python uses
        signed values, ~value returns the two's complement instead of the
        bitwise not value.  For example, to return the bitwise not of an 8-bit
        value, subtract it from 0x1 0000 0000 and then subtract one.
         """
        return (1 << theInt.bit_length()) - 1 - n

    #===========================================================================
    # current_time
    #===========================================================================
    @staticmethod
    def current_time():
        """
        This method returns the current time and the current time in a string
        format like this:
            2018_12_20_11_38_32_991324

        """

        current_time = datetime.datetime.now()
        current_time_str = str(current_time)

        #  Turn a time value like this: 2018-12-20 11:38:32.991324
        #  into a string like this:     2018_12_20_11_38_32_991324
        replaceList = [':',"-","."," "]
        for item in replaceList:
            if item in str(current_time_str):
                current_time_str = current_time_str.replace(item, "_")

        return current_time, current_time_str



    #===========================================================================
    #  open_exclusive
    #===========================================================================
    @staticmethod
    @contextmanager
    def open_exclusive(filename, *args, **kwargs):
        '''
        This function will make sure that only one process at a time can
        access a file.
        '''

        #  Time to wait for timeout and retry when trying to exclusively open
        #  a file.
        timeout = 60
        retry = 1

        deadline = datetime.datetime.now() + \
            datetime.timedelta(seconds = timeout)


#        print"Karillon:open_exclusive: " + filename + "."
#        if filename == "":
#            print "empty filename:"
#            raise (ValueError)



        continueLoop = True
        while continueLoop:
            try:
               with open(filename, *args, **kwargs) as fh:
                    # ngb fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    continueLoop = False
                    yield fh
            #except BlockingIOError as excp:
            #except IOError as excp:
            #    print"Karillon:open_exclusive:\n", excp
            #    continueLoop = False
            #    yield None
        #except:
            #    if datetime.datetime.now() > deadline:
            #        # ngb raise excp
            #        raise
            #    else:
            #        time.sleep(retry)

            #except ValueError as excp:
            except IOError as excp:
                if str(excp).find("No such file or directory") >= 0:
                    raise excp
                if str(excp).find("Bad file descriptor") >= 0:
                    raise excp

                if datetime.datetime.now() > deadline:
                    continueLoop = False
                    yield None
                    raise excp
                else:
                    time.sleep(retry)


    #===========================================================================
    # setup_parser
    #===========================================================================
    @staticmethod
    def setup_parser():
        """
        Def documentation goes here.
        """

        parser = argparse.ArgumentParser(
            description = 'Karillon Test Executive: menu-driven automated '
                          'test & evaluation.',
            epilog = 'Example: python Karillon_TestExec_v1.py --debug '
                     '--loglevel DEBUG   (run tests against simulated '
                     'instruments and log every simulated command)'
        )

        #  --eng: engineering mode (remember and offer the serial number);
        #  the default is technician mode (type it in every time).
        parser.add_argument(
            '--eng',
            dest = 'eng',
            default = False,
            action = "store_true",
            help = 'Engineering mode: remember the DUT serial number and '
                   'offer it at the next prompt (<Enter> reuses it). '
                   'Without --eng (technician mode) it must be typed in '
                   'for every test sequence.'
        )

        #  --debug: simulate all instruments so tests can be debugged without
        #  hardware. K_Sim.set_debug() is called in run() before any test
        #  creates an instrument object; each driver checks K_Sim.is_debug()
        #  in its constructor.
        parser.add_argument(
            '--debug',
            dest = 'debug',
            default = False,
            action = "store_true",
            help = 'Debug mode: simulate all instruments (no hardware or '
                   'network needed) so tests can be run and debugged, and '
                   'include file/line numbers in the log.'
        )
        parser.add_argument(
            '--logfile',
            dest = 'logfile',
            nargs = "?",
            default = None,
            help = 'Name of file for logging messages.'
        )
        parser.add_argument(
            '--loglevel',
            dest = 'loglevel',
            nargs = "?",
            default = 'INFO',
            help = 'Logging level: DEBUG, INFO, WARNING, ERROR, CRITICAL. '
                   'With --debug, DEBUG also logs every command sent to the '
                   'simulated instruments.'
        )
        parser.add_argument(
            '--verbosity',
            dest = 'verbosity',
            nargs = "?",
            default = 2,
            help = 'Verbosity level (not used yet)'
        )

        #return parser.parse_args()
        return parser


    #===========================================================================
    #  print_addrinfo
    #===========================================================================
    @staticmethod
    def print_addrinfo(host = None, port = 50007):
        """
        This function prints socket addrinfo
        """

        HOST = host
        PORT = port

        # ngb for res in socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC):
        results = socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC)
        for res in results:

            afamily, socktype, proto, canonname, sockaddr = res

            mystr = "{:s}={:22s} {:s}={:22s} {:s}={:2s} {:s}={:s} {:s}={:s}".format(
                "afamily", str(afamily),
                "socktype", str(socktype),
                "proto", str(proto),
                "canonname", str(canonname),
                "sockaddr", str(sockaddr))

            print(mystr)



    #===========================================================================
    #  run_cmd
    #===========================================================================
    @staticmethod
    def run_cmd(cmd):
        """
        This method runs the specified command on the OS and returns a tuple
        which is the exit code and either stdout or stderr depending if the
        command completed successfully.
        INPUT:
        cmd - OS command to execute
        OUTPUT:
        exitcode, result - Where exitcode is the OS exit code for the command
        and result is either stdout or stderr from the OS.

        """

        proc = subprocess.Popen(cmd, shell=True,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)

        stdout, stderr = proc.communicate()
        exit_code = proc.wait()

        if len(stderr) > 0:
            return exit_code, stderr
        else:
            return exit_code, stdout


    #===========================================================================
    #  select_addrinfo
    #===========================================================================
    @staticmethod
    def select_addrinfo(host = None, port = 50007):
        """
        This function returns chosen the socket addrinfo .
        """

        HOST = host
        PORT = port

        menu_items = []

        # ngb for res in socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC):
        results = socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC)
        for res in results:

            family, socktype, proto, canonname, sockaddr = res

            mystr = "{:s}={:22s} {:s}={:22s} {:s}={:2s} {:s}={:s} {:s}={:s}".format(
                "family", str(family),
                "socktype", str(socktype),
                "proto", str(proto),
                "canonname", str(canonname),
                "sockaddr", str(sockaddr))

            menu_items.append(mystr)

        menu = K_Menu("Select addrinfo", menu_items,
                       single_choice = True,
                       clear_screen = False,
                       default = (len(menu_items) - 1))

        return results[menu.choose()]


    #===========================================================================
    # setup_logging
    #===========================================================================
    @staticmethod
    def setup_logging(**kwargs):
        """
        Def documentation goes here.
        """

        if kwargs['logfile'] == None:
            # ngb self.logfilename = self.name + "_" + self.start_time_str + \
                # ngb ".log"
            logfilename = "Karillon_TestExec.log"
        else:
            logfilename = kwargs['logfile']

        if kwargs['debug'] == True:
            logging.basicConfig(
                level = kwargs['loglevel'],
                format='%(pathname)s\n%(levelname)s: ' + \
                    'line %(lineno)d: %(message)s',
                handlers = [
                    logging.FileHandler(logfilename),
                    logging.StreamHandler()
                    ]
                )
        else:
            logging.basicConfig(
                level = kwargs['loglevel'],
                format='{asctime} - {levelname} - {message}',
                style="{",
                datefmt="%Y-%m-%d %H:%M",
                handlers = [
                    logging.FileHandler(logfilename),
                    logging.StreamHandler()
                    ]
                )

        log = logging.getLogger()

        return log


    #===========================================================================
    #  run
    #===========================================================================
    def run(self, argv=None):
        #  Parse the command line (see setup_parser / --help).
        args = self.setup_parser().parse_args(argv)

        #  Debug mode must be on before any test creates an instrument object,
        #  because each driver decides real vs. simulated in its constructor.
        if args.debug:
            K_Sim.set_debug(True)
        self.eng = args.eng

        clearScreen()

        log = self.setup_logging(loglevel=args.loglevel.upper(),
                                 logfile=args.logfile or 'test_exec.log',
                                 debug=args.debug)
        logging.info("Log file created")
        if args.debug:
            logging.warning("DEBUG MODE: all instruments are simulated")
        logging.info("ENGINEERING MODE: serial number remembered" if self.eng
                     else "Technician mode: serial number entered for each run")
                
        banner_msg = 'Karillon Test Executive v2026' + \
                     ('   [ENGINEERING MODE]' if self.eng else '') + \
                     ('   [DEBUG: SIMULATED INSTRUMENTS]' if args.debug else '')
        logo = K_AnsiLogo(clear_screen=False, color=False,
                          title_banner_msg=banner_msg)

        #  Main menu: one choice, which submenu to open. Submenus: any
        #  number of rows. Both numbered from 0.
        main_menu = K_Menu("Main menu",
                           [title for title, rows, ask_sn in MAIN_MENU],
                           start_index=0,
                           single_choice=True,
                           color=False,
                           msg=banner_msg,
                           logo=logo)
        submenus = [K_Menu(title + " - select one or more",
                           [label for label, test_fn, name in rows],
                           start_index=0,
                           color=False,
                           msg=banner_msg,
                           logo=logo)
                    for title, rows, ask_sn in MAIN_MENU]

        while True:
            try:
                choice = main_menu.choose()
                if choice is None:              # <Enter> on the main menu: quit
                    newport_tests.close_link()   # release the FPGA COM port
                    break
                title, rows, ask_sn = MAIN_MENU[choice]

                #  Stay in this submenu until <Enter> takes us back.
                while True:
                    #  results is a list of 0-based indexes returned from
                    #  choose(), in the order the user entered them.
                    results = submenus[choice].choose()
                    if not results:
                        break
                    self.run_selection(rows, results, ask_sn)

            except Exception as e:
                print(colored("Exception Encountered", "light_red"))
                print(e)


    #===========================================================================
    #  run_selection
    #===========================================================================
    def run_selection(self, rows, results, ask_sn=True):
        """
        Run the selected rows of one submenu, in the order chosen. Asks for
        the serial number first if ask_sn (Tests only); otherwise each entry
        gets "" and the remembered engineering-mode serial number is left
        alone. Plays the fanfare only if every selected test passed; stops
        early on a DUT power fault.
        """
        winsound.PlaySound('./sounds/RobotBlip.wav', winsound.SND_ALIAS)
        if ask_sn:
            self.serial_number = self.get_serial_number()
            sn = self.serial_number
        else:
            sn = ""

        #  Fanfare only if every selected test passed.
        noFanfare = 0
        for index in results:
            winsound.PlaySound('./sounds/BeepPing.wav', winsound.SND_ALIAS)
            label, test_fn, name = rows[index]

            print(colored("Start " + name, "blue"))
            logging.info("Start " + name + " (" + test_fn.__name__ + ")" +
                         (", serial number " + str(sn) if ask_sn else ""))
            if test_fn(sn) == 0:
                print(colored(name + " passed", "green"))
                logging.info(name + " passed")
            else:
                print(colored(name + " failed", "light_red"))
                logging.info(name + " failed")
                noFanfare = 1
            print(colored("End " + name, "blue"))
            #  A DUT power fault (POWER_GOOD change, USB-Blaster on the
            #  FPGA's JTAG header) stops the rest of the sequence.
            fault = newport_tests.power_fault()
            if fault:
                print(colored("DUT power fault: " + fault + " -- remaining "
                              "selected tests not run", "light_red"))
                logging.error("DUT power fault: " + fault +
                              " -- remaining selected tests not run")
                noFanfare = 1
                break

        print('Test Sequence Complete')
        if noFanfare == 0:
            winsound.PlaySound('./sounds/fanfare.wav', winsound.SND_ALIAS)

#===============================================================================
#  __main__
#===============================================================================
if (__name__ == "__main__"):
    prog = ktest()
    try:
        prog.run()
    finally:
        newport_tests.close_link()   # DUT supplies down, FPGA COM port released

