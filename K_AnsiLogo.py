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
#  Purpose: ANSI art banner for the Test Executive. Draws the ANSI art in
#           ANSI_ART_FILE when the terminal can show ANSI color and Unicode
#           block characters and is wide enough, and falls back to the K_Logo
#           ASCII banner when it can't.
#
#  Usage:
#    logo = K_AnsiLogo(clear_screen=False, color=False, title_banner_msg=msg)
#    menu = K_Menu("Select one", items, logo=logo, ...)
#
#  Set K_ASCII_LOGO=1 (or NO_COLOR=1) in the environment to force the ASCII
#  banner. Run this file directly to preview the banner.
#
#-------------------------------------------------------------------------------

import os
import re
import sys

from K_Logo import *

#===============================================================================
#  ANSI art
#
#  The banner is read from ANSI_ART_FILE, a UTF-8 text file with color codes
#  inline (ESC = "\033"). 256-color codes ("\033[38;5;Nm") work in Windows
#  Terminal and Windows 10+ consoles; 24-bit ("\033[38;2;R;G;Bm") works in
#  Windows Terminal. If the file can't be read, the ASCII banner is used.
#===============================================================================
ANSI_ART_FILE = "G:\\Shared drives\\K_01\\Docs\\General\\Business\\Marketing" \
                "\\Logos\\Karillon_Logo_ansi-art.utf.ans"

_ANSI_CODE = re.compile(r"\033\[[0-9;]*m")


#===============================================================================
#  load_ansi_art
#===============================================================================
def load_ansi_art(path=ANSI_ART_FILE):
    """
    Return the lines of the ANSI art file (trailing blank lines dropped), or
    None if it can't be read.
    """
    try:
        with open(path, encoding="utf-8-sig") as f:
            lines = [line.rstrip("\r\n") for line in f]
    except (OSError, UnicodeDecodeError):
        return None
    while lines and _ANSI_CODE.sub("", lines[-1]).strip() == "":
        lines.pop()
    return lines or None


#===============================================================================
#  ansi_supported
#===============================================================================
def ansi_supported():
    """
    True when stdout is a terminal that will show ANSI color and the Unicode
    block characters used in ANSI_ART. On Windows this also switches on
    virtual terminal processing for the console, which older consoles need.
    """
    if os.environ.get("K_ASCII_LOGO") or os.environ.get("NO_COLOR"):
        return False
    if not sys.stdout.isatty() or os.environ.get("TERM") == "dumb":
        return False

    #  The console encoding must be able to print the block characters.
    try:
        "█▐▛▄".encode(sys.stdout.encoding or "ascii")
    except (UnicodeEncodeError, LookupError):
        return False

    if os.name != "nt":
        return True

    #  Windows: turn on ENABLE_VIRTUAL_TERMINAL_PROCESSING (0x0004). If the
    #  console refuses, it can't interpret ANSI codes.
    try:
        import ctypes
        from ctypes import wintypes
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        handle = kernel32.GetStdHandle(-11)     # STD_OUTPUT_HANDLE
        mode = wintypes.DWORD()
        if not kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
            return False
        return bool(kernel32.SetConsoleMode(handle, mode.value | 0x0004))
    except Exception:
        return False


#===============================================================================
#  K_AnsiLogo
#===============================================================================
class K_AnsiLogo(K_Logo):
    """
    K_Logo that draws the ANSI art file when the terminal supports it,
    otherwise the standard K_Logo ASCII banner.
    """

    def __init__(self, *args, **kwargs):
        #  Set before K_Logo.__init__, which calls update().
        self.art = load_ansi_art() if ansi_supported() else None
        K_Logo.__init__(self, *args, **kwargs)

    #===========================================================================
    #  update
    #===========================================================================
    def update(self, title_banner_msg=''):
        if not self.art:
            return K_Logo.update(self, title_banner_msg)

        #  Width of the widest line, ignoring color codes. K_Logo keeps sizeX
        #  one less than the window width. If the window is too narrow the
        #  art would wrap, so use the ASCII banner instead.
        width = max(len(_ANSI_CODE.sub("", line)) for line in self.art)
        if width > self.sizeX + 1:
            return K_Logo.update(self, title_banner_msg)

        #  Center the art in the window.
        pad = " " * max(0, int((self.sizeX + 1 - width) / 2))

        logo = "\n"
        for line in self.art:
            logo += pad + line + "\033[0m\n"
        logo += "\n" + pad + self.title_banner_msg + "\n"
        return logo


#===============================================================================
#  __main__
#===============================================================================
if (__name__ == "__main__"):
    print("ansi_supported():", ansi_supported())
    sys.stdout.write(K_AnsiLogo(clear_screen=False,
                                title_banner_msg="Karillon Test Executive").str())
