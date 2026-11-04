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
#  Purpose: This test routine is used to communicate with instruments interactively.
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

import logging
import datetime
import time

import winsound

#import pwr_adj

import sys
sys.path.append("G:\\Shared drives\\K_Env\\lib\\python\\K_lib")
sys.path.append("G:\\Shared drives\\K_Env\\lib\\python\\k_instruments")
sys.path
import K_lib
#from K_TCB import K_TCB_v2
#from K_Uno import K_Uno

sys.path.append('./cfg/')
import cfg_com
sys.path.append('./instruments/')
#from PSe3647a import PSe3647a
#from DMM2831e import DMM2831e
from DHO804 import DHO804
from DG1022Z import DG1022Z
from DP832 import DP832

test_title = 'Instrument_debug'

DMM1_port = cfg_com.DMM1_port
DMM2_port = cfg_com.DMM2_port
ARB1_ip_addr = cfg_com.ARB_ip_addr
DHO1_ip_addr = cfg_com.DHO_ip_addr
#PS1_ip_addr = cfg_com.PS_ip_addr
#PS_port = cfg_com.PS_port
#TCB_port = cfg_com.TCB_port
#UNO_port = cfg_com.UNO_port

logging_file=test_title+'.log'
v_pwr = 3
v_anatst = 0
x_muxchan_start = 1
x_muxchan_stop = 7

def writeAll_vpmx(vp, mx, note, data, outf, csvf):
    data = str(data).strip("b")
    data = data.replace("'","")
    logging.info(str(vp)+str(mx)+note+':'+str(data))
    outf.write(str(vp)+','+str(mx)+','+note+','+':'+str(data))
    csvf.write(str(data).strip()+',')
    print(str(vp)+','+str(mx)+','+note+','+':'+str(data))
    
def writeAll_vpmx_eol(vp, mx, note, data, outf, csvf):
    logging.info('\n')
    outf.write('\n')
    csvf.write('\n')
    print('eol')

def instQuery(inst, qStr):
    return inst.iQuery(qStr)
    
def instCommand(inst, cStr):
    return inst.iCommand(cStr)

def test1(serial_num):
    '''
    Instrument Debug
    '''
    
    '''
    ######################################################################################
    # Initialize Instruments and Supplies
    '''
    # Initialize TH TCB
    #UNO1 = K_Uno(UNO_port)
    
    # Initiaize Power Supply    
    #PS1 = DP832(#PS1_ip_addr)
    #PS1.allOff()
    
    # Initialize TH TCB
    #TCB1 = K_TCB_v2(TCB_port)
    
    # Initialize DMM1
    #DMM1 = DMM2831e(DMM1_port)
    
    # Initialize DMM2
    #DMM2 = DMM2831e(DMM2_port)
    
    # Initialize DHO1
    DHO1 = DHO804(DHO1_ip_addr)

    ARB1 = DG1022Z(ARB1_ip_addr)
    print("Serial number="+serial_num)
    if serial_num == "99":
        print("Serial number="+str(serial_num))
        return 1
    
    print("ARB1.query "+ARB1.iQuery(":SOUR1:APPL?"))
    X = "Z"
    while X != "E":
        X = input("Enter QA for query ARB1, QP for query #PS1, QD for query DHO1, CA for command ARB1, E for exit:")
        if X == "QA":
            qStr = input("Enter ARB Query string: ")
            print(instQuery(ARB1, qStr))
        elif X == "QD":
            qStr = input("Enter DHO Query string: ")
            print(instQuery(DHO1, qStr))
        elif X == "QP":
            qStr = input("Enter PS Query string: ")
            print(instQuery(PS1, qStr))
        elif X == "CA":
            cStr = input("Enter ARB Command string: ")
            instCommand(ARB1,cStr)
        elif X == "CD":
            cStr = input("Enter DHO Command string: ")
            instCommand(DHO1,cStr)
        elif X == "CP":
            cStr = input("Enter PS Command string: ")
            instCommand(PS1,cStr)
        else:
            print("Exiting")
    
    return 0