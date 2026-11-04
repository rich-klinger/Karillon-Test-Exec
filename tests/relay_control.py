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
#  Purpose: This test routine conducts a sequence of power on/off/on cycles 
#           and captures waveform data on a pair of Rigol DHP804.
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
import struct
import numpy as np

from os import listdir
from os import makedirs
from os import path

from math import trunc
from progress.bar import Bar
import winsound

import sys
sys.path.append("C:\\python\\k_lib")
sys.path
import K_lib
from K_MinimaR4 import K_MinimaR4

sys.path.append('./cfg/')
import cfg_com
sys.path.append('./instruments/')
from DMM2831e import DMM2831e
from DHO804 import DHO804
from DS7014 import DS7014
from DG1022Z import DG1022Z

from test_power_cycle_seq1_dho_setup_v6 import scope_setup_dho1
from test_power_cycle_seq1_dho_setup_v6 import scope_setup_dho2

test_title = 'Power_Cycle_Sequence_1'

DMM1_port = cfg_com.DMM1_port
ARB1_ip_addr = cfg_com.ARB_ip_addr
DHO1_ip_addr = cfg_com.DHO_ip_addr
#DHO2_ip_addr = cfg_com.DSO_ip_addr
DHO2_ip_addr = cfg_com.DHO2_ip_addr

logging_file=test_title+'.log'

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
    
def getlastSet(filePath):
    listDir = listdir(filePath)
    lastDir = listDir[-1]
    setNum = lastDir.split()
    return setNum[-1]

def test1(serial_num):
    '''
    Power Cycle testing
    Measure VIN, VOUT
    '''
    
    '''
    ######################################################################################
    # Initialize log and output files
    '''
    
    
    '''
    ######################################################################################
    # Initialize Instruments and Supplies
    '''
    UNO1_port = 'com9'
    UNO1 = K_MinimaR4(UNO1_port)
    print("ID = "+UNO1.readID()+"\n") 
    input("press enter")
    UNO1.setMOSFET(2, 'false') # 0x4000 = 0101 0100 0000 0000 0x5400 = 0101 0100 0000 0000
    #input("press enter")
    
    input("press enter")
    UNO1.setMOSFET(2, 'true')
    
    input("press enter")
    UNO1.setMOSFET(3, 'true')  # 0x0400 = 0101 0100 0000 0000 0x5400 = 0101 0100 0000 0000
    #input("press enter")
    
    input("press enter")
    UNO1.setMOSFET(3, 'false')
    
    input("press enter")
    UNO1.setMOSFET(4, 'true') #      0x4c00 = 0100 1100 0000 0000
    
    input("press enter")
    UNO1.setMOSFET(4, 'false')
    
    '''
    UNO1.setRelay(1, 'true') # 0x01 = 0000 0001 #0x80 = 1000 0000
    time.sleep(0.5)
    input("press enter")
    
    go = 0
    while go == 0:
        UNO1.setRelay(1, 'false') # 0x80 = 1000 0000 #0x01 = 0000 0001
        time.sleep(0.5)
        go = int(input("enter 0 to continue"))
    
    #UNO1.setRelay(1, 'true')
    UNO1.setRelay(2, 'false') # 0x20 0010 0000 #0x04 = 0000 0100
    time.sleep(0.5)
    input("press enter")
    
    UNO1.setRelay(2, 'true') # 0x40 0100 0000 #0x02 = 0000 0010
    time.sleep(0.5)
    input("press enter")
    
    #UNO1.setRelay(2, 'false')
    UNO1.setRelay(3, 'true') #  0x02 0000 0010 #0x40 = 0100 0000
    time.sleep(0.5)
    input("press enter")
    
    UNO1.setRelay(3, 'false') #  0x04 0000 0100 #0x20 = 0010 0000
    time.sleep(0.5)
    input("press enter")
    
    #UNO1.setRelay(3, 'true')
    UNO1.setRelay(4, 'true') # 0x08 0000 1000 #0x10 = 0001 0000
    time.sleep(0.5)
    input("press enter")
    
    UNO1.setRelay(4, 'false') #  0x10 0001 0000 #0x08 = 0000 1000
    time.sleep(0.5)
    input("press enter")
    
    UNO1.setRelay(4, 'true') # 0x08 0000 1000 #0x10 = 0001 0000
    input("press enter")
    '''
    #detachSquibEmuInt()
      
    
    return 0