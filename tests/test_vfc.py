import logging
import datetime
import time
import cfg_com
import winsound

import acm_globals
import pwr_adj

import sys
sys.path.append('./lib/')
import th_lib
from TH_TCB import TH_TCB_v2
from TH_Uno import TH_Uno

sys.path.append('./instruments/')
from PSe3647a import PSe3647a
from DMM2831e import DMM2831e

test_title = 'VFC_Test'
serial_num = 0;
temperature = 27;

DMM1_port = cfg_com.DMM1_port
DMM2_port = cfg_com.DMM2_port
PS_port = cfg_com.PS_port
TCB_port = cfg_com.TCB_port
UNO_port = cfg_com.UNO_port

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

def test1(serial_num):
    '''
    VFC testing
    Measure (ipwr), vbus, vldo, vdc1, vdc2, vbg/ibias, (iload)
    For vpwr = 2V to 5V
    '''
    
    '''
    ######################################################################################
    # Initialize log and output files
    '''
    
    # Get serial number and temperature from user
    test_name = test_title+'_SN='+str(serial_num)+'_T='+str(th_lib.getTemperature())
    
    logging.basicConfig(filename=logging_file, level=logging.DEBUG)
    print('------------------------------Start '+test_name+'------------------------------')
    startTime = datetime.datetime.now()
    startTimeFname = startTime.strftime('%Y_%m_%d_%H_%M_%S')
    startTimeHuman = startTime.strftime('%Y/%m/%d %H:%M:%S')
    print(startTimeHuman)
    outFileName = './logs/'+startTimeFname+'_'+test_name
    
    csvFileName = './logs/'+startTimeFname+'_'+test_name+'.csv'
    
    try:
        csvfile = open(csvFileName, 'w')
    except:
        print('Unable to open '+csvFileName)
        return 1
    else:
        csvfile.write('------------------------------Start '+test_name+'-------------------------------'+startTimeHuman+'\n')
        csvfile.write('vset, vin, vsrc, vpwr, vbus, vldo, vfc, ipwr\n')
    
    try:
        outfile = open(outFileName, 'w')
    except:
        print('Unable to open '+outFileName)
        return 1
    else:
        outfile.write('------------------------------Start '+test_name+'-------------------------------'+startTimeHuman+'\n')
        
    logging.info('----------------------------Start '+test_name+'----------------------------')
    logging.info(startTimeHuman)
    
    '''
    ######################################################################################
    # Initialize Instruments and Supplies
    '''
    # Initialize TH TCB
    UNO1 = TH_Uno(UNO_port)
    
    # Initiaize Power Supply    
    PS1 = PSe3647a(PS_port)
    PS1.setOutVolts(chan=1, volts=v_pwr)
    PS1.setOutVolts(chan=2, volts=v_anatst)
    PS1.onPS(True)
    
    # Initialize TH TCB
    TCB1 = TH_TCB_v2(TCB_port)
    
    # Initialize DMM
    DMM1 = DMM2831e(DMM1_port)
    
    # Initialize DMM
    DMM2 = DMM2831e(DMM2_port)
    
    
    '''
    ######################################################################################
    # Run the test sequence
    # 
    # VFC Test
    # Sweep Vpwr.
    # Measure and record voltages on mux channels.
    #
    '''
    muxList = [1, 2, 3, 4]
    # ANATST on LED1
    tcData = '0x87'
    PS1.setOutVolts(chan=1, volts=3)
    TCB1.i2cWrite(acm_globals.BSICA_reg['BSICA_TESTCTRL'], tcData)
    time.sleep(0.5)
    TCB1.i2cWrite(acm_globals.BSICA_reg['BSICA_VFC_INTERFACE'], '0x07')
    winsound.Beep(3000, 500)
    input('Connect PS1 Out2 to LED1. Press any key to continue:')
    
    for vp in range(30, 51, 5):
        vset = vp/10
        PS1.setOutVolts(chan=1, volts=vset)
        for vin in range(29, vp, 5):
            vfcIn = vin/10
            PS1.setOutVolts(chan=2, volts=vfcIn)
            csvfile.write(str(vset)+','+str(vfcIn)+',')
            for muxch in muxList:
                UNO1.sendMux(muxch)
                time.sleep(0.5)
                writeAll_vpmx(vset, muxch, '', DMM1.getVolt(), outfile, csvfile) 
            
            # vfcr
            vfc = TCB1.sendData('$VFCR\r')
            time.sleep(0.5)
            vfc = TCB1.readStuff()
            vfc = str(vfc).strip("b")
            vfc = vfc.replace("'","")
            print('0x'+vfc[7:12])
            writeAll_vpmx(vset, muxch, '', '0x'+vfc[7:12], outfile, csvfile)
            writeAll_vpmx(vp, muxch, tcData, DMM2.getCurr(), outfile, csvfile)
            writeAll_vpmx_eol(vset, muxch, '', 0, outfile, csvfile)
    

    winsound.Beep(3000, 500)
    input('Disconnect PS1 Out2 to LED1. Press any key to continue:')    
    tcData = '0x00'
    PS1.setOutVolts(chan=1, volts=3)
    TCB1.i2cWriteVerify(acm_globals.BSICA_reg['BSICA_TESTCTRL'], tcData)
    time.sleep(0.5)
    TCB1.i2cWriteVerify(acm_globals.BSICA_reg['BSICA_VFC_INTERFACE'], '0x0D')
    
    
    '''
    ######################################################################################
    # Shut everything down, close ports, files, etc.
    '''
    PS1.onPS(False)
    
    PS1.closePort()
    DMM2.closePort()
    DMM1.closePort()
    TCB1.closePort()
    UNO1.closePort()
    
    stopTimeHuman = datetime.datetime.now().strftime('%Y/%m/%d %H:%M:%S')
    print('-------------------------Stop '+test_name+'--------------------------------')
    print(stopTimeHuman)
    outfile.write('------------------------------Stop '+test_name+'-------------------------------'+stopTimeHuman+'\n')
    outfile.close()
    csvfile.write('------------------------------Stop '+test_name+'-------------------------------'+stopTimeHuman+'\n')
    csvfile.close()
    logging.info('-------------------------Stop '+test_name+'--------------------------------')
    logging.info(stopTimeHuman)
    logging.info('===================================================================')
    
    return 0