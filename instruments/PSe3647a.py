import serial
import logging
import time

class PSe3647a:
    """
    This class provides basic functionality to operate with an Agilent E3647A Dual Output DC Power Supply.
    """
    
    baud = 9600
    ser = None
    psid = None
    
    def __init__(self, serial_port=None, logging_file='ag_e3647a.log'):
        """
        Initialize the serial port in which the multimeter is connected.
        :param serial_port: is the ID (Windows) or dev resource (Linux) which the multimeter is connected.
        :param time_resolution: is the time between measures.
        :param logging_file: optional, is the file which the log messages will be stored.
        """
        if serial_port is not None:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=self.baud,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
        
        #logging.basicConfig(filename=logging_file, level=logging.DEBUG)
        
        self.setupPS(0, 0, False)
        
    def closePort(self):
        self.ser.close()

    def sendData(self, data):
        for char in data:
            self.ser.write(char.encode())
        
    def onPS(self, on):
        if on:
            self.sendData('outp on\n')
            print('Power Supply Output ON')
            logging.info('Power Supply '+str(self.psid)+' Output=ON')
        else:
            self.sendData('outp off\n')
            print('Power Supply Output OFF')
            logging.info('Power Supply '+str(self.psid)+' Output=OFF')

    def setupPS(self, out1, out2, on):
        """
        Setup Power Supply.
        :return: True if connection and configuration is successful: otherwise, return False.
        """
        # Please set the multimeter to use new "LF" aka. "\n"
        
        print('Power Supply Setup')
        
        if self.ser.is_open:
            logging.info('The power supply serial port name='+str(self.ser.name)+' is opened.')
        else:
            logging.error('ERROR: The power supply serial port '+str(serial_port)+' could not been opened! Check serial port configuration.')
            return False
        
        self.sendData('*idn?\n')
        self.psid = self.ser.readline().strip()
        print("ID = ", self.psid)
        logging.info('Power Supply ID is '+str(self.psid))
        
        self.sendData('inst:sel outp1\n')
        mystring = 'apply '+str(out1)+'\n'
        self.sendData(mystring)
        print('Out1 = '+str(out1)+'V')
        logging.info('Power Supply Output1 setpoint = '+str(out1)+'V')
        
        self.sendData('inst:sel outp2\n')
        mystring = 'apply '+str(out2)+'\n'
        self.sendData(mystring)
        print('Out2 = '+str(out2)+'V')
        logging.info('Power Supply Output1 setpoint = '+str(out2)+'V')

        self.onPS(on)

    def setOutVolts(self, chan, volts):
        writeString = 'inst:sel outp'+str(chan)+'\n'
        self.sendData(writeString)
        writeString = 'apply '+str(volts)+'\n'
        self.sendData(writeString)
        print('Out'+str(chan)+' = '+str(volts)+'V')
'''
    def setCurrentLimit(self, chan, curr):
        writeString = 'inst:sel outp'+str(chan)+'\n'
        self.ser.write(writeString.encode())
        writeString = 'apply '+str(volts)+'\n'
        self.ser.write(writeString.encode())
        print('Out'+str(chan)+' = '+str(volts)+'V')
'''   