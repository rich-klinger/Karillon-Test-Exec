import serial
import logging
import time

class DMM2831e:
    """
    This class provides basic functionality to operate with a BK Precision multimeter 2831E.
    """

    baud = 38400
    ser = None
    time_resolution = 0.008
    dmmid = None
    
    def __init__(self, serial_port=None, time_resolution=0.008, logging_file='bk_2831e.log'):
        """
        Initialize the serial port in which the multimeter is connected.
        :param serial_port: is the ID (Windows) or dev resource (Linux) which the multimeter is connected.
        :param time_resolution: is the time between measures.
        :param logging_file: optional, is the file which the log messages will be stored.
        """
        if serial_port is not None:
            self.ser = serial.Serial(port=serial_port,
                                     baudrate=self.baud,
                                     bytesize=serial.EIGHTBITS,
                                     #timeout=0,
                                     parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE)
        
        #logging.basicConfig(filename=logging_file, level=logging.DEBUG)
        #self.configure_connection()
            
        self.setupDMM()
        self.time_resolution = time_resolution
        
    def configure_connection(self):
        """
        Configure the connection.
        :return: True if connection and configuration is successful: otherwise, return False.
        """
        ''' RK: Didn't work as well as setupDMM.
        if self.ser.is_open:
            logging.info('The serial port is opened.')
        else:
            logging.error('ERROR: The serial port could not been opened! Check serial port configuration.')
            return False
        self.ser.write(b'*IDN?\r')
        time.sleep(0.1)
        out = self.ser.read(100)
        logging.info('BKPrecision 2831E: (%d) %s' % (len(out), out))
        self.ser.flush()
        self.ser.write(b':FUNC?\r')
        time.sleep(0.1)
        out = self.ser.read(100)
        logging.info('BKPrecision 2831E: (%d) %s' % (len(out), out))
        self.ser.flush()
        return True
        '''

    def measure(self):
        """
        Query a measure command to the multimeter.
        :return: a float value representing the response from the multimeter at a given time_resolution. None if
                 conversion could not been completed.
        """
        ''' RK: Didn't work as well as getVolt and GetCurr.
        self.ser.write(b':FETC?\r')
        time.sleep(self.time_resolution)
        out = self.ser.read(20)
        self.ser.flush()
        if out is not None and out != '':
            try:
                return float(out)
            except ValueError:
                return None  
        '''

    def closePort(self):
        self.ser.close()

    def sendData(self, data):
        for char in data:
            self.ser.write(char.encode())

    def readData(self):
        buffer = ""
        while True:
            oneByte = self.ser.read(1)
            if oneByte == b"\r":    #method should returns bytes
                return buffer
            else:
                buffer += oneByte.decode("ascii")
    
    def setupDMM(self):
        """
        Setup the DMM
        :return: True if connection and configuration is successful: otherwise, return False.
        
        # Please set the multimeter to use new "LF" aka. "\n"
        """
        print('DMM Setup')
        
        if self.ser.is_open:
            logging.info('The DMM serial port name='+str(self.ser.name)+' is opened.')
        else:
            logging.error('ERROR: The DMM serial port '+str(serial_port)+' could not been opened! Check serial port configuration.')
            return False
        
        self.sendData('*idn?\n')
        dmmid = self.ser.readline().strip()
        print('ID = ', dmmid)
        logging.info('DMM ID is '+str(dmmid))
        
        print('Resetting')
        self.sendData('*rst\n')
        time.sleep(3)
        logging.info('DMM Reset')
        self.ser.flush()

    def getVolt(self):
        self.sendData(':function voltage:dc\n')
        #Preceeding commands need time to finish, so we wait for a little while.
        time.sleep(1)
        self.sendData(':fetch?\n')
        volt = self.ser.readline().strip()
        self.ser.flush()
        return volt
        
    def getCurr(self):
        self.sendData(':function current:dc\n')
        #Preceeding commands need time to finish, so we wait for a little while.
        time.sleep(1)
        self.sendData(':fetch?\n')
        curr = self.ser.readline().strip()
        self.ser.flush()
        return curr
        
    def getFreq(self):
        self.sendData(':function frequency\n')
        self.sendData(':frequency:threshold:voltage:range 1.5\n')
        #Preceeding commands need time to finish, so we wait for a little while.
        time.sleep(1)
        self.sendData(':fetch?\n')
        freq = self.ser.readline().strip()
        self.ser.flush()
        return freq
