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
#  Purpose: 
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

DMM1_port = 'com3'
DMM2_port = 'com13'
PS_port = 'com4'
TCB_port = 'com3'
UNO1_port = 'com9'
ARB_ip_addr = '192.168.0.120'
DHO_ip_addr = '192.168.0.240'
PS_ip_addr = '192.168.1.106'
DHO2_ip_addr = '192.168.0.226'
DSO_ip_addr = '192.168.1.240'
DS1_ip_addr = '192.168.0.240'

# Newport FPGA test controller UART (FT230X on the dev kit, 1 Mbaud).
# If this port can't be opened or doesn't answer, the test exec lists the
# serial ports on the machine and asks which one to use.
NEWPORT_port = 'COM3'
