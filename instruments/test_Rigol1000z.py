import visa
import time

import sys
sys.path.append('./rigol1000z/')
from rigol1000z import Rigol1000z

#from rigol_ds1054z import rigol_ds1054z

rm = visa.ResourceManager()

# We are connecting the oscilloscope through USB here.
# Only one VISA-compatible instrument is connected to our computer,
# thus the first resource on the list is our oscilloscope.
# You can see all connected and available local devices calling
#
print(rm.list_resources())
#
osc_resource = rm.open_resource(rm.list_resources()[1])
print(osc_resource)

osc = Rigol1000z(osc_resource)
#osc = rigol_ds1054z()

print(osc.get_mode())
print(osc.get_sampling_rate())
osc.set_normal_mode()
'''
# Change voltage range of channel 1 to 50mV/div.
osc[1].set_vertical_scale_V_div(50e-3)
'''
# Stop the scope.
osc.stop()
time.sleep(1)
osc.run()

'''
# Take a screenshot.
osc.get_screenshot('screenshot.png', 'png')

# Capture the data sets from channels 1--4 and
# write the data sets to their own file.
for c in range(1,5):
    osc[c].get_data('raw', 'channel%i.dat' % c)
'''