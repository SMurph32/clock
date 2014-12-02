#----------
# import the PyUSB module
import d2xx

# list devices by description, returns tuple of attached devices description strings
d = d2xx.listDevices(d2xx.OPEN_BY_DESCRIPTION)
print d

# list devices by serial, returns tuple of attached devices serial strings
d = d2xx.listDevices() # implicit d2xx.OPEN_BY_SERIAL_NUMBER
print d

h = d2xx.open(0)
print h

# read eeprom
print h.eeRead()

# get queue status
print h.getQueueStatus()

# set RX/TX timeouts
h.setTimeouts(1000,1000)

# write bytes (serial mode)
print h.write("Hello world!\r\n")

# read bytes (serial mode)
print h.read(5)

# ----------
