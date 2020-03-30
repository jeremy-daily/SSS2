import usb.core
import usb.util
import struct
import crc16
import traceback
import time

# must have usblib installed.

USB_HID_OUTPUT_ENDPOINT_ADDRESS = 0x04
USB_HID_INPUT_ENDPOINT_ADDRESS = 0x83
USB_HID_LENGTH = 64
USB_HID_TIMEOUT = 0 # 0 = Blocking


def crc16_ccitt(crc, data):
    msb = (crc & 0xFF00) >> 8
    lsb = crc & 0xFF
    for c in data:
        x = c ^ msb
        x ^= (x >> 4)
        msb = (lsb ^ (x >> 3) ^ (x << 4)) & 255
        lsb = (x ^ (x << 5)) & 255
    return bytes([lsb, msb])

# find our device
sss = usb.core.find(idVendor=0x16c0, idProduct=0x0486)

# was it found?
if sss is None:
    raise ValueError('Device not found')
print(sss)
sss.set_configuration()
# get an endpoint instance
cfg = sss.get_active_configuration()
sss_interface = cfg[(0,0)]


data = b'\x10,1,32,2,33,3,34,4,42,17,42,50,1' 
padded_data = data + bytes([0 for i in range(62 - len(data))])
crc = crc16_ccitt(0xFFFF, bytes(padded_data))
print(crc)
data_to_send = bytes(padded_data) + crc
print(data_to_send)
print(len(data_to_send))
sss.write(USB_HID_OUTPUT_ENDPOINT_ADDRESS, data_to_send, USB_HID_TIMEOUT)
for i in range(20):
	# read method returns an array of bytes 
	# see (https://docs.python.org/3/library/array.html)
	try:
		data_stream = bytes(sss.read(USB_HID_INPUT_ENDPOINT_ADDRESS, 
								 USB_HID_LENGTH, 
								 300))
		data_stream_crc = data_stream[62:64]
		calculated_crc = crc16_ccitt(0xFFFF, data_stream[0:62])
		print(data_stream)
		#print(calculated_crc)
		assert data_stream_crc == calculated_crc
		
	except usb.core.USBError:
		time.sleep(0.002)
	except:
		print(traceback.format_exc())

 # The data payload can be any sequence type that can be used 
 # as a parameter for the array __init__ method. 
data = b'\x10,50,0,b1, ,B0' 
padded_data = data + bytes([0 for i in range(62 - len(data))])
crc = crc16_ccitt(0xFFFF, bytes(padded_data))
print(crc)
data_to_send = bytes(padded_data) + crc
print(data_to_send)
print(len(data_to_send))
sss.write(USB_HID_OUTPUT_ENDPOINT_ADDRESS, data_to_send, USB_HID_TIMEOUT)
usb.util.release_interface(sss,sss_interface)