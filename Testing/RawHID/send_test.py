import pywinusb.hid as hid
import time

def sample_handler(data):
    print("Raw data: {0}".format(data))

filter = hid.HidDeviceFilter(vendor_id = 0x16c0, product_id = 0x0486)
hid_device = filter.get_devices()
print("hid_device:")
print(hid_device)

device = hid_device[0]
print("device:")
print(device)

device.open()

out_report = device.find_output_reports()
print("out_report")
print(out_report)

target_usage = hid.get_full_usage_id(0x00, 0x3f)
device.set_raw_data_handler(sample_handler)
print("target_usage:")
print(target_usage)
#device.close()

data = [0xFF]*64


for i in range(256):
    #device.open()
    #out_report = device.find_output_reports()
    #print(i)
    #try:
    data[0] = i
    buf = [0] + data
    out_report[0].set_raw_data(buf)
    out_report[0].send()
    print("Success with {}".format(i))
    time.sleep(.001)
device.close()
    #except:
    #    pass
