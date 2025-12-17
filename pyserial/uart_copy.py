import serial
import time
 
ser = serial.Serial('/dev/ttyUSB0', 115200)
 
with open("blinky.bin", "rb") as f:
    data = f.read()
 
size = len(data)
print("Sending", size, "bytes")
 
ser.write(size.to_bytes(4, 'little'))
print("Sending", size, "bytes")
time.sleep(0.1)
ser.write(data)
 
ser.close()
