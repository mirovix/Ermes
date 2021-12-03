import serial
import time
ser = serial.Serial('/dev/ttyACM0', 57600, timeout=0) 
a = 0
f = open('test.txt','w') 
while a<10000:
   data=ser.readline()
   data = data[0:len(data)-2].decode("utf-8")
   print(data)
   f.write(data)
   a+=1
   time.sleep(1) 
f.close()