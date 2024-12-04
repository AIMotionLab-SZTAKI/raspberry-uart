import serial

ser = serial.Serial('/dev/ttyAMA0', 115200)

#ser.write(b"hello")

a = ser.read(1)

print(a)