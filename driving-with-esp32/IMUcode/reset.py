import serial
ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
reset_message = 'RESET\n'
ser.write(reset_message.encode('utf-8'))
