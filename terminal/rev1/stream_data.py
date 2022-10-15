import serial
import time

# Open the serial port
serObj = serial.Serial()
serObj.baudrate = 9600
serObj.port = "/dev/ttyUSB0"
serObj.timeout = 1
serObj.open()

# Start the timer
serObj.write( b'\x00')

while True:

	# Request Data
	serObj.write( b'\x00')

	# Read 4 bytes
	time_ticks = 0
	for i in range( 4 ):
		time_byte = serObj.read() 	
		time_ticks += ( int.from_bytes(time_byte, 'big') << 8*(i) )
	
	# Print to screen
	time = time_ticks/1000.0
	print( time )

