import serial   
import os, time
import adxl345
import smbus
import RPi.GPIO as GPIO
import i2cutils as I2CUtils
from adxl345 import ADXL345
from gy80 import GY80
from quaternions import quaternion_from_euler_angles, quaternion_to_euler_angles

GPIO.setmode(GPIO.BOARD)
#GPIO.setmode(GPIO.BCM)
GPIO_PIN1 = 18 
GPIO.setup(GPIO_PIN1, GPIO.IN)
GPIO_PIN2 = 16 
GPIO.setup(GPIO_PIN2, GPIO.IN)
GPIO_PIN3 = 22 
GPIO.setup(GPIO_PIN3, GPIO.IN)
GPIO_PIN4 = 15 
GPIO.setup(GPIO_PIN4, GPIO.IN)
"""
#LED_Button
GPIO.setup(8, GPIO.OUT)
GPIO.setup(12, GPIO.IN)
led=8
button=12
GPIO.output(led,GPIO.LOW)
##bn= GPIO.input(button)
##print bn
###
"""
mylist=[0,0,0,0,0,0,0,0,0,0]

url="http://thingtalk.ir/update?key=NAQNSYQJ3VM8KJL3"
bus = smbus.SMBus(I2CUtils.i2c_raspberry_pi_bus_number())
adxl345=ADXL345(bus, 0x53, "accel")
adxl345.read_raw_data()
sens1=adxl345.read_scaled_accel_x()
sens2=adxl345.read_scaled_accel_y()
sens3=adxl345.read_scaled_accel_z()

print sens1
print sens2
print sens3
# Enable Serial Communication
port = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=0.2)
rcv = port.read(120)
print rcv



def setup():
	port.write("AT\r\n")
	rcv = port.read(120)
	print rcv
	port.write("AT+CGPSPWR=1\r\n")
	rcv = port.read(120)
	print rcv
	port.write("AT+CREG?\r\n")
	rcv = port.read(120)
	print rcv
	port.write("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n")
	rcv = port.read(120)
	print rcv
	port.write("AT+SAPBR=3,1,\"APN\",\"mcinet\"\r\n")
	time.sleep(2)
	rcv = port.read(120)
	print rcv
	port.write("AT+SAPBR=1,1\r\n")
	rcv = port.read(120)
	print rcv
	port.write("AT+SAPBR=2,1\r\n")
	rcv = port.read(120)
	print rcv
	port.write("AT+HTTPINIT\r\n")
	rcv = port.read(120)
	print rcv




setup()

print("******************")

####front
def acct1(null):
	"""
	#LED_Button
	print("#####")
	print bn
	GPIO.output(led,GPIO.HIGH)
	print("#####")
	###
	"""
	imu=GY80()
	imu.update()
	time.sleep(5)
	#w, x, y, z = imu.current_orientation_quaternion_hybrid()
	w, x, y, z = imu._current_hybrid_orientation_q
	#print("Gyroscope/Accl/Comp q (%0.2f, %0.2f, %0.2f, %0.2f)" % (w, x, y, z))
	yaw, pitch, roll = quaternion_to_euler_angles(w, x, y, z)
	roll= roll* 180.0 /3.14
	pitch= pitch* 180.0 /3.14
	
	print roll
	print pitch
	
	if( -10<roll<10 and 70<pitch<80 ):
		b=2
		print ("khodro be samte rast vazhegon shode")
	elif( -30<roll<0 and -70<pitch<90):
		b=3
		print ("khodro be samte chap vazhegon shode")
	elif( 80<roll<110 and 0<pitch<15 ):
		b=4
		print ("khodro be aghab vazhegoon shode ast")
	elif( -90<roll<-110 and -5<pitch<10 ):
		b=5
		print ("khodro be jelo vazhegoon shode ast")
	else:
		b=1
		print ("tanha zarbe")
	#if( a=1 or b=2 or =b=3 or b=4 )
	#	print("accident occured")
	##GPS Read
	print("$$$$$$")
	port.write("AT+CGPSINF=0"+'\r\n')
	rcv = port.read(120)
	print rcv
	str1=rcv.split(",")
	#print str1[1],str1[2]
	
	## HTTP Post
	sens1=adxl345.read_scaled_accel_x()
	sens2=adxl345.read_scaled_accel_y()
	sens3=adxl345.read_scaled_accel_z()
	#v average
	avr=0
	for i in range(9):	
		avr=avr+(mylist[i])/9
	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field1='+str1[1] +'&field2='+str1[2]+ '&field3='+str(b) +'&field4='+str(a)+'&field4='+str(avr)+ "\"\r\n")
#	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field5='+str(sens2) + "\"\r\n")
#	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field6='+str(sens3) + "\"\r\n")
	rcv = port.read(500)
	print rcv
	port.write("AT+HTTPACTION=0\r\n")
	rcv = port.read(120)
	print rcv
	rcv = port.read(120)
	print rcv
	time.sleep(5)
	rcv = port.write("AT+HTTPTERM\r\n")
	rcv = port.read(500)
	print rcv
	print ("zarbe az jelo")
	time.sleep(5)

###right
def acct2(null):
	"""
	#LED_Button
	print("#####")
	print bn
	GPIO.output(led,GPIO.HIGH)
	print("#####")
	###
	"""
	imu=GY80()
	imu.update()
	#w, x, y, z = imu.current_orientation_quaternion_hybrid()
	w, x, y, z = imu._current_hybrid_orientation_q
	#print("Gyroscope/Accl/Comp q (%0.2f, %0.2f, %0.2f, %0.2f)" % (w, x, y, z))
	yaw, pitch, roll = quaternion_to_euler_angles(w, x, y, z)
	roll= roll* 180.0 /3.14
	pitch= pitch* 180.0 /3.14
	
	if( -10<roll<10 and 70<pitch<80 ):
		b=2
		print ("khodro be samte rast vazhegon shode")
	elif( -30<roll<0 and -70<pitch<90):
		b=3
		print ("khodro be samte chap vazhegon shode")
	elif( 80<roll<110 and 0<pitch<15 ):
		b=4
		print ("khodro be aghab vazhegoon shode ast")
	elif( -90<roll<-110 and -5<pitch<10 ):
		b=5
		print ("khodro be jelo vazhegoon shode ast")
	else:
		b=1
		print ("tanha zarbe")
		
	print roll
	print pitch
	
	#if( a=1 or b=2 or =b=3 or b=4 )
	#	print("accident occured")
	##GPS Read
	print("$$$$$$")
	port.write("AT+CGPSINF=0"+'\r\n')
	rcv = port.read(120)
	print rcv
	str1=rcv.split(",")
	#print str1[1],str1[2]
	
	## HTTP Post
	sens1=adxl345.read_scaled_accel_x()
	sens2=adxl345.read_scaled_accel_y()
	sens3=adxl345.read_scaled_accel_z()
	#v average
	avr=0
	for i in range(9):	
		avr=avr+(mylist[i])/9
	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field1='+str1[1] +'&field2='+str1[2]+ '&field3='+str(b) +'&field4='+str(a)+'&field4='+str(avr)+ "\"\r\n")
#	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field5='+str(sens2) + "\"\r\n")
#	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field6='+str(sens3) + "\"\r\n")
	rcv = port.read(500)
	print rcv
	port.write("AT+HTTPACTION=0\r\n")
	rcv = port.read(120)
	print rcv
	rcv = port.read(120)
	print rcv
	time.sleep(5)
	rcv = port.write("AT+HTTPTERM\r\n")
	rcv = port.read(500)
	print rcv
	print ("zarbe az rast")
	time.sleep(5)
	

#back
def acct3(null):
	"""
	#LED_Button
	print("#####")
	print bn
	GPIO.output(led,GPIO.HIGH)
	print("#####")
	###
	"""
	imu=GY80()
	imu.update()
	#w, x, y, z = imu.current_orientation_quaternion_hybrid()
	w, x, y, z = imu._current_hybrid_orientation_q
	#print("Gyroscope/Accl/Comp q (%0.2f, %0.2f, %0.2f, %0.2f)" % (w, x, y, z))
	yaw, pitch, roll = quaternion_to_euler_angles(w, x, y, z)
	roll= roll* 180.0 /3.14
	pitch= pitch* 180.0 /3.14
	
	if( -10<roll<10 and 70<pitch<80 ):
		b=2
		print ("khodro be samte rast vazhegon shode")
	elif( -30<roll<0 and -70<pitch<90):
		b=3
		print ("khodro be samte chap vazhegon shode")
	elif( 80<roll<110 and 0<pitch<15 ):
		b=4
		print ("khodro be aghab vazhegoon shode ast")
	elif( -90<roll<-110 and -5<pitch<10 ):
		b=5
		print ("khodro be jelo vazhegoon shode ast")
	else:
		b=1
		print ("tanha zarbe")
		
	print roll
	print pitch
	
	#if( a=1 or b=2 or =b=3 or b=4 )
	#	print("accident occured")
	##GPS Read
	print("$$$$$$")
	port.write("AT+CGPSINF=0"+'\r\n')
	rcv = port.read(120)
	print rcv
	str1=rcv.split(",")
	#print str1[1],str1[2]
	
	## HTTP Post
	sens1=adxl345.read_scaled_accel_x()
	sens2=adxl345.read_scaled_accel_y()
	sens3=adxl345.read_scaled_accel_z()
	#v average
	avr=0
	for i in range(9):	
		avr=avr+(mylist[i])/9
	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field1='+str1[1] +'&field2='+str1[2]+ '&field3='+str(b) +'&field4='+str(a)+'&field4='+str(avr)+ "\"\r\n")
#	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field5='+str(sens2) + "\"\r\n")
#	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field6='+str(sens3) + "\"\r\n")
	rcv = port.read(500)
	print rcv
	port.write("AT+HTTPACTION=0\r\n")
	rcv = port.read(120)
	print rcv
	rcv = port.read(120)
	print rcv
	time.sleep(5)
	rcv = port.write("AT+HTTPTERM\r\n")
	rcv = port.read(500)
	print rcv
	print ("zarbe az aghab")
	time.sleep(5)
	
#left
def acct4(null):
	"""
	#LED_Button
	print("#####")
	print bn
	GPIO.output(led,GPIO.HIGH)
	print("#####")
	###
	"""
	imu=GY80()
	imu.update()
	#w, x, y, z = imu.current_orientation_quaternion_hybrid()
	w, x, y, z = imu._current_hybrid_orientation_q
	#print("Gyroscope/Accl/Comp q (%0.2f, %0.2f, %0.2f, %0.2f)" % (w, x, y, z))
	yaw, pitch, roll = quaternion_to_euler_angles(w, x, y, z)
	roll= roll* 180.0 /3.14
	pitch= pitch* 180.0 /3.14
	
	if( -10<roll<10 and 70<pitch<80 ):
		b=2
		print ("khodro be samte rast vazhegon shode")
	elif( -30<roll<0 and -70<pitch<90):
		b=3
		print ("khodro be samte chap vazhegon shode")
	elif( 80<roll<110 and 0<pitch<15 ):
		b=4
		print ("khodro be aghab vazhegoon shode ast")
	elif( -90<roll<-110 and -5<pitch<10 ):
		b=5
		print ("khodro be jelo vazhegoon shode ast")
	else:
		b=1
		print ("tanha zarbe")
		
	print roll
	print pitch
	
	#if( a=1 or b=2 or =b=3 or b=4 )
	#	print("accident occured")
	##GPS Read
	print("$$$$$$")
	port.write("AT+CGPSINF=0"+'\r\n')
	rcv = port.read(120)
	print rcv
	str1=rcv.split(",")
	#print str1[1],str1[2]
	
	## HTTP Post
	sens1=adxl345.read_scaled_accel_x()
	sens2=adxl345.read_scaled_accel_y()
	sens3=adxl345.read_scaled_accel_z()
	#v average
	avr=0
	for i in range(9):	
		avr=avr+(mylist[i])/9
	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field1='+str1[1] +'&field2='+str1[2]+ '&field3='+str(b) +'&field4='+str(a)+'&field4='+str(avr)+ "\"\r\n")
#	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field5='+str(sens2) + "\"\r\n")
#	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field6='+str(sens3) + "\"\r\n")
	rcv = port.read(500)
	print rcv
	port.write("AT+HTTPACTION=0\r\n")
	rcv = port.read(120)
	print rcv
	rcv = port.read(120)
	print rcv
	time.sleep(5)
	rcv = port.write("AT+HTTPTERM\r\n")
	rcv = port.read(500)
	print rcv
	print ("zarbe az chap")
	time.sleep(5)

	
GPIO.add_event_detect(GPIO_PIN1, GPIO.FALLING, callback=acct1, bouncetime=100) 
GPIO.add_event_detect(GPIO_PIN2, GPIO.FALLING, callback=acct2, bouncetime=100)
GPIO.add_event_detect(GPIO_PIN3, GPIO.FALLING, callback=acct3, bouncetime=100)
GPIO.add_event_detect(GPIO_PIN4, GPIO.FALLING, callback=acct4, bouncetime=100)


try:
	
	print("######")
	for i in range(8):	
		time.sleep(1)
		a=adxl345.read_scaled_accel_x()
	#	print("hbh"+str(a))
		mylist[i+1]=a*1+mylist[i]
	while True:
##		if bn==True:
##			GPIO.output(led,GPIO.LOW)
		#....
		for i in range(8):
			mylist[i]=mylist[i+1]
		time.sleep(1)
		a=adxl345.read_scaled_accel_x()
		mylist[9]=a*1+mylist[8]	
		#for i in range(9):
	#		print mylist[i]	
	
				
except KeyboardInterrupt:
	GPIO.cleanup()


"""
while True:

	sens4=GPIO.input(18)
	print sens4
	#time.sleep(5)
"""	


""""	
while True:
	##GPS Read
	port.write("AT+CGPSINF=0"+'\r\n')
	rcv = port.read(120)
	print rcv
	## HTTP Post
	sens1=adxl345.read_scaled_accel_x()
	sens2=adxl345.read_scaled_accel_y()
	sens3=adxl345.read_scaled_accel_z()
#	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field4='+str(sens1) +'&field5='+str(sens2)+ '&field6='+str(sens3) + "\"\r\n")
#	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field5='+str(sens2) + "\"\r\n")
#	port.write("AT+HTTPPARA=\"URL\", \"" + url +'&field6='+str(sens3) + "\"\r\n")
	rcv = port.read(500)
	print rcv
	port.write("AT+HTTPACTION=0\r\n")
	rcv = port.read(120)
	print rcv
	rcv = port.read(120)
	print rcv
	time.sleep(5)


"""

