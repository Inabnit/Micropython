#Configuration file

#wifi settings
ssid = 'Inabnit'
password = 'Buochserhorn'


#mqtt settings
mqtt_server = '192.168.50.31'

topic_pub_temp1 = b'esp/ds18b20/temperature1' #topic name, add more if needed
topic_pub_temp2 = b'esp/ds18b20/temperature2' #topic name, add more if needed
topic_pub_output1 = b'esp/mosfet/output1' #topic name, add more if needed
topic_pub_output2 = b'esp/mosfet/output2' #topic name, add more if needed

#temperature settings
tempsetpoint = 30.0   # temperature set point
maxtemperaturedifference = 10


#PID Settings

# The Tweakable values that will help tune for our use case. TODO: Make accessible via menu on OLED
# Explanation Stolen From Reddit: In terms of steering a ship:
# Kp is steering harder the further off course you are,
# Ki is steering into the wind to counteract a drift
# Kd is slowing the turn as you approach your course
Kp=100.   # 400 Proportional term - Basic steering (This is the first parameter you should tune for a particular setup)
Ki=.01   # .05 Integral term - Compensate for heat loss by vessel
Kd=0.  # Derivative term - to prevent overshoot due to inertia - if it is zooming towards setpoint this
         # will cancel out the proportional term due to the large negative gradient


#datalogging

datalogging = False

#DS18b20 addresses

#roms = ds_sensor.scan()
#print('Found DS devices: ', roms)
Sensor1 = b"('/\x07\x00\x00\x00<"
Sensor2 = b'\x10Fd\x16\x03\x08\x00\x91'
Sensor3 = b'\x10Fd\x16\x03\x08\x00\x80' #update rom with actual address


