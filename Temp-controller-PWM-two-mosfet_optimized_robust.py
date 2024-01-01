
from machine import Pin, SoftI2C
import ssd1306
import machine
import utime
import onewire, ds18x20, time
from umqtt.robust import MQTTClient
import ubinascii
from machine import RTC
import os
import micropython
import network
import esp
esp.osdebug(None)
import gc
gc.collect()


###################### define global variables##########################
tempsetpoint = 30.0   # temperature set point
last_message = 0
message_interval = 5
integral = 0
lastupdate = utime.time()  
lasterror = 0
output1=0
output2=0

maxtemperaturedifference = 10

datalogging = False

# The Tweakable values that will help tune for our use case. TODO: Make accessible via menu on OLED
checkin = 0
# Explanation Stolen From Reddit: In terms of steering a ship:
# Kp is steering harder the further off course you are,
# Ki is steering into the wind to counteract a drift
# Kd is slowing the turn as you approach your course
Kp=100.   # 400 Proportional term - Basic steering (This is the first parameter you should tune for a particular setup)
Ki=.01   # .05 Integral term - Compensate for heat loss by vessel
Kd=0.  # Derivative term - to prevent overshoot due to inertia - if it is zooming towards setpoint this
         # will cancel out the proportional term due to the large negative gradient
output1=0
output2=0

maxtemperaturedifference = 10

################### Connect to WIFI and MQTT server ##################################

ssid = 'Inabnit'
password = 'Buochserhorn'
mqtt_server = '192.168.50.31'
"""
station = network.WLAN(network.STA_IF)

station.active(True)
station.connect(ssid, password)

while station.isconnected() == False:
  pass

print('Connection to WIFI successful')
"""
class wifi:
  def __init__(self, ssid, password):
    self.ssid = ssid
    self.password = password

  def connect(self):
    station = network.WLAN(network.STA_IF)
    station.active(True)
    station.connect(self.ssid, self.password)
    while station.isconnected() == False:
      pass

    print("Connection to %s successful" %self.ssid)

credentials = wifi("Inabnit", "Buochserhorn")
credentials.connect()


    ################### MQTT Setup ##################################
client_id = ubinascii.hexlify(machine.unique_id())

topic_pub_temp1 = b'esp/ds18b20/temperature1' #topic name, add more if needed
topic_pub_temp2 = b'esp/ds18b20/temperature2' #topic name, add more if needed
topic_pub_output1 = b'esp/mosfet/output1' #topic name, add more if needed
topic_pub_output2 = b'esp/mosfet/output2' #topic name, add more if needed



    ################### RTC setup ##################################

rtc=RTC()
timeStamp=0

    ################### Mosfet setup ##################################

 #define mosfet1 pin
p0 = machine.Pin(0)
pwm0 = machine.PWM(p0)
pwm0.freq(500)
pwm0.duty_u16(0)

 #define mosfet2 pin
p0 = machine.Pin(1)
pwm1 = machine.PWM(p0)
pwm1.freq(500)
pwm1.duty_u16(0)

    ################### DS18b20 setup ##################################

#define temp sensor pin
ds_pin = machine.Pin(21)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))


 
#roms = ds_sensor.scan()
#print('Found DS devices: ', roms)
Sensor1 = b"('/\x07\x00\x00\x00<"
Sensor2 = b'\x10Fd\x16\x03\x08\x00\x91'
Sensor3 = b'\x10Fd\x16\x03\x08\x00\x80' #update rom with actual address


     ################### OLED Setup ##################################

#setup oled
WIDTH  = 128                                            # oled display width
HEIGHT = 64                                             # oled display height
 
i2c = SoftI2C(scl=Pin(8), sda=Pin(9))
 
oled_width = 128
oled_height = 64
oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)

    ################### Functions ##################################



######################## Logging function#########################
with open('savedata.txt', 'a') as f:
    f.write('Temperature Data Logging')
    f.write('\n')
    
def logData():
    timeTuple=rtc.datetime()
    file_size=os.stat('/savedata.txt')
    if(file_size[6]<2000000):
        try:
            with open('savedata.txt', 'a') as f:
                f.write(str(timeTuple[4])+':'+str(timeTuple[5])+':'+str(timeTuple[6])+ ',')
                f.write(str(temp)+ ',')
                f.write(str(output1))
                #f.write(scaled_output)
                f.write('\n')
                print("Data Saved!")
        
        except:
                print("Error! Could not save")
 
################scaling function to scale X-XXX to X-XXX exapmple: scale_value(output, 0, 100, 0, 65535)#############
def scale_value(value, in_min, in_max, out_min, out_max):
  scaled_value = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
  return int(scaled_value)

################################
def checktemperature():
    global temp_max
    global temp1
    global temp2
    global tempdifference
    #global temp3 #uncomment if 3 sensors will be used
    ds_sensor.convert_temp()
    temp1 = round(ds_sensor.read_temp(Sensor1),1)
    temp2 = round(ds_sensor.read_temp(Sensor2),1)
    #temp3 = round(ds_sensor.read_temp(Sensor3),1) #uncomment if 3 sensors will be used

    
    temp_max = round(max(temp1, temp2),1)
    #temp_max = round(max(temp1, temp2, temp3),1) #uncomment if 3 sensors will be used
    
    #print(temp_max)
    tempdifference = abs(temp1-temp2)
  
################################ 
def displayonOLED():
     #display data on the oled    
    # Clear the oled display in case it has junk on it.
    oled.fill(0)
    oled.contrast(255)  # bright
    # Add some text
    #oled.text("Temperature ",12,5)
    oled.text("Setpoint: ",1,5)
    oled.text(str(tempsetpoint),88,5)
    oled.text("Sensor 1: ",1,15)
    oled.text(str(temp1),88,15)
    oled.text("Sensor 2: ",1,25)
    oled.text(str(temp2),88,25)
    oled.text("Sensor Max: ",1,35)
    oled.text(str(temp_max),88,35)
    oled.text("Output 1: ",1,45)
    oled.text(str(output1),88,45)
    # Finally update the oled display so the image & text is displayed
    oled.show()
###################################    
def displayonOLED_tempdiff():
     #display data on the oled    
    # Clear the oled display in case it has junk on it.
    oled.fill(0)
    """### Blink OLED###
    oled.show()
    utime.sleep_ms(200)
    oled.fill(1)
    oled.show()
    utime.sleep_ms(200)
    """
    oled.fill(0)

    oled.contrast(255)  # bright
    # Add some text

    oled.text("Temp diff.> 10",1,5)
    oled.text("Sensor 1: ",1,15)
    oled.text(str(temp1),88,15)
    oled.text("Sensor 2: ",1,25)
    oled.text(str(temp2),88,25)

    # Finally update the oled display so the image & text is displayed
    oled.show()
    
    
####################### Connect to MQTT Broker###################
def connect_mqtt():
  global client_id, mqtt_server
  try:
    client = MQTTClient(client_id, mqtt_server)
    #client = MQTTClient(client_id, mqtt_server, user=your_username, password=your_password)
    client.DEBUG = True

    client.connect()
    #client.loop_start()
    print('Connected to %s MQTT broker' % (mqtt_server))

    return client
  except:
    print("Error: MQTT client not reachable")
     

######################### restart MQTT
def restart_and_reconnect():
  failsafe()
  print('Failed to connect to MQTT broker. Reconnecting...')
  #time.sleep(10)
  #machine.reset()

###########publish to mqtt#################
def publishtomqtt():
    global client_id, mqtt_server
    temp1_mqtt = (b'{0:3.1f}'.format(temp1))
    temp2_mqtt = (b'{0:3.1f}'.format(temp2))
    output1_mqtt = (b'{0:3.1f}'.format(output1))
    output2_mqtt = (b'{0:3.1f}'.format(output2))

    try:
        client = MQTTClient(client_id, mqtt_server)
        #client = MQTTClient(client_id, mqtt_server, user=your_username, password=your_password)
        client.DEBUG = True

        client.connect()
        #client.loop_start()
        print('Connected to %s MQTT broker' % (mqtt_server))
    
    
    except OSError as e:
      #restart_and_reconnect()
      print("Error puplishtomqtt")
      pass
    
    #client = connect_mqtt()
    client.publish(topic_pub_temp1, temp1_mqtt)
    client.publish(topic_pub_temp2, temp2_mqtt)
    client.publish(topic_pub_output1, output1_mqtt)
    client.publish(topic_pub_output2, output2_mqtt)
    
    client.disconnect()
    

#######################Heater Failsafe##############################

def failsafe():
    pwm0.duty_u16(0)
    pwm1.duty_u16(0)
    print("Heating Stopped!")
  
    
    
    ################### Main Logic ##################################


checktemperature()
 
while True:
    if (time.ticks_ms()-timeStamp)>5000:
        timeStamp=time.ticks_ms()

    if tempdifference <= maxtemperaturedifference:

        try:           
            checktemperature()
            displayonOLED()
                        
            if (time.ticks_ms()-timeStamp)>5000:
                if datalogging == True :
                    logData()
                    
                publishtomqtt()
                timeStamp=time.ticks_ms()
                
                

            
            now = utime.time()
            dt= now-lastupdate
            
            if dt > checkin:
                error=tempsetpoint-temp1
                integral = integral + dt * error
                derivative = (error - lasterror)/dt
                output1 = Kp * error + Ki * integral + Kd * derivative
                #print(str(output)+"= Kp term: "+str(Kp*error)+" + Ki term:" + str(Ki*integral) + "+ Kd term: " + str(Kd*derivative))
                output1 = max(min(100, output1), 0) # Clamp output between 0 and 100
                scaled_output1 = scale_value(output1, 0, 100, 0, 65535)
                
                error=tempsetpoint-temp2
                integral = integral + dt * error
                derivative = (error - lasterror)/dt
                output2 = Kp * error + Ki * integral + Kd * derivative
                #print(str(output)+"= Kp term: "+str(Kp*error)+" + Ki term:" + str(Ki*integral) + "+ Kd term: " + str(Kd*derivative))
                output2 = max(min(100, output2), 0) # Clamp output between 0 and 100
                scaled_output2 = scale_value(output2, 0, 100, 0, 65535)
                
                print('Output1: ', round(output1),'%    ', 'Output2: ', round(output2),'%    ', 'Temp_max:  ', temp_max,'°C     ', 'Temp1: ', temp1, '°C    ', 'Temp2: ', temp2, '°C')
                #print('Output: ', output,'%    ', 'Temp_max:  ', temp_max,'°C     ', 'Temp1: ', temp1, '°C    ', 'Temp2: ', temp2, '°C', 'Temp3: ', temp3, '°C') #uncomment if 3 sensors will be used
                
                if output1>0:  
                    pwm0.duty_u16(scaled_output1)
                if output2>0:  
                    pwm1.duty_u16(scaled_output2)

                else:
                    pwm0.duty_u16(0)
                    pwm1.duty_u16(0)

                #utime.sleep(.1)
                lastupdate = now
                lasterror = error
                
        except Exception as e:
            if (time.ticks_ms()-timeStamp)>1000:
                timeStamp=time.ticks_ms()
                failsafe()
                print('error encountered:'+str(e))
                
                utime.sleep(checkin)
    else:
            if (time.ticks_ms()-timeStamp)>1000:
                timeStamp=time.ticks_ms()
                print("Temperature difference >5!")
                checktemperature()
                displayonOLED_tempdiff()
                failsafe()

  


