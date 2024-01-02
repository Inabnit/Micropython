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
from Temp_Controller.config import tempsetpoint, maxtemperaturedifference, Kp, Ki, Kd, Sensor1, Sensor2, Sensor3
from Temp_Controller.setup import ds_sensor, timeStamp


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
#######################Heater Failsafe##############################

def failsafe():
    pwm0.duty_u16(0)
    pwm1.duty_u16(0)
    print("Heating Stopped!")
  
    
    ################### Main Logic ##################################


checktemperature()

def controltemp():
    global timeStamp
    global pwm0
    global pwm1

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

