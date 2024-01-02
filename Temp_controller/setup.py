from machine import Pin, SoftI2C
import machine
import ssd1306
import utime
import onewire, ds18x20, time
from machine import RTC
import os
import micropython
import esp
esp.osdebug(None)
import gc
gc.collect()
import config

###################### define global variables##########################
global last_message
global message_interval
global integral
global lastupdate
global lasterror
global output1
global output2
global checkin
global timeStamp
global ds_pin
global ds_sensor
global oled
global Sensor1
global Sensor2
global Sensor3
global temp_max
global temp1
global temp2
global tempdifference
global pwm0
global pwm1


last_message = 0
message_interval = 5
integral = 0
lastupdate = utime.time()  
lasterror = 0
output1=0
output2=0
checkin = 0


#RTC setup
rtc=RTC()
timeStamp=0
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

#define temp sensor pin
ds_pin = machine.Pin(21)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))

#setup oled
WIDTH  = 128                                            # oled display width
HEIGHT = 64                                             # oled display height
 
i2c = SoftI2C(scl=Pin(8), sda=Pin(9))
 
oled_width = 128
oled_height = 64
oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)
print ("Setup done!")

