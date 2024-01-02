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
    
    
