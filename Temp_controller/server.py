import asyncio
import network
import ubinascii
import machine


from Temp_Controller.config import ssid, password, mqtt_server

def connectwifi():
    station = network.WLAN(network.STA_IF)
    station.active(True)
    station.connect(ssid, password)

    while station.isconnected() == False:
      pass

    print('Connection to WIFI successful')

############### MQTT ########################

client_id = ubinascii.hexlify(machine.unique_id())


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
    

