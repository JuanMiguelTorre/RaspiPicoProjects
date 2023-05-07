import network
import time
from machine import Pin
from umqtt.simple import MQTTClient
led_g = Pin(27, Pin.OUT)
led_g.low()
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect("MOVISTAR_3578","BLVZf8f8GvFA34Au5Q38")
time.sleep(5)
print(wlan.isconnected())
sensor = Pin(16, Pin.IN)
led_r = Pin(17, Pin.OUT)
led_r.low()
mqtt_server = 'broker.hivemq.com'
client_id = 'bigles'
topic_pub = b'TomsHardware'
topic_msg = b'Movimiento Detectado'
def mqtt_connect():
     client = MQTTClient(client_id, mqtt_server, keepalive=3600)
     client.connect()
     print('Conectado a %s MQTT Broker'%(mqtt_server))
     led_g.high()
     return client
def reconnect():
   print('Failed to connect to the MQTT Broker. Reconnecting...')
   led_g.low()
   time.sleep(5)
   machine.reset()
try:
   client = mqtt_connect()
except OSError as e:
   reconnect()
while True:
   if sensor.value() == 1:    
        client.publish(topic_pub, topic_msg)
        led_r.high()
        time.sleep(5)
   else:
       led_r.low()
       pass
    
       