#Importar cuatro módulos de código Python para conectividad Wi-Fi, controle el ritmo del código, acceda a los pines GPIO y MQTT.
import network
import time
from machine import Pin
from umqtt.simple import MQTTClient
#Este led verde de conexion a internet se crea antes de la propia conexion para que comience en baja
led_g = Pin(27, Pin.OUT)
led_g.low()
#Se un objeto, wlan, que luego se usa para conectarse a su punto de acceso Wi-Fi.
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
#"Wi-Fi AP","PASSWORD"
wlan.connect("MOVISTAR_3578","BLVZf8f8GvFA34Au5Q38")
#Agreguar una pausa de cinco segundos, luego imprima el estado de la conexión Wi-Fi. 
time.sleep(5)
print(wlan.isconnected())
#Se Crea un objeto, sensor, para vincular nuestro sensor PIR al GPIO 16 de la Raspberry Pi Pico W.
sensor = Pin(16, Pin.IN)
led_r = Pin(17, Pin.OUT)
led_r.low()
#Se Crea cuatro variables para almacenar la URL de nuestro agente MQTT, nuestro client_id, el tema y el mensaje.
mqtt_server = 'broker.hivemq.com'
client_id = 'bigles'
topic_pub = b'TomsHardware'
topic_msg = b'Movement Detected'
#Crear una función para manejar las conexiones MQTT.
def mqtt_connect():
#Cree un objeto, cliente, y utilícelo para almacenar el ID_cliente, los detalles del intermediario MQTT, junto con una opción para mantener la conexión activa durante una hora.
     client = MQTTClient(client_id, mqtt_server, keepalive=3600)
#Se utiliza el objeto de cliente para conectarse al intermediario MQTT.
     client.connect()
#Imprimir un mensaje en el shell de python después de una conexión exitosa, luego devuelva el contenido del objeto del cliente al shell.
     print('Connected to %s MQTT Broker'%(mqtt_server))
     led_g.high()
     return client
#Se crea una nueva función, reconectar, que hará una pausa de cinco segundos antes de reiniciar el Pico W. Esto obligará al Pico W a intentar volver a conectarse.
def reconnect():
   print('Failed to connect to the MQTT Broker. Reconnecting...')
   led_g.low()
   time.sleep(5)
   machine.reset()
#Se crea un controlador de excepciones que intentará conectarse al intermediario MQTT mediante la función mqtt_connect().
try:
   client = mqtt_connect()
#Se agrega una excepción para que, si el código no se conecta, se genere un error, lo que obligará al Pico W a reiniciarse mediante la función reconectar().
except OSError as e:
   reconnect()
#Se crea un bucle para verificar constantemente el valor del sensor.
while True:
#Se verifica si el sensor se ha activado. Los sensores PIR HC-SR501 tienen un estado natural en el que su pin de salida tiene un valor de 0. Cuando activamos el sensor, el pin de salida sube y el valor cambia a 1.
   if sensor.value() == 1:    
#Con el sensor activado, se publica un mensaje a través de MQTT, utilizando el tema y el mensaje que creamos anteriormente.
#Luego se hace una pausa de cinco segundos, esto le da tiempo al sensor para reiniciarse, listo para el próximo evento.
        client.publish(topic_pub, topic_msg)
        #led.toggle()
        led_r.high()
        time.sleep(5)
        #sensor.low()
#Se agrega una condición else que se activará si el sensor permanece sin activarse. El uso de pase permite que el código avance sin salida.
   else:
       led_r.low()
       pass
    
       