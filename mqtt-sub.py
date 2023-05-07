#Importar cuatro módulos de código Python para conectividad Wi-Fi, controle el ritmo del código, acceda a los pines GPIO y MQTT.
import network
import time
from machine import Pin
from umqtt.simple import MQTTClient
#Este led verde de conexion a internet se crea antes de la propia conexion para que comience en baja
led_g = Pin(27, Pin.OUT)
led_g.low()
#Crear un objeto, wlan, que luego se usa para conectarse a su punto de acceso Wi-Fi.
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect("MOVISTAR_3578","BLVZf8f8GvFA34Au5Q38")
#Agregar una pausa de cinco segundos, luego imprima el estado de la conexión Wi-Fi. 
time.sleep(5)
print(wlan.isconnected())
#Crear un objeto, LED, para almacenar y configurar el pin GPIO al que se ha conectado un LED como salida.
LED = Pin(17, Pin.OUT)
# Cree tres variables para almacenar la URL de nuestro agente MQTT, nuestro client_id y el tema.
mqtt_server = 'broker.hivemq.com'
client_id = 'bigles'
topic_sub = b'TomsHardware'
#Crear una función, sub_cb, que escuchará y reaccionará a los mensajes sobre nuestro tema elegido.
#Esta función toma dos argumentos, el tema y el mensaje en sí.
def sub_cb(topic, msg):
    #Imprima una alerta en el shell de Python que indique que hay un nuevo mensaje sobre el tema.
    #Necesitamos decodificar el tema para que sea más legible.
    print("New message on topic {}".format(topic.decode('utf-8')))
    #Crear un objeto, msg, y allí almacene el mensaje MQTT decodificado. Luego imprimir el mensaje en el shell de Python.
    msg = msg.decode('utf-8')
    print(msg)
    #Crear una declaración condicional para verificar el contenido del objeto msg. Si el mensaje contiene "on", encenderá el LED.
    if msg == "on":
       LED.on()
    #Crear una condición else if (elif) que apague el LED si el objeto msg contiene "off".
    elif msg == "off":
       LED.off()
    elif msg == "reset":
       machine.reset() 
#Cree una función para manejar las conexiones MQTT.
#Usando una función, podemos llamar fácilmente a la función más adelante en el código, sin necesidad de repetir el contenido de la función.
def mqtt_connect():
    # Crear un objeto, cliente, y utilícelo para almacenar el ID_cliente, los detalles del intermediario MQTT,
    #..junto con una opción para mantener la conexión activa durante una hora.
    client = MQTTClient(client_id, mqtt_server, keepalive=3600)
    client.set_callback(sub_cb)
    #Utilice el objeto de cliente para conectarse al intermediario MQTT.
    client.connect()
    #Imprimir un mensaje en el shell de python después de una conexión exitosa,
    #luego devuelva el contenido del objeto del cliente al shell. Esto es útil para la depuración.
    print('Connected to %s MQTT Broker'%(mqtt_server))
    led_g.high()
    return client
# Crear una nueva función, reconectar, que hará una pausa de cinco segundos antes de reiniciar el Pico W.
#Esto obligará al Pico W a intentar volver a conectarse.
def reconnect():
   print('Failed to connect to the MQTT Broker. Reconnecting...')
   led_g.low()
   time.sleep(5)
   machine.reset()
# Crear un controlador de excepciones que intentará conectarse al intermediario MQTT mediante la función mqtt_connect().
try:
   client = mqtt_connect()
# Agregar una excepción para que, si el código no se conecta, se genere un error,
#.. lo que obligará al Pico W a reiniciarse mediante la función reconectar().
except OSError as e:
   reconnect()
# Crear un bucle para ejecutar constantemente nuestro código.
while True:
    #Suscríbirse al tema, TomsHardware, utilizando el objeto topic_sub. Luego haga una pausa de un segundo.
    client.subscribe(topic_sub)
    time.sleep(1)
    