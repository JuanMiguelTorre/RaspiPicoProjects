#Importar los siguientes módulos de código Python para control sensor DHT, conectividad Wi-Fi, controle el ritmo del código,
#.. acceda a los pines GPIO y MQTT. ..
import dht
from dht import DHT11  #librería/módulo para controlar el sensor DHT11
#from pigpio_dht import DHT11
import network
import time
from time import sleep_ms
from machine import Pin,PWM    #modulos que controlan los pines y el servo
from machine import I2C       #modulo para controlar la pantalla LCD
from esp8266_i2c_lcd import I2cLcd #Código de Python para comunicarse con LCD de matriz de puntos basados ​​en caracteres compatibles con HD44780.
from utime import sleep
from umqtt.simple import MQTTClient

#Se crean las variable globales temperatura
global temperatura
global temperatura2
global temp
temp = 0
# pantalla LCD
sda = Pin(0)     # objeto tipo pin que especifica el pin que se utilizará para SDA, en este caso utilizaremos GP0 como sda                                                                # en este caso utilizaremos GP0 como sda
scl = Pin(1)     # objeto tipo pin que especifica el pin que se utilizará para SCL, en este caso utilizaremos GP1 como scl                                                               # en este caso utilizaremos GP1 como scl
i2c = I2C(0, sda=sda, scl=scl, freq=400000)   # le decimos que vamos a usar I2C cero y frecuencia max de 400k para scl                                                                               max de 400k para scl
# The PCF8574 has a jumper selectable address: 0x20 - 0x27
DEFAULT_I2C_ADDR = 0x3F
#sensores de temperatura
sensorDHT = DHT11 (Pin(6))
sensorDHT2 = DHT11 (Pin(15))
#servo actuador
servo = PWM(Pin(8))
servo.freq(50)
continuar = True
#Este led verde de conexion a internet se crea antes de la propia conexion para que comience en baja
led_g = Pin(27, Pin.OUT)
led_g.low()

i2c = I2C(0,scl=Pin(1), sda=Pin(0), freq=100000)
lcd = I2cLcd(i2c, DEFAULT_I2C_ADDR, 2, 16)
lcd.putstr("Inicio proyecto\nJuanMiguel Torre")
sleep_ms(3000)
lcd.display_off()
#Crear un objeto, wlan, que luego se usa para conectarse a su punto de acceso Wi-Fi.
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect("MOVISTAR_3578","BLVZf8f8GvFA34Au5Q38")
#Agregar una pausa de cinco segundos, luego imprima el estado de la conexión Wi-Fi. 
time.sleep(5)
print(wlan.isconnected())
sensor = Pin(16, Pin.IN)
#Crear un objeto, LED, para almacenar y configurar el pin GPIO al que se ha conectado un LED como salida.
led_b = Pin(17, Pin.OUT)
led_r = Pin(2, Pin.OUT)
led_b.low()
led_r.low()
BUZZER= PWM(Pin(22))
# Cree tres variables para almacenar la URL de nuestro agente MQTT, nuestro client_id y el tema.
mqtt_server = 'broker.hivemq.com'
client_id = 'bigles'
topic_sub = b'JMT'
topic_pub = b'JMT'
topic_msg = b'Movimiento Detectado'
#Crear una función, sub_cb, que escuchará y reaccionará a los mensajes sobre nuestro tema elegido.
#Esta función toma dos argumentos, el tema y el mensaje en sí.
def sub_cb(topic, msg):
    #Imprima una alerta en el shell de Python que indique que hay un nuevo mensaje sobre el tema.
    #Necesitamos decodificar el tema para que sea más legible.
    print("New message on topic {}".format(topic.decode('utf-8')))
    #Crear un objeto, msg, y allí almacene el mensaje MQTT decodificado. Luego imprimir el mensaje en el shell de Python.
    msg = msg.decode('utf-8')
    print(msg)
    #Crear una declaración condicional para verificar el contenido del objeto msg. Si el mensaje contiene "on", sonara la alarma (BUZZER).
    if msg == "on":
       # while True:
           led_r.on()
           lcd.clear()
           lcd.putstr(" ALARMA!!! ")
           i = 0
           while i < 4:
               BUZZER.freq(300)
               BUZZER.duty_u16(1500)
               sleep(1)
               led_r.off()
               lcd.clear()
               lcd.move_to(4,1)
               lcd.putstr(" ALARMA!!! ")
               BUZZER.freq(500)
               BUZZER.duty_u16(1500)
               sleep(1)
               led_r.on()
               lcd.clear()
               lcd.move_to(0,0)
               lcd.putstr(" ALARMA!!! ")
               i=i+1
           sleep(1)
           BUZZER.duty_u16(0)
           sleep(1)
        # pass
    #Crear una condición else if (elif) que apague el BUZZER si el objeto msg contiene "off".
    elif msg == "off":
        lcd.clear()
        lcd.putstr(" Alarma         desconectada")
        sleep(3)
        led_r.off()
        lcd.clear()
        
        BUZZER.duty_u16(0)
    elif msg == "temperatura":
        #si se elige esta opción imprime por pantalla tft y consola las temperaturas que el sensor DHT11 va consiguiendo
        print(f"Temperatura dentro:  {temperatura}")
        print(f"Temperatura fuera:  {temperatura2}")
        #client.publish(temperatura)
        lcd.clear() 
        lcd.putstr("Temp IN: " )#%str(temperatura))
        lcd.putstr (str (temperatura))    #no se puede hacer en una misma instruccion
        lcd.move_to(0,1)
        lcd.putstr("Temp OUT: " )#%str(temperatura2))
        lcd.putstr (str (temperatura2))
        sleep(3)
        lcd.clear()    #limpia la pantalla LCD independientemente si está encendida o no
        temp = 1
        return temp
    elif msg == "reg_ventana":
        lcd.clear()
        lcd.putstr("Regulando          ventana")
        sleep(3)
        lcd.clear() 
        lcd.putstr("Temp IN: " )
        lcd.putstr (str (temperatura))    #no se puede hacer en una misma instruccion
        lcd.move_to(0,1)
        lcd.putstr("Temp OUT: " )
        lcd.putstr (str (temperatura2))
        sleep(3)
        if temperatura >= 24 & temperatura2 >= 20 & temperatura2 <= 30:
            lcd.clear()
            lcd.putstr("La ventana se está abriendo")
            sleep(3)
            servo.duty_ns(2500000)  # servo a 180 grados indica que la ventana se abre son 2500000 nanosegundos)
            sleep(3)
        else:
            lcd.clear()
            lcd.putstr("La ventana se está cerrando")
            sleep(3)
            servo.duty_ns(500000)  # servo a 0 grados indica que la ventana se cierra son 500000 nanosegundos)
            sleep(3)
    #Crear una condición else if (elif) que desconecte la placa si el objeto msg contiene "reset"   
    elif msg == "reset":
       lcd.clear() 
       lcd.putstr(" Apagando          sistema")
       sleep(2)
       lcd.clear()
       sleep(2)
       lcd.backlight_off()
       lcd.display_off()    # deben estar antes de machine.reset()
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
    BUZZER.freq(500)
    BUZZER.duty_u16(1000)
    sleep(2)
    lcd.backlight_off()
    BUZZER.duty_u16(0)
    sleep(1)
    lcd.clear()   #limpia la pantalla LCD independientemente si está encendida o no
    lcd.display_on()
    lcd.backlight_on()
    servo.duty_ns(1500000)  # servo a 90 grados indica que la ventana esta semiabierta son 1500000 nanosegundos)
    lcd.putstr(" Conexion realizada con exito")
    return client
# Crear una nueva función, reconectar, que hará una pausa de cinco segundos antes de reiniciar el Pico W.
#Esto obligará al Pico W a intentar volver a conectarse.
def reconnect():
   print('Failed to connect to the MQTT Broker. Reconnecting...')
   lcd.putstr("Fallo en conexion")
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
    #Suscríbirse al tema, JMT, utilizando el objeto topic_sub. Luego haga una pausa de un segundo.
    client.subscribe(topic_sub)
    time.sleep(1)
    
    sensorDHT.measure ()
    sensorDHT2.measure ()
    sleep_ms (2000)
    
    temperatura = sensorDHT.temperature()
    temperatura2 = sensorDHT2.temperature()
    
    if temp == 1:
       client.publish(temperatura)
       client.publish(temperatura2)
       client.suscribe(temperatura)
       client.suscribe(temperatura2)
    #Se verifica si el sensor se ha activado. Los sensores PIR HC-SR501 tienen un estado natural en el que su pin de salida tiene un valor de 0. Cuando activamos el sensor, el pin de salida sube y el valor cambia a 1.
    if sensor.value() == 1:    
#Con el sensor activado, se publica un mensaje a través de MQTT, utilizando el tema y el mensaje que creamos anteriormente.
#Luego se hace una pausa de cinco segundos, esto le da tiempo al sensor para reiniciarse, listo para el próximo evento.
        client.publish(topic_pub, topic_msg)
        lcd.clear()
        lcd.putstr(" Deteccion de   movimiento")
        led_b.high()
        time.sleep(7)
        lcd.clear()
#Se agrega una condición else que se activará si el sensor permanece sin activarse. El uso de pase permite que el código avance sin salida.
    else:
        led_b.low()
      # pass
    
    