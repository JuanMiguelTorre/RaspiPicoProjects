import dht
from dht import DHT11
from machine import Pin
from time import sleep_ms

global temp
global hum
global temp2
global hum2

sensorDHT = DHT11 (Pin(6))
sensorDHT2 = DHT11 (Pin(15))

while (True):
    sleep_ms (2000)

    sensorDHT.measure ()
    sensorDHT2.measure ()
    sleep_ms (2000)
    temp=sensorDHT.temperature ()
    hum=sensorDHT.humidity()
    temp2=sensorDHT2.temperature ()
    hum2=sensorDHT2.humidity()
    
    print ("T={:02d} ºC, H={:02d} %".format (temp,hum),"T2={:02d} ºC, H2={:02d} %".format (temp2,hum2))