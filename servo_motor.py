from machine import Pin,PWM    #modulos que controlan los pines y el servo
#servo actuador
servo = PWM(Pin(8))
servo.freq(50)
continuar = True
while continuar:
    a = input ("Introduzca el valor del ángulo:  ")
    if a == "fin":
        continuar = False
        servo.deinit()
        print("Fin de programa")
    else:
        ton = ((int(a) + 45)/9)
        servo.duty_ns(int(ton))
        