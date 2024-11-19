import machine
import time
import uasyncio as asyncio

# GPIO da ESP32
MOTOR_LEFT_PIN1 = machine.Pin(12, machine.Pin.OUT) 
MOTOR_LEFT_PIN2 = machine.Pin(13, machine.Pin.OUT)
MOTOR_RIGHT_PIN1 = machine.Pin(2, machine.Pin.OUT)
MOTOR_RIGHT_PIN2 = machine.Pin(15, machine.Pin.OUT)
BACK_MOTOR_PIN = machine.Pin(6, machine.Pin.OUT)

#commands seriais da Rasp
async def commandList(command):
    if command == 'L':
        turnLeft()
    elif command == 'R':
        turnRight()
    elif command == 'F':
        goForward()
    elif command == 'B':
        goBackward()
    elif command == 'O':
        openBack()
    elif command == 'C':
        closeBack()
    else:
        stop_all()

def turnLeft():
    MOTOR_LEFT_PIN1.value(0)
    MOTOR_LEFT_PIN2.value(1)
    MOTOR_RIGHT_PIN1.value(1)
    MOTOR_RIGHT_PIN2.value(0)

def turnRight():
    MOTOR_LEFT_PIN1.value(1)
    MOTOR_LEFT_PIN2.value(0)
    MOTOR_RIGHT_PIN1.value(0)
    MOTOR_RIGHT_PIN2.value(1)

def goForward():
    MOTOR_LEFT_PIN1.value(1)
    MOTOR_LEFT_PIN2.value(0)
    MOTOR_RIGHT_PIN1.value(1)
    MOTOR_RIGHT_PIN2.value(0)

def goBackward():
    MOTOR_LEFT_PIN1.value(0)
    MOTOR_LEFT_PIN2.value(1)
    MOTOR_RIGHT_PIN1.value(0)
    MOTOR_RIGHT_PIN2.value(1)

def openBack():
    BACK_MOTOR_PIN.value(1)  

def closeBack():
    BACK_MOTOR_PIN.value(0)

def stop_all():
    MOTOR_LEFT_PIN1.value(0)
    MOTOR_LEFT_PIN2.value(0)
    MOTOR_RIGHT_PIN1.value(0)
    MOTOR_RIGHT_PIN2.value(0)

#Comunicação serial entre RASP e Esp32; Tx - Transfere/Transfer, Rx - Recebe/Read
uart = machine.UART(1, baudrate=9600, tx=17, rx=16)

async def main():
    while True:
        if uart.any():
            command = uart.read(1).decode('utf-8')
            await commandList(command)
        await asyncio.sleep(0.1)

#ativa o loop principal de leituras
asyncio.run(main())
