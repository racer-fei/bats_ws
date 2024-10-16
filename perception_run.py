# IMPORTANTE dar permissão aos pinos: sudo chmod 777 /de/tty23
import RPi.GPIO as GPIO
import time
GPIO.cleanup()

# Configuração dos pinos
TRIG = 13
ECHO = 11

# Configuração do GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def detect_distance():
    try:
        while True:
            # Envia um pulso de 10us no pino TRIG
            GPIO.output(TRIG, True)
            time.sleep(0.00001)
            GPIO.output(TRIG, False)

            # Espera pelo sinal de ECHO
            while GPIO.input(ECHO) == 0:
                pulse_start = time.time()

            while GPIO.input(ECHO) == 1:
                pulse_end = time.time()

            # Calcula a distância
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Em centímetros
            distance = round(distance, 2)

            # Imprime a distância no terminal
            print(f"Distância: {distance} cm")

            # Aguarda um pouco antes da próxima leitura
            time.sleep(1)

    except KeyboardInterrupt:
        print("Medidas interrompidas.")
        GPIO.cleanup()
