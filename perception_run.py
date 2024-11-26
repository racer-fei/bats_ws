import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time
import serial

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # Configuração dos pinos para os sensores
        self.sensor1_pub = self.create_publisher(Float32, 'sensor1/read', 10)
        self.sensor2_pub = self.create_publisher(Float32, 'sensor2/read', 10)
        self.sensor3_pub = self.create_publisher(Float32, 'sensor3/read', 10)
        self.sensor4_pub = self.create_publisher(Float32, 'sensor4/read', 10)

        # Configuração dos pinos para os sensores HC-SR04
        self.trig_pin1 = 11
        self.echo_pin1 = 13
        self.trig_pin2 = 16
        self.echo_pin2 = 18
        self.trig_pin3 = 29
        self.echo_pin3 = 31
        self.trig_pin4 = 32
        self.echo_pin4 = 33

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.trig_pin1, GPIO.OUT)
        GPIO.setup(self.echo_pin1, GPIO.IN)
        GPIO.setup(self.trig_pin2, GPIO.OUT)
        GPIO.setup(self.echo_pin2, GPIO.IN)
        GPIO.setup(self.trig_pin3, GPIO.OUT)
        GPIO.setup(self.echo_pin3, GPIO.IN)
        GPIO.setup(self.trig_pin4, GPIO.OUT)
        GPIO.setup(self.echo_pin4, GPIO.IN)

        GPIO.output(self.trig_pin1, GPIO.LOW)
        GPIO.output(self.trig_pin2, GPIO.LOW)
        GPIO.output(self.trig_pin3, GPIO.LOW)
        GPIO.output(self.trig_pin4, GPIO.LOW)
        time.sleep(2)

        # Comunicação serial com a ESP32 via UART
        self.uart = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)

    def measure_distance(self, trig_pin, echo_pin):
        GPIO.output(trig_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trig_pin, GPIO.LOW)

        pulse_start = time.time()
        while GPIO.input(echo_pin) == 0:
            pulse_start = time.time()

        pulse_end = time.time()
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        return round(distance, 2)

    def run(self):
        while rclpy.ok():
            # Medir distâncias dos sensores
            distance1 = self.measure_distance(self.trig_pin1, self.echo_pin1)
            distance2 = self.measure_distance(self.trig_pin2, self.echo_pin2)
            distance3 = self.measure_distance(self.trig_pin3, self.echo_pin3)
            distance4 = self.measure_distance(self.trig_pin4, self.echo_pin4)

            # Publicar distâncias
            self.sensor1_pub.publish(Float32(data=distance1))
            self.sensor2_pub.publish(Float32(data=distance2))
            self.sensor3_pub.publish(Float32(data=distance3))
            self.sensor4_pub.publish(Float32(data=distance4))

            # Enviar comandos para a ESP32 com base nas distâncias
            if distance1 < 20:  # Se o sensor 1 detectar um obstáculo a menos de 20 cm
                self.uart.write(b'L')  # Enviar comando 'L' para virar à esquerda
            elif distance2 < 20:  # Se o sensor 2 detectar um obstáculo à esquerda
                self.uart.write(b'R')  # Enviar comando 'R' para virar à direita
            elif distance3 < 20:  # Se o sensor 3 detectar um obstáculo à frente
                self.uart.write(b'B')  # Enviar comando 'B' para ir para trás
            else:
                self.uart.write(b'F')  # Caso contrário, ir para frente

            # Log para verificação
            self.get_logger().info(f'Distância Sensor 1: {distance1} cm')
            self.get_logger().info(f'Distância Sensor 2: {distance2} cm')
            self.get_logger().info(f'Distância Sensor 3: {distance3} cm')
            self.get_logger().info(f'Distância Sensor 4: {distance4} cm')

            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    node.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
