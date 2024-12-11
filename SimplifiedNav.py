import random
import time

class SimplifiedNavigation:
    def __init__(self, uart):
        self.uart = uart

    def sendCommand(self, command):
        """Send command to robot via UART."""
        self.uart.write(command.encode())
        print(f"Command sent: {command}")
        time.sleep(0.1)  # Small delay to allow command processing

    def pathFinder1(self):
        """Path-finding routine 1."""
        self.sendCommand('F')  # Forward
        time.sleep(2)
        self.sendCommand('L')  # Turn left
        time.sleep(0.5)
        self.sendCommand('F')  # Forward
        time.sleep(1)
        self.sendCommand('R')  # Turn right
        time.sleep(0.5)
        self.sendCommand('F')  # Forward
        time.sleep(6)
        self.sendCommand('L')  # Turn left
        time.sleep(0.5)
        self.sendCommand('F')  # Forward
        time.sleep(2)
        self.sendCommand('L')  # Turn left
        time.sleep(1)
        self.sendCommand('B')  # Go backward
        time.sleep(1)

    def pathFinder2(self):
        """Path-finding routine 2."""
        self.sendCommand('F')  # Forward
        time.sleep(3)
        self.sendCommand('L')  # Turn left
        time.sleep(0.5)
        self.sendCommand('F')  # Forward
        time.sleep(6)
        self.sendCommand('L')  # Turn left
        time.sleep(0.5)
        self.sendCommand('F')  # Forward
        time.sleep(2)
        self.sendCommand('L')  # Turn left
        time.sleep(1)
        self.sendCommand('B')  # Go backward
        time.sleep(1)
        pass

    def pathFinder3(self):
        """Path-finding routine 3."""
        self.sendCommand('F')  # Forward
        time.sleep(3)
        self.sendCommand('L')  # Turn left
        time.sleep(0.5)
        self.sendCommand('F')  # Forward
        time.sleep(3)
        self.sendCommand('R')  # Turn right
        time.sleep(0.5)
        self.sendCommand('F')  # Forward
        time.sleep(0.5)
        self.sendCommand('L')  # Turn left
        time.sleep(0.5)
        self.sendCommand('F')  # Turn left
        time.sleep(3)
        self.sendCommand('L')  # Turn left
        time.sleep(0.5)
        self.sendCommand('F')  # Go backward
        time.sleep(1)
        self.sendCommand('L')  # Turn left
        time.sleep(1)
        self.sendCommand('B')  # Go backward
        time.sleep(1)
        pass

    def pathFinder4(self):
        """Path-finding routine 4."""
        self.sendCommand('F')  # Forward
        time.sleep(2)
        self.sendCommand('L')  # Turn left
        time.sleep(0.5)
        self.sendCommand('F')  # Forward
        time.sleep(1)
        self.sendCommand('R')  # Turn right
        time.sleep(0.5)
        self.sendCommand('F')  # Forward
        time.sleep(7)
        self.sendCommand('L')  # Turn left
        time.sleep(0.5)
        self.sendCommand('F')  # Forward
        time.sleep(2)
        self.sendCommand('L')  # Turn left
        time.sleep(0.5)
        self.sendCommand('B')  # Go backward
        time.sleep(1)
        pass

    def pathFinder5(self):
        """Path-finding routine 5."""
        self.sendCommand('F')  # Forward
        time.sleep(2)
        self.sendCommand('L')  # Turn left
        time.sleep(0.5)
        self.sendCommand('F')  # Forward
        time.sleep(1)
        self.sendCommand('R')  # Turn right
        time.sleep(0.5)
        self.sendCommand('F')  # Forward
        time.sleep(6)
        self.sendCommand('L')  # Turn left
        time.sleep(0.5)
        self.sendCommand('F')  # Forward
        time.sleep(2)
        self.sendCommand('L')  # Turn left
        time.sleep(1)
        self.sendCommand('B')  # Go backward
        time.sleep(1)
        pass

    def executeRandomPath(self):
        """Randomly select a path-finder routine and execute it."""
        path_finders = [self.pathFinder1, self.pathFinder2, self.pathFinder3, self.pathFinder4, self.pathFinder5]
        selected_path = random.choice(path_finders)
        print(f"Executing {selected_path.__name__}")
        selected_path()

# Example Usage
if __name__ == "__main__":
    import serial

    # Initialize UART communication
    uart = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust port as necessary
    time.sleep(2)  # Allow UART to initialize

    navigator = SimplifiedNavigation(uart)

    # Loop to randomly pick and execute a path every 10 seconds
    while True:
        navigator.executeRandomPath()
        time.sleep(10)