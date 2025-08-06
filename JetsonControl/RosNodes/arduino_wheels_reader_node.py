import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import logging
import pyudev

# Set up logger for Arduino commands and responses 
arduino_logger = logging.getLogger("arduino_commands")
arduino_logger.setLevel(logging.INFO)
file_handler = logging.FileHandler("arduino_commands.log")
formatter = logging.Formatter('%(asctime)s - %(message)s')
file_handler.setFormatter(formatter)
if not arduino_logger.hasHandlers():
    arduino_logger.addHandler(file_handler)

class ArduinoWheelsReaderNode(Node):
    def __init__(self):
        super().__init__('arduino_wheels_reader_node')

        # Use pyudev to find the device with specific idVendor and idProduct
        context = pyudev.Context()
        device = None
        for dev in context.list_devices(subsystem='tty'):
            if dev.get('ID_VENDOR_ID') == '2a03' and dev.get('ID_MODEL_ID') == '0043':
                device = dev.device_node
                break

        if not device:
            raise RuntimeError('Arduino device not found with idVendor=2a03 and idProduct=0043')

        self.ser = serial.Serial(device, 9600, timeout=1)  # Connect to the identified device
        self.get_logger().info(f'Serial reader connected to Arduino on {device}.')

        self.publisher = self.create_publisher(
            String, 
            'wheels_response_topic',
            10
        )
        self.running = True
        self.thread = threading.Thread(target=self.read_serial)
        self.thread.daemon = True
        self.thread.start()

    def read_serial(self):
        while self.running:
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode().strip()
                if response:
                    msg = String()
                    msg.data = response
                    self.publisher.publish(msg)
                    # Log the response from Arduino
                    arduino_logger.info(f"RESPONSE FROM ARDUINO: {response}")
                    self.get_logger().info(f'Received from Arduino: {response}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoWheelsReaderNode()
    try:
        rclpy.spin(node)
    finally:
        node.running = False
        node.destroy_node()

if __name__ == '__main__':
    main()