import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import threading
import time
from enum import Enum

class ArduinoType(Enum):
    MOTORS = "motors"
    ROBOT = "robot"

class ArduinoManagerNode(Node):
    def __init__(self):
        super().__init__('arduino_manager_node')
        
        # Publishers
        self.command_publisher = self.create_publisher(String, 'arduino_command_validated', 10)
        self.response_publisher = self.create_publisher(String, 'arduino_response', 10)
        
        # Subscribers
        self.command_subscription = self.create_subscription(
            String,
            'arduino_command',
            self.command_callback,
            10
        )
        self.raw_response_subscription = self.create_subscription(
            String,
            'arduino_response_raw',
            self.raw_response_callback,
            10
        )
        
        # Command tracking
        self.command_counter = 0
        self.pending_commands = {}
        self.command_lock = threading.Lock()
        
        # Ping mechanism
        self.last_ping_time = time.time()
        self.ping_interval = 0.1  # 100ms
        self.ping_timer = self.create_timer(self.ping_interval, self.ping_callback)
        
        self.get_logger().info('Arduino Manager Node initialized')

    def command_callback(self, msg):
        """Procesează comenzile primite de la Car.py"""
        try:
            # Parse command
            command_parts = msg.data.split(',')
            command = [int(x) for x in command_parts]
            
            # Validate command
            if self.validate_command(command):
                # Add command counter if not present
                if len(command) > 0 and command[0] == 0:
                    with self.command_lock:
                        self.command_counter += 1
                        command[0] = self.command_counter
                        self.pending_commands[self.command_counter] = time.time()
                
                # Forward validated command
                validated_msg = String()
                validated_msg.data = ','.join(map(str, command))
                self.command_publisher.publish(validated_msg)
                
                self.get_logger().info(f'Validated and forwarded command: {command}')
            else:
                self.get_logger().error(f'Invalid command rejected: {command}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')

    def validate_command(self, command):
        """Validează comenzile înainte de a le trimite"""
        if not command or len(command) < 2:
            return False
            
        # Validate command ID ranges
        cmd_id = command[1]
        
        # ArduinoCommands validation
        if cmd_id in [1, 2, 3]:  # EMERGENCY, END_EMERGENCY, PING
            return True
            
        # MotorCommands validation  
        if cmd_id in [4, 5, 6, 7]:  # START, BREAK, SET_PWM, SEND_WHEEL_SPEEDS
            if cmd_id == 6 and len(command) >= 5:  # SET_PWM needs PWM + directions
                pwm = abs(command[2])
                return 0 <= pwm <= 255  # Valid PWM range
            return True
            
        return False

    def raw_response_callback(self, msg):
        """Procesează răspunsurile raw de la Arduino"""
        try:
            # Parse response
            response_data = msg.data.strip()
            if ',' in response_data:
                response = [int(x) for x in response_data.split(',')]
            else:
                response = [int(response_data)]
            
            # Process response based on type
            processed_response = self.process_response(response)
            
            # Forward processed response
            if processed_response:
                response_msg = String()
                response_msg.data = ','.join(map(str, processed_response))
                self.response_publisher.publish(response_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing response: {e}')

    def process_response(self, response):
        """Procesează răspunsurile și aplică logica specifică"""
        if not response:
            return None
            
        # Handle ping responses (repurposed for data)
        if len(response) == 3:
            # Motors ping response (distance sensor)
            self.get_logger().debug(f'Motors ping response: {response}')
            return response
            
        elif len(response) > 3:
            # Robot ping response (position data)
            try:
                # Save start position to file
                if not os.path.exists("startPosition.txt"):
                    open("startPosition.txt", 'w').close()
                    
                with open("startPosition.txt", 'w') as file:
                    file.write(f"{response[2]}\n{response[3]}")
                    
                self.get_logger().info(f'Saved start position: x={response[2]}, y={response[3]}')
                return response
                
            except Exception as e:
                self.get_logger().error(f'Error saving start position: {e}')
                return response
        else:
            # Regular command response
            return response

    def ping_callback(self):
        """Trimite ping-uri periodice pentru a menține conectivitatea"""
        current_time = time.time()
        
        # Send ping to motors
        ping_command_motors = [0, 3]  # ArduinoCommands.PING
        msg = String()
        msg.data = ','.join(map(str, ping_command_motors))
        self.command_publisher.publish(msg)
        
        # Send ping to robot  
        ping_command_robot = [0, 3]  # ArduinoCommands.PING
        msg = String()
        msg.data = ','.join(map(str, ping_command_robot))
        self.command_publisher.publish(msg)
        
        # Cleanup old pending commands (timeout detection)
        with self.command_lock:
            expired_commands = [
                cmd_id for cmd_id, timestamp in self.pending_commands.items()
                if current_time - timestamp > 5.0  # 5 second timeout
            ]
            for cmd_id in expired_commands:
                del self.pending_commands[cmd_id]
                self.get_logger().warning(f'Command {cmd_id} timed out')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()