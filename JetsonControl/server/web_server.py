from flask import Flask, request, send_from_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from flask import Response

app = Flask(__name__, static_folder='.')

# ROS2 Node wrapper
class ROS2Publisher(Node):
    def __init__(self):
        super().__init__('web_server_node')
        
        # Create publishers for different topics
        self.wheels_publisher_ = self.create_publisher(String, 'wheels_command_topic', 10)
        self.arm_publisher_ = self.create_publisher(String, 'arm_command_topic', 10)
        self.control_mode_publisher_ = self.create_publisher(String, 'control_mode_topic', 10)

    def publish(self, msg, topic):
        try:
            msg_obj = String()
            msg_obj.data = msg
            
            if topic == 'wheels_command_topic':
                self.wheels_publisher_.publish(msg_obj)
            elif topic == 'arm_command_topic':
                self.arm_publisher_.publish(msg_obj)
            elif topic == 'control_mode_topic':
                self.control_mode_publisher_.publish(msg_obj)
            else:
                print(f"[ROS2Publisher] Unknown topic: {topic}")
        except Exception as e:
            print(f"[ROS2Publisher] ERROR at publish: {e}")

# Start rclpy and node in a background thread
import threading
rclpy.init()
ros2_node = ROS2Publisher()
ros2_node.publish("manual", "control_mode_topic")  # Publish 'manual' at startup
ros2_executor = rclpy.executors.SingleThreadedExecutor()
ros2_executor.add_node(ros2_node)

def spin_ros():
    ros2_executor.spin()
ros_thread = threading.Thread(target=spin_ros, daemon=True)

ros_thread.start()

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

def gen_frames(camera_id):
    cap = cv2.VideoCapture(camera_id)
    print(f"[Flask] Starting video capture for camera {camera_id}")
    while True:
        success, frame = cap.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/car_camera_feed')
def car_video_feed():
    print("[Flask] Serving car video feed")
    return Response(gen_frames(0),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
    
@app.route('/arm_camera_feed')
def arm_video_feed():
    print("[Flask] Serving arm video feed")
    return Response(gen_frames(1),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/<path:path>')
def static_files(path):
    return send_from_directory('.', path)

@app.route('/manual', methods=['POST']) 
def catch_command_manual():
    try:
        ros2_node.publish("manual", "control_mode_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/selfDriving', methods=['POST'])
def catch_command_self_driving():
    try:
        ros2_node.publish("selfDriving", "control_mode_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/forward/<pwm_value>', methods=['POST'])
def catch_command_forward(pwm_value):
    try:
        msg = f"<0, 2, {pwm_value}, {pwm_value}, 0, 0>"
        ros2_node.publish(msg, "wheels_command_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/backward/<pwm_value>', methods=['POST'])
def catch_command_backward(pwm_value):
    try:
        msg = f"<0, 2, {pwm_value}, {pwm_value}, 1, 1>"
        ros2_node.publish(msg, "wheels_command_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/turnLeft/<pwm_value>', methods=['POST'])
def catch_command_turn_left(pwm_value):
    try:
        msg = f"<0, 2, {pwm_value}, {pwm_value}, 0, 1>"
        ros2_node.publish(msg, "wheels_command_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/turnRight/<pwm_value>', methods=['POST'])
def catch_command_turn_right(pwm_value):
    try:
        msg = f"<0, 2, {pwm_value}, {pwm_value}, 1, 0>"
        ros2_node.publish(msg, "wheels_command_topic")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

@app.route('/<path:cmd>', methods=['POST'])
def catch_all_commands(cmd):
    print(f"[Flask] Received POST /{cmd}")
    try:
        ros2_node.publish(cmd)
        print(f"[Flask] Published command: {cmd}")
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

def run_server():
    app.run(host='0.0.0.0', port=8080)

if __name__ == '__main__':
    run_server()
    

