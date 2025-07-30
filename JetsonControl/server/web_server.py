from flask import Flask, request, send_from_directory
import rospy
from std_msgs.msg import String

app = Flask(__name__, static_folder='.')

# ROS initialization
rospy.init_node('web_server_node', anonymous=True)
pub = rospy.Publisher('aruino_command', String, queue_size=10)

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

@app.route('/<path:path>')
def static_files(path):
    return send_from_directory('.', path)

@app.route('/command/<cmd>', methods=['POST'])
def send_command(cmd):
    pub.publish(cmd)
    return {'status': 'OK'}


def run_server():
    app.run(host='0.0.0.0', port=8080)

if __name__ == '__main__':
    run_server()