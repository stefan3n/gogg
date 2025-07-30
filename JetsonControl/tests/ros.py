import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import  PoseStamped

# Simple ros node that listens to /map and prints the data
def callback(data):
    x = data.data
    print(x)


def main():
    rospy.init_node('my_python_listener', anonymous=True)
    
    # Specify the topic and message type
    topic_name = '/map'
    msg_type = OccupancyGrid
    
    # Wait for a single message on the specified topic
    try:
        msg = rospy.wait_for_message(topic_name, msg_type, timeout=10.0)
        file = open("map.txt", "w")
        file.write(str(msg.data))
    except rospy.ROSException as e:
        print("Failed to receive message:", e)
    

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("WTF")