#!/usr/bin/env python3

"""
Built as an additional node to help the ACTor GUI.

Authors:
- [Devson Butani] <dbutani@ltu.edu>

License: MIT
"""

import redis  # Redis Key Value Store Python API
import rospy  # ROS Python API
from sensor_msgs.msg import Image  # ROS Image Messages
from cv_bridge import CvBridge
import cv2  # OpenCV Python API


# End of Imports --------------------------------------------------------------


# Start of Callbacks and Functions --------------------------------------------


def image_callback(image_message, callback_args):
    """Callback to get the latest ROS Image and write it to the Redis server"""
    global redis_client

    if not redis_client.ping():  # Check if redis server is connected
        print("Redis server is not connected.")
        return

    try:
        cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
        # Convert OpenCV image > image binary string
        redis_client.set(callback_args, cv2.imencode(".jpg", cv_image)[1].tobytes())
    except Exception as e:
        print(e)


# End of Callbacks and Functions ----------------------------------------------


# Start of ROS node -----------------------------------------------------------

rospy.init_node("actor_gui_support", anonymous=True)

bridge = CvBridge()

# Start Redis Key-Value Store Server -----
rospy.sleep(2)  # Wait for redis server to start
global redis_client
redis_client = redis.Redis(host="localhost", port=6379, db=0, decode_responses=True)  # Connect to redis server
rospy.sleep(2)  # Sleep for 2 seconds before starting

if redis_client.ping():  # Check if redis server is connected
    print("Redis server is connected.")

# Define subscribers -----
rospy.Subscriber(rospy.get_param("gui_image_topic_1"), Image, image_callback, callback_args="image_1", queue_size=1)
rospy.Subscriber(rospy.get_param("gui_image_topic_2"), Image, image_callback, callback_args="image_2", queue_size=1)
rospy.Subscriber(rospy.get_param("gui_image_topic_3"), Image, image_callback, callback_args="image_3", queue_size=1)
rospy.Subscriber(rospy.get_param("gui_image_topic_4"), Image, image_callback, callback_args="image_4", queue_size=1)

rospy.loginfo("actor_gui_support node running.")
try:
    rospy.spin()
except rospy.ROSInterruptException:
    redis_client.close()
    pass

# End of ROS node -------------------------------------------------------------
