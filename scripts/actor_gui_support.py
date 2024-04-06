#!/usr/bin/env python3

"""
ROS Node for supporting ROS to GUI communication.

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
    global redis

    try:
        if not redis.ping():  # Check if redis server is connected
            cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
            serialized_image = cv2.imencode(".jpg", cv_image)[1].tobytes()
            redis.set(callback_args, serialized_image)

    except Exception as e:
        print(e)


# End of Callbacks and Functions ----------------------------------------------


# Start of ROS node -----------------------------------------------------------
rospy.init_node("actor_gui_support", anonymous=True)

bridge = CvBridge()

# Start Redis Key-Value Store Server -----
rospy.sleep(2)  # Wait for redis server to start
redis = redis.Redis(host="localhost", port=6379, db=0, decode_responses=True)  # Connect to redis server
rospy.sleep(2)  # Sleep for 2 seconds before starting

# Define subscribers -----
rospy.Subscriber(rospy.get_param("gui_image_topic_1"), Image, image_callback, callback_args="image_1", queue_size=1)

rospy.loginfo("actor_gui_support node running.")
try:
    rospy.spin()
except rospy.ROSInterruptException:
    redis.close()
    pass
# End of ROS node -------------------------------------------------------------
