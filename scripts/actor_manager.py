import rospy

from actor_ros.actor import Actor

if __name__ == "__main__":
    rospy.init_node("actor")
    actor = Actor()
    rospy.spin()
