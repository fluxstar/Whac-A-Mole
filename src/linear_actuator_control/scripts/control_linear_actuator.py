#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import math

def control_linear_actuator():
    rospy.init_node('linear_actuator_controller', anonymous=True)
    
    # Create publisher for the slider joint
    pub = rospy.Publisher('/linear_actuator/slider_joint_position_controller/command', 
                         Float64, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        # Move between 0 and 0.2 meters
        position = 0.2 * math.sin(rospy.get_time())
        pub.publish(Float64(position))
        rate.sleep()

if __name__ == '__main__':
    try:
        control_linear_actuator()
    except rospy.ROSInterruptException:
        pass