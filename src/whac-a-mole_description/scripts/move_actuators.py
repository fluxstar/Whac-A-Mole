#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64

def move_actuators():
    rospy.init_node('move_linear_actuators', anonymous=True)
    
    # Publishers for each linear actuator
    pub1 = rospy.Publisher('/slider_joint_1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/slider_joint_2_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/slider_joint_3_position_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/slider_joint_4_position_controller/command', Float64, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.get_time()
    
    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        elapsed_time = current_time - start_time
        
        # Calculate positions using sine wave with phase shifts
        position1 = math.sin(elapsed_time) - 0.6
        position2 = math.sin(elapsed_time + math.pi / 2)  - 0.6
        position3 = math.sin(elapsed_time + math.pi)  - 0.6
        position4 = math.sin(elapsed_time + 3 * math.pi / 2)  - 0.6
        
        # Publish position to each actuator
        pub1.publish(position1)
        pub2.publish(position2)
        pub3.publish(position3)
        pub4.publish(position4)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        move_actuators()
    except rospy.ROSInterruptException:
        pass