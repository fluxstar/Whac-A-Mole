#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def move_actuators():
    rospy.init_node('move_linear_actuators', anonymous=True)
    
    # Publishers for each linear actuator
    pub1 = rospy.Publisher('/slider_joint_1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/slider_joint_2_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/slider_joint_3_position_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/slider_joint_4_position_controller/command', Float64, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    position = 0.0
    direction = 1  # 1 for increasing, -1 for decreasing
    rospy.sleep(1)  # Wait for the actuators to be ready
    
    while not rospy.is_shutdown():
        # Publish position to each actuator
        pub1.publish(position)
        pub2.publish(position)
        pub3.publish(position)
        pub4.publish(position)
        print(position)
        
        # Update position
        position += direction * 0.01
        if position >= 0.3:
            position = 0.3
            direction = -1
        elif position <= 0.0:
            position = 0.0
            direction = 1
        
        rate.sleep()

if __name__ == '__main__':
    try:
        move_actuators()
    except rospy.ROSInterruptException:
        pass