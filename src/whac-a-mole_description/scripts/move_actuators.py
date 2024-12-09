#!/usr/bin/env python3

import rospy
import math
import random
from std_msgs.msg import Float64

def move_actuators():
    rospy.init_node("move_linear_actuators", anonymous=True)
    
    # Publishers for each linear actuator
    pub1 = rospy.Publisher("/slider_joint_1_position_controller/command", Float64, queue_size=10)
    pub2 = rospy.Publisher("/slider_joint_2_position_controller/command", Float64, queue_size=10)
    pub3 = rospy.Publisher("/slider_joint_3_position_controller/command", Float64, queue_size=10)
    pub4 = rospy.Publisher("/slider_joint_4_position_controller/command", Float64, queue_size=10)

    # Publishers for each button
    butt1 = rospy.Publisher("/button_joint_1_position_controller/command", Float64, queue_size=10)
    butt2 = rospy.Publisher("/button_joint_2_position_controller/command", Float64, queue_size=10)
    butt3 = rospy.Publisher("/button_joint_3_position_controller/command", Float64, queue_size=10)
    butt4 = rospy.Publisher("/button_joint_4_position_controller/command", Float64, queue_size=10)
    
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

        butt_pos_1 = -0.25 * math.sin(elapsed_time + math.pi / 2) - 0.25
        butt_pos_2 = -0.25 * math.sin(elapsed_time + math.pi) - 0.25
        butt_pos_3 = -0.25 * math.sin(elapsed_time + 3 * math.pi / 2) - 0.25
        butt_pos_4 = -0.25 * math.sin(elapsed_time) - 0.25
        
        # Publish position to each actuator
        pub1.publish(position1)
        pub2.publish(position2)
        pub3.publish(position3)
        pub4.publish(position4)
        
        butt1.publish(butt_pos_1)
        butt2.publish(butt_pos_2)
        butt3.publish(butt_pos_3)
        butt4.publish(butt_pos_4)

        rate.sleep()

if __name__ == "__main__":
    try:
        move_actuators()
    except rospy.ROSInterruptException:
        pass