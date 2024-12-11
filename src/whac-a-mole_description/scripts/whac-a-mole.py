#!/usr/bin/env python3

import rospy
import random
import threading
import sys
import select
import tty
import termios
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState

# Global variables to store the current positions of the joints
current_positions = {
    "slider_joint_1": 0.4,
    "slider_joint_2": 0.4,
    "slider_joint_3": 0.4,
    "slider_joint_4": 0.4,
    "button_joint_1": -0.5,
    "button_joint_2": -0.5,
    "button_joint_3": -0.5,
    "button_joint_4": -0.5
}

# Target positions for the joints
target_positions = current_positions.copy()

# Variable to keep track of score
score = 0

# Lock for thread-safe operations
score_lock = threading.Lock()

# Flags to track if a button has already been hit
button_hit_flags = {
    "button_1": True,
    "button_2": True,
    "button_3": True,
    "button_4": True
}

# Flag for nightmare mode
nightmare_mode = False


def slider_joint_callback(msg, joint_name):
    current_positions[joint_name] = msg.process_value


def button_joint_callback(msg, joint_name):
    current_positions[joint_name] = msg.process_value


def getKey():
    original_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
    return key


def user_input_thread():
    global score, nightmare_mode
    while not rospy.is_shutdown():
        try:
            user_input = getKey()
            if not user_input:
                continue
            if user_input == "\x03":  # Ctrl+C
                break
            if user_input == "`":
                nightmare_mode = True
                print("Nightmare mode activated! Good luck, nerd.")
                continue
            button_num = int(user_input)
            if 1 <= button_num <= 4:
                button_joint = f"button_joint_{button_num}"
                with score_lock:
                    if current_positions[button_joint] >= -0.4:
                        target_positions[button_joint] = -0.5
                        score += 1
                        button_hit_flags[f"button_{button_num}"] = True
                        print(f"Button {button_num} pushed down by user. "
                              f"Score: {score}")
                    else:
                        score -= 1
                        print(f"Button {button_num} is already down. "
                              f"Score: {score}")
            else:
                print("Please enter a valid number between 1 and 4.")
        except ValueError:
            print("Invalid input. Please enter a number between 1 and 4.")


def whac_a_mole():
    global score
    rospy.init_node("move_linear_actuators", anonymous=True)

    # Publishers for sliders and buttons
    slider_pubs = [
        rospy.Publisher(f"/slider_joint_{i}_position_controller/command",
                        Float64, queue_size=10)
        for i in range(1, 5)
    ]
    button_pubs = [
        rospy.Publisher(f"/button_joint_{i}_position_controller/command",
                        Float64, queue_size=10)
        for i in range(1, 5)
    ]

    # Subscribers for joint states
    for i in range(1, 5):
        rospy.Subscriber(
            f"/slider_joint_{i}_position_controller/state",
            JointControllerState,
            slider_joint_callback,
            callback_args=f"slider_joint_{i}"
        )
        rospy.Subscriber(
            f"/button_joint_{i}_position_controller/state",
            JointControllerState,
            button_joint_callback,
            callback_args=f"button_joint_{i}"
        )

    # Start the user input thread
    threading.Thread(target=user_input_thread, daemon=True).start()

    rate = rospy.Rate(10)  # 10 Hz
    last_pop_time = rospy.get_time()
    next_pop_interval = 5  # Initial interval for popping up buttons

    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        elapsed_time = current_time - last_pop_time

        # Determine the difficulty level based on the score
        with score_lock:
            if nightmare_mode:
                threshold = -0.35  # Nightmare mode
            elif score <= 10:
                threshold = -0.1  # Easy
            elif score <= 20:
                threshold = -0.2  # Medium
            else:
                threshold = -0.25  # Hard

        # Randomly pop up a button every 1-3 seconds
        if elapsed_time >= next_pop_interval:
            button_to_pop = random.randint(1, 4)
            button_joint = f"button_joint_{button_to_pop}"
            if current_positions[button_joint] <= -0.1 and \
               not button_hit_flags[f"button_{button_to_pop}"]:
                # Pop up the button
                target_positions[button_joint] = 0.0
                # Reset the robot hit flag
                button_hit_flags[f"button_{button_to_pop}"] = False
            last_pop_time = current_time
            next_pop_interval = random.uniform(1, 3)

        # Check each button and control the corresponding slider
        for i in range(1, 5):
            button_joint = f"button_joint_{i}"
            slider_joint = f"slider_joint_{i}"
            button_flag = f"button_{i}"

            # If the button is up (popped)
            if current_positions[button_joint] >= threshold:
                # Move the slider down to hit the button
                target_positions[slider_joint] = -1.6
            else:
                # Keep the slider up
                target_positions[slider_joint] = 0.4

            # If the slider is up (resting position)
            if current_positions[slider_joint] >= 0.1:
                # Reset flag when slider is up
                button_hit_flags[button_flag] = False

            # If the slider is down (has hit the button)
            if current_positions[slider_joint] <= -1.5:
                # Reset the button to down position
                target_positions[button_joint] = -0.5
                with score_lock:
                    if not button_hit_flags[button_flag]:
                        score -= 1
                        print(f"Robot hit Button {i}. Score: {score}")
                        # Prevent multiple decrements
                        button_hit_flags[button_flag] = True

        # Publish button positions
        for i, pub in enumerate(button_pubs, start=1):
            pub.publish(target_positions[f"button_joint_{i}"])

        # Publish slider positions
        for i, pub in enumerate(slider_pubs, start=1):
            pub.publish(target_positions[f"slider_joint_{i}"])

        rate.sleep()


if __name__ == "__main__":
    try:
        whac_a_mole()
    except rospy.ROSInterruptException:
        pass
