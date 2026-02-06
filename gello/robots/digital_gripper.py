import urx
import time

# Connect to UR robot
rob = urx.Robot("192.168.20.101")  # Replace with your UR controller IP

for i in range(2):

    # Open gripper (set DO[0] = True, DO[2] = False)
    rob.set_digital_out(2, False)
    rob.set_digital_out(0, True)
    time.sleep(2)
    print("Open")

    # Close gripper (set DO[2] = True, DO[0] = False)
    rob.set_digital_out(0, False)
    rob.set_digital_out(2, True)
    time.sleep(2)
    print("close")

# Release both (optional)
rob.set_digital_out(2, False)
rob.set_digital_out(0, True)

rob.close()