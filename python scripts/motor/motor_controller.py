
# the current state of the robot
# if we use accel and gyro, we will get this from them.
current_state = {'x':0, 'y':0, 'theta':0}

def robot_motor_driver(step_left, step_right):

    return 1

#
def robot_motor_turn(radis):
    return 1

def robot_motor_forward(distance):

    return 1

# the absoulte position and angling
def motor_control(x, y, theta):

    robot_motor_turn(theta - current_state[theta])
    # calculate
    distance = ((x - current_state[x]) ** 2 + (y-current_state[y]) ** 2) ** (1/2)

    # perfer to cut the distance to piece, so that we can run the obstacle avoidence at the same time.
    robot_motor_forward(distance)
    return 1