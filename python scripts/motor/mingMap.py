import math
from threading import Thread
from time import sleep
import serial
import sys
import glob
import queue
import copy
import re
import requests
import json
import threading


class Coordinate:
    """
    creates the class of x,y, theta corrdinate of the robot
    """

    def __init__(self, x=0, y=0, theta=90):
        self.x = x
        self.y = y
        self.theta = theta

    def print_coordinate(self):
        """
        :return:
        """
        print("coordinate: x: {0}, y: {1}, theta: {2}".format(self.x, self.y, self.theta))

    def return_pos_to_dict(self):
        return {"x": self.x, "y": self.y}

    def return_coordinate_to_dict(self):
        return {"x": self.x, "y": self.y, "angle": self.theta}


# Current Coordinates of the robot.
# Angle is kept between 180 and -180
current_coord = Coordinate()
# global used by functions

patrol_path = []

mode = {"Manual": 0, "Mapping": 1, "Patrol": 2}

current_mode = 0

ser = serial

current_patrol_point = 0

boundary_map = queue.Queue()

serial_message = queue.Queue()

DEBUG = 0

ARDUINO = 0

NO_CAR = 0

MODE_1_DISABLED = 1

MODE_2_DISABLED = 1

map_point = False

revert = False

mode_lock = threading.Lock()


# helper function to reset the current coordinates and parameter array.
def resetStates():
    """

    :return:
    """
    global current_coord
    global boundary_map
    global serial_message

    current_coord = Coordinate()
    boundary_map = queue.Queue(maxsize=1000)
    boundary_map.put(current_coord)
    serial_message = queue.Queue()


# Function call which tells the robot to go to a certain X,Y coordinate.
def motor_control(x, y):
    # calculates the angle based on the difference of the current location and the
    # destination X,Y
    global ser

    print("Driver: going to x: {0}, y: {1}".format(x, y))
    deltaX = x - current_coord.x
    deltaY = y - current_coord.y
    if deltaX == 0 and deltaY == 0:
        print("Robot Arrive at the position")
        return
    # calculates angle
    destinationAngle = math.degrees(math.atan2(deltaY, deltaX))
    angleToChange = destinationAngle - current_coord.theta

    delta_distance = (deltaX ** 2 + deltaY ** 2) ** (1 / 2)

    # update_current_coordinate(delta_distance, angleToChange)

    update_current_coordinate(0, angleToChange)
    update_current_coordinate(delta_distance, 0)

    if NO_CAR == 1:
        sleep(2)
        print("Arrived at x: {0}, y: {1}".format(x, y))
        return

    robot_control("turn", angleToChange)

    robot_control('forward', delta_distance)

    print("Driver: Arrived at x: {0}, y: {1}".format(x, y))


# this module only send data
# Thi function takes
def robot_control(command, arg1):
    """

    :param command:
    :param arg1:
    :param serial_port:
    :return:
    """
    # process the direction
    # process the time (if any)
    global message
    global current_mode
    global ser
    # ADding 0 at the front to work with patrol mode and maual mode

    if command == 'forward':
        message = "01" + ("1" if arg1 >= 0 else "0") + str(abs(arg1)).zfill(4)
    elif command == 'turn':
        message = "02" + ("1" if arg1 >= 0 else "0") + str(abs(arg1)).zfill(4)
    elif command == 'stop':
        # there are two way to stop: one is natural one is interrupt stop/
        message = "03" + str(0).zfill(5)

    # only forward and turn will reach here

    ser.write(message.encode())

    # wait for arduino feedback to send another serial message

    # TODO The program can get Stucked here!
    while ser.readline() != b'done\r\n':
        if current_mode == mode['Mapping']:
            break
        print("Waiting for feedback")

    print('Robot control message sent!')
    return


# url string, data string
def make_request(URL, http_mode='GET', data=None):
    if http_mode == 'GET':
        r = requests.get(URL)
        return json.loads(r.content.decode("utf-8"))
    else:
        requests.post(URL, data)
        return


# Check port
def portIsUsable(portName):
    try:
        serial.Serial(port=portName, timeout=10, baudrate=9600)
        return True
    except:
        return False


def serial_message_parser(one_message):
    """
    The format of the output message is:
    forward arg1(-9999 < arg1 < 9999)
    turn arg1 (-90 < arg1 < 90)
    Distance: < int > Angle: < int >
    :param one_message:
    :return:
    """
    received_coordinate = {}

    # remove the mapping tag
    # if one_message[:2] == "M_":
    #     one_message = one_message[2:]

    data = re.findall(r"[-+]?\d*\.\d+|\d+", str(one_message))
    # IF the data is corrupted
    if len(data) != 2:
        received_coordinate['cmd'] = "forward"
        received_coordinate['arg1'] = 0
        return received_coordinate

    # if the daa is good

    if float(data[0]) != 0:
        received_coordinate['cmd'] = "forward"
        received_coordinate['arg1'] = float(data[0])
    else:
        received_coordinate['cmd'] = "turn"
        received_coordinate['arg1'] = float(data[1])

    return received_coordinate


# helper function to detect distance between 2 points
def distance(x1, y1, x2, y2):
    """

    :param x1:
    :param y1:
    :param x2:
    :param y2:
    :return:
    """
    deltaX = x2 - x1
    deltaY = y2 - y1
    return (deltaX ** 2 + deltaY ** 2) ** (1 / 2)


def update_current_coordinate_from_serial():
    """
    :return:
    False if dumped the serial message and add a point with no turn
    True of add a way point with angle change
    """
    global current_coord
    global serial_message

    delta_distance = 0
    delta_angle = 0
    # print("Start")

    while not serial_message.empty():

        current_message = serial_message.get()
        serial_message.task_done()

        # print(current_message)

        if current_message[:1] == "M":
            # if it's a map point message, remove the map point tag and put it back
            serial_message.put(current_message[2:])
            # print("Message reverted----!")
            break

        parsed_message = serial_message_parser(current_message)
        # print(parsed_message)

        if parsed_message["cmd"] == "forward":
            delta_distance += parsed_message['arg1']

        elif parsed_message["cmd"] == "turn":
            delta_angle += parsed_message['arg1']
            # break

    print("d_distance = " + str(delta_distance) + " d_angle = " + str(delta_angle))

    update_current_coordinate(delta_distance, delta_angle)

    current_coord.print_coordinate()

    print("End")

    return


def update_current_coordinate(delta_distance, delta_angle):
    """
    Function call which updates Current_Coord
    Left angle == POSITIVE, Right angle == NEGATIVE
    assumes Arduino gives a continuous distance while it's going 1
    direction, otherwise returns 0. Angle gives updated angle.
    :param delta_angle:
    :param delta_distance:
    :return:
    """
    global current_coord

    # the current heading is where the distance headed, so we update the angle in the end

    current_coord.x += round((delta_distance * math.cos(math.radians(current_coord.theta))), 2)
    current_coord.y += round((delta_distance * math.sin(math.radians(current_coord.theta))), 2)

    # avoid inputs that are larger than 360
    delta_angle = delta_angle % 360

    # keeps angle between 0 and 360
    current_coord.theta += round(delta_angle, 2)

    # clamp theat between -180 and 180 for the atan2
    if current_coord.theta > 180:
        current_coord.theta -= 360
    if current_coord.theta < -180:
        current_coord.theta += 360



"""
This part are the code for modes
Contains: 
mapping mode
patrol mode
manual mode
"""


# Defining Mapping State;
# Function for the mapping; will only add corner points to the array
# **Unsure how the distance and angle will be updated from the robot.
def mapping_mode():
    # de queue
    """
    # :return:
    # False if dumped the serial message and add a point with no turn
    # True of add a way point with angle change
    # """

    global boundary_map
    global map_point
    delta_distance = 0
    delta_angle = 0
    isAngleChanged = 0

    parsed_message = serial_message_parser(ser.readline().decode("utf-8"))
    print(parsed_message)
    current_coord.print_coordinate()
    if parsed_message['cmd'] == "forward":
        delta_distance = parsed_message['arg1']
    if parsed_message['cmd'] == "turn":
        delta_angle = parsed_message['arg1']
    update_current_coordinate(delta_distance, delta_angle)

    if delta_angle != 0:
        if not isAngleChanged:
            boundary_map.put(current_coord)
            print("corrdinate send")
            make_request("http://griin.today/API/boundaries", "POST", current_coord.return_pos_to_dict())
            isAngleChanged = 1

    if isAngleChanged and delta_distance != 0:
        isAngleChanged = 0
    # delta_distance = 0
    # delta_angle = 0

    # Use map point to signal that there is a new point to be added. Otherwise the main thread skip everything
    # if map_point:
    #     print("Add New Way point!")
    #
    #     # parsed_message = serial_message_parser(ser.readline().decode("utf-8"))
    #     # delta_distance += parsed_message['arg1']
    #     # delta_angle += parsed_message['arg1']
    #     # update_current_coordinate(delta_distance, delta_angle)
    #
    #     update_current_coordinate_from_serial()
    #     boundary_map.put(current_coord)
    #     if DEBUG != 1:
    #         make_request("http://griin.today/API/boundaries", "POST", current_coord.return_pos_to_dict())
    #     map_point = False

    # global current_coord
    # global serial_message
    #
    # delta_distance = 0
    # delta_angle = 0
    # print("Start")
    #
    # while not serial_message.empty():
    #
    #     current_message = serial_message.get()
    #     serial_message.task_done()
    #
    #     print(current_message)
    #
    #     if current_message[:2] == "M_":
    #         boundary_map.put(current_coord)
    #
    #     parsed_message = serial_message_parser(current_message)
    #
    #     print(parsed_message)
    #
    #     if parsed_message["cmd"] == "forward":
    #         delta_distance = parsed_message['arg1']
    #
    #     elif parsed_message["cmd"] == "turn":
    #         delta_angle = parsed_message['arg1']
    #
    #     print("d_distance = " + str(delta_distance) + " d_angle = " + str(delta_angle))
    #
    #     update_current_coordinate(delta_distance, delta_angle)
    #
    #     current_coord.print_coordinate()
    #
    # print("End")
    #
    #
    # make_request("http://griin.today/API/boundaries", "POST", current_coord.return_pos_to_dict())
    #


# prototype mode for patrol
# Will go directly through the array of points given by user.
def patrol_mode():
    """
     The following are codes for threads
     this loop will keep looping, until
    """

    # if the queue is updated
    global current_patrol_point
    global revert

    if len(patrol_path) == 0:
        print("Map Needed or No patrol path loaded")
        return
    print("Patrol point %d" % current_patrol_point)
    motor_control(patrol_path[current_patrol_point]['x'], patrol_path[current_patrol_point]['y'])
    if revert:
        current_patrol_point -= 1
        if current_patrol_point < 0:
            current_patrol_point = 1
            revert = False
    else:
        current_patrol_point += 1
        if current_patrol_point >= len(patrol_path):
            current_patrol_point = len(patrol_path) - 2
            revert = True
    return


"""
Thread functions
"""


def robot_mode_main():
    global current_mode
    global ser
    global current_patrol_point
    global boundary_maps
    global patrol_path
    global revert

    while True:

        if DEBUG != 1:
            temp = make_request("http://griin.today/API/current_mode", "GET")['current_mode']
            # update the current coordinate

            if temp != current_mode:
                change = 1
            else:
                change = 0

            current_mode = temp
        else:
            current_mode = 1
            change = 0

        print("Current Mode: " + str(current_mode))

        if current_mode == mode['Mapping']:
            if change == 1:
                resetStates()
                ser.write("1000000".encode())
                print("--------------initalized mapping!")
                make_request("http://griin.today/API/boundaries", "POST", {"x":0, "y":0})
            mapping_mode()

        elif current_mode == mode['Patrol']:
            if change == 1:
                if DEBUG != 1:
                    global patrol_path
                    new_path = make_request("http://griin.today/API/patrol_path", "GET")['patrol_path']

                    # if the len is not the same, that meaans it's the new boundary
                    if len(new_path) != len(patrol_path):
                        patrol_path = new_path
                        current_patrol_point = 0
                        revert = False

                else:
                    patrol_path = {}

            # if not serial_message.empty():
            #     print("Mapping points are not completed")
            #     finish_mapping()

            patrol_mode()
            current_coord.print_coordinate()
            make_request("http://griin.today/API/current_location", "POST", current_coord.return_coordinate_to_dict())

        elif current_mode == mode['Manual']:
            # if change == 1 and not serial_message.empty():
            #     finish_mapping()

            if DEBUG != 1:
                location = make_request("http://griin.today/API/current_target", "GET")
                print(location)
                try:
                    float(location['x'])
                    float(location['y'])
                except:
                    print("Invalid Location")
                    continue
            else:
                location = {"x": 0, "y": 0}

            print("Going to: " + str(int(location['x'])) + " " + str(int(location['y'])))

            if abs(location['x'] - current_coord.x) < 3 and abs(location['y'] - current_coord.y) < 3:
                pass
            else:
                motor_control(float(location['x']), float(location['y']))
            # motor_control(float(location['x']), float(location['y']))

            make_request("http://griin.today/API/current_location", "POST", current_coord.return_coordinate_to_dict())
            current_coord.print_coordinate()
        else:
            assert "Mode is not defined!"
        # sleep(2)


if __name__ == "__main__":
    print("Program init")
    """
    """
    port = 'COM13'
    """
    """
    while not portIsUsable(port):
        print("Try to connect to " + port)
        sleep(2)
        if port == 'COM13':
            port = 'COM15'
        else:
            port = '/dev/ttyACM0'

    ser = serial.Serial(port=port, timeout=10, baudrate=9600)

    print("Arduino connected")
    # TODO: the hand shake process to make sure it's connected
    sleep(2)
    print("Start controlling")
    print("Debug mode is " + str(DEBUG))
    print("Arduino mode is " + str(ARDUINO))
    print("No car mode is " + str(NO_CAR))

    robot_mode_main()

    while True:
        pass
