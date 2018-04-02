import math
from threading import Thread
from time import sleep
import serial
import sys
import glob
import queue
import copy
import matplotlib
import re
import requests
import json


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

    def return_dict_for_server(self):
        return {"x": self.x, "y": self.y}


# Current Coordinates of the robot.
# Angle is kept between 180 and -180
current_coord = Coordinate()
# global used by functions

patrol_path = []

mode = {"Manual": 0, "Mapping": 1, "Patrol": 2}

current_mode = 0

ser = serial

map_point_acquired = queue.Queue()

new_path_acquired = False

current_patrol_point = 0

boundary_map = queue.Queue()

serial_message = queue.Queue()

DEBUG = 0


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
    serial_message = queue.Queue(maxsize=1000)


# Function call which tells the robot to go to a certain X,Y coordinate.
def motor_control(x, y, serial_port):
    # calculates the angle based on the difference of the current location and the
    # destination X,Y

    deltaX = x - current_coord.x
    deltaY = y - current_coord.y
    # calculates angle
    destinationAngle = math.degrees(math.atan2(deltaY / deltaX))
    angleToChange = destinationAngle - current_coord.theta

    robot_control("turn", angleToChange, serial_port)

    delta_distance = (deltaX ** 2 + deltaY ** 2) ** (1 / 2)

    robot_control('forward', delta_distance, serial_port)


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


# this module only send data
# Thi function takes
def robot_control(command, arg1, serial_port):
    """

    :param command:
    :param arg1:
    :param serial_port:
    :return:
    """
    # process the direction
    # process the time (if any)
    global message
    if command == 'forward':
        message = "1" + ("1" if arg1 >= 0 else "0") + str(abs(arg1)).zfill(4)
    elif command == 'turn':
        message = "2" + ("1" if arg1 >= 0 else "0") + str(abs(arg1)).zfill(4)
    elif command == 'stop':
        # there are two way to stop: one is natural one is interrupt stop/
        message = "3" + str(0).zfill(5)

    # only forward and turn will reach here

    serial_port.write(message.encode())

    # wait for arduino feedback to send another serial message
    while serial_port.readline() != b'done\r\n':
        print("Waiting for feedback")

    print('Robot control message sent!')
    return 1


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
    if one_message[:2] == "M_":
        one_message = one_message[2:]

    data = re.findall(r"[-+]?\d*\.\d+|\d+", one_message)
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

    delta_angle = delta_angle % 360
    # keeps angle between 0 and 360
    current_coord.theta += round(delta_angle, 2)
    if current_coord.theta > 180:
        current_coord.theta -= 360
    if current_coord.theta < -180:
        current_coord.theta += 360


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
    print("Start")

    while not serial_message.empty():

        current_message = serial_message.get()
        serial_message.task_done()

        print(current_message)

        if current_message[:2] == "M_":
            serial_message.put(current_message[2:])
            break

        parsed_message = serial_message_parser(current_message)
        print(parsed_message)

        if parsed_message["cmd"] == "forward":
            delta_distance += parsed_message['arg1']

        elif parsed_message["cmd"] == "turn":
            delta_angle += parsed_message['arg1']

    print("d_distance = " + str(delta_distance) + " d_angle = " + str(delta_angle))

    update_current_coordinate(delta_distance, delta_angle)

    current_coord.print_coordinate()

    print("End")

    return


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
    global boundary_map
    # de queue
    update_current_coordinate_from_serial()
    boundary_map.put(current_coord)
    if DEBUG != 1:
        make_request("http://griin.today/API/boundaries", "POST", current_coord.return_dict_for_server())


# prototype mode for patrol
# Will go directly through the array of points given by user.
def patrol_mode():
    """
     The following are codes for threads
     this loop will keep looping, until
    """

    # if the queue is updated
    global current_patrol_point

    if new_path_acquired:
        copy_list = copy.copy(patrol_path)
        while not copy_list:
            patrol_path.append(copy_list.pop())
            current_patrol_point = 0
    elif current_patrol_point != len(patrol_path):

        motor_control(patrol_path[current_patrol_point].x, patrol_path[current_patrol_point].y, ser)
        current_patrol_point += 1

"""
Thread functions
"""


def robot_mode_main():
    global current_mode
    global ser
    change = 0
    while True:
        if DEBUG != 1:
            temp = make_request("http://griin.today/API/current_mode", "GET")['current_mode']
            if temp != current_mode:
                change = 1
            else:
                change = 0
            current_mode = temp

        if DEBUG == 1:
            current_mode = 1
            change = 0
        print("Current Mode: " + str(current_mode))

        if current_mode == mode['Mapping']:
            if change == 1:
                resetStates()
                ser.write("1000000".encode())

            mapping_mode()

        elif current_mode == mode['Patrol']:
            if not serial_message.empty():
                update_current_coordinate_from_serial()
            else:
                patrol_mode()




def serial_read():
    """
    This is the thraed to read message from serial port when it's mapping mode
    :param serial_port:
    :param message_queue:
    """
    global current_mode
    global ser
    global serial_message

    last_message = ""
    current_message = ""
    while True:
        # only add message if it's mapping mode
        if current_mode == mode['Mapping']:
            current_message = ser.readline().decode("utf-8")

            if serial_message_parser(current_message)['cmd'] == "forward" and serial_message_parser(last_message)[
                'cmd'] == "turn":
                current_message = "M_" + current_message

            serial_message.put(current_message)

            # if angle is detected

            print("thread serial read ------------ " + current_message + str(serial_message.qsize()))

            # change the signal from turn to forward, then update the map
            last_message = current_message


if __name__ == "__main__":
    print("Program init")

    port = 'COM15'
    while not portIsUsable(port):
        print("Try to connect to " + port)
        sleep(2)
    ser = serial.Serial(port=port, timeout=10, baudrate=9600)
    print("Arduino connected")
    # TODO: the hand shake process to make sure it's connected
    sleep(2)
    print("Start controlling")

    # # the first thread is the main one that will send
    main_thread = Thread(target=robot_mode_main, args=())

    serial_read_thread = Thread(target=serial_read, args=())

    main_thread.start()
    serial_read_thread.start()

    # TODO image!!

    main_thread.join()
    serial_read_thread.join()

    while True:
        pass

    ser.close()
    print("Closed " + port)
