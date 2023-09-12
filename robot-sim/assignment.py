from __future__ import print_function

import time
from sr.robot import *
import random

GO_TO_SILVER = "go-to-silver"
GRAB_TOKEN = "grab-token"
UPDATE_SILVER_LIST = "update-silver-list"
GO_TO_GOLD = "go-to-gold"
RELEASE_SILVER = "release-token"
UPDATE_GOLD_LIST = "updatee-gold-list"
FINISH = "finish"
""" states of the finite state machine """

a_th = 2.0
""" float: Threshold for the control of the orientation """

d_th_grab = 0.4
""" float: Threshold for the control of the linear distance for grabbing a token """

d_th_release = 0.5
""" float: Threshold for the control of the linear distance for releasing a token """

T = 0.05
""" float: Sampling period in seconds for executing driving actions"""

speed = 100.0
""" float: Percentage of power for the robot wheels"""


def rotate(speed, seconds):
    """
    Function to rotate counter-clockwise the robot
    around its vertical axis

    Args:
          speed (float): speed of the wheels
        seconds (float): duration in seconds the robot rotates
    """
    R.motors[0].m0.power = -speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0


def turnRight(speed, seconds):
    """
    Function to rotate clockwise the robot
    around its right side

    Args:
        speed (float): speed of the wheels
      seconds (float): duration in seconds the robot rotates
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = 0
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0


def turnLeft(speed, seconds):
    """
    Function to rotate counter-clockwise the robot
    around its left side

    Args:
        speed (float): speed of the wheels
      seconds (float): duration in seconds the robot rotates
    """
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0


def drive(speed, seconds):
    """
    Function for setting a linear velocity

    Args:
        speed (float): speed of the wheels in percentage
	  seconds (float): time in seconds the motors are active
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0


def retreat_from_token(speed = 100):
    """
    Function to retreat the robot after releasing a silver token

    Args:
        speed (float): speed of the wheels in percentage
    """
    kd = 20.0
    drive(-speed, kd/speed)


def stop():
    """
    Stops the robot for T seconds
    T: global variable defining sample pariod of the program 
    """
    drive(0, T)


def find_closest_free_token_per_type(token_type, exclude):
    """
    Function to look for token of token_type not present in
    the exclude list
    If not any token of type token_type is detected, it returns
    (-1, -1, 0)

    Args:
        token_type (string): token type to look for
                             currently supported token types:
                                MARKER_TOKEN_GOLD = "gold-token"
                                MARKER_TOKEN_SILVER = "silver-token"
             exclude (list): list containing all the token code
                             to not consider while searching

    Returns:
         dist (float): distance between the robot and the closest token
                       of type token_type (-1 if not any token of type
                       token_type is detected)
        rot_y (float): angle in degree between the robot and the closest
                       token of type token_type (-1 if not any token of type
                       token_type is detected)
           code (int): target token code (0 if not any token of type
                       token_type)
    """
    dist = 100.0
    # choose randomly which direction to look for
    direction = 1 if random.randint(0,1) else -1
    # research circular sector every of 30 degrees
    # for smoother behaviour
    for count in range(12):
        for token in R.see():
            # filter token list with respect type and exclusion list
            if (token.info.marker_type == token_type and not(token.info.code in exclude)):
                if (token.dist < dist):
                    dist = token.dist
                    rot_y = token.rot_y
                    code = token.info.code

        if (dist != 100): 
            return dist, rot_y, code
        else:
            # not found any token. Turn for checking
            # the next circular sector
            dist = 100
            rotate(direction*speed,2*T)
            stop()

    # not found any token
    return (-1,-1, 0)


def find_closest_silver_in_sight(exclude):
    """
    Function to look for the closest silver token not present in
    the exclude list
    If not any token of type token_type is detected, it returns
    (-1, -1, 0)

    Args:
        exclude (list): list containing all the token code
                        to not consider while searching

    Returns:
         dist (float): distance between the robot and the closest silver
                       token (-1 if not any token of type silver is detected)
        rot_y (float): angle in degree between the robot and the closest
                       silver token (-1 if not any silver token is detected)
           code (int): target token code (0 if not any token of type silver
                       is detected)
    """
    return find_closest_free_token_per_type(MARKER_TOKEN_SILVER, exclude)


def find_closest_gold_in_sight(exclude):
    """
    Function to look for the closest golden token not present in
    the exclude list.
    If not any token of type token_type is detected, it returns
    (-1, -1, 0)

    Args:
        exclude (list): list containing all the token code
                        to not consider while searching

    Returns:
         dist (float): distance between the robot and the closest golden
                       token (-1 if not any golden token is detected)
        rot_y (float): angle in degree between the robot and the closest
                       golden token (-1 if not any golden token is detected)
           code (int): target token code (0 if not any golden token is detected)
    """
    return find_closest_free_token_per_type(MARKER_TOKEN_GOLD, exclude)


def drive_to_silver(exclude, speed = 100):
    """
    Function to drive the robot close to a silver token

    Args:
               exclude (list): list containing all the token code
                               to not consider while searching
        speed [100.0] (float): speed the robot uses to drives toward the target token

    Returns:
        code (int): code of the reached silver token
    """
    return drive_to_closest_token_with_type(MARKER_TOKEN_SILVER, exclude, d_th_grab, a_th, speed)


def drive_to_gold(exclude, speed = 100):
    """
    Function to drive the robot close to a golden token

    Args:
               exclude (list): list containing all the token code
                               to not consider while searching
        speed [100.0] (float): speed the robot uses to drives toward the target token

    Returns:
        code (int): code of the reached golden token
    """
    return drive_to_closest_token_with_type(MARKER_TOKEN_GOLD, exclude, d_th_release, a_th, speed)


def drive_to_closest_token_with_type(token_type, exclude, d_th, a_th, speed = 100):
    """
    Function to drive the robot to the closest token of type token_type

    Args:
          token_type (string): token type to look for
                               currently supported token types:
                                   - MARKER_TOKEN_GOLD = "gold-token"
                                   - MARKER_TOKEN_SILVER = "silver-token"
               exclude (list): list containing all the token code
                               to not consider while searching
                 d_th (float): linear distance threshold after which the robot
                               is considered close enough to the target token
                 a_th (float): orientation distance threshold after which the
                               robot is considered close enough to the target token
        speed [100.0] (float): speed the robot uses to drives toward the target token

    Returns:
        code (int): code of the reached target token
    """
    kr = 3
    while True:
        # get distances and code of the closest eligible target token
        dist, rot_y, code = find_closest_free_token_per_type(token_type, exclude)

        # not any available target token
        if (dist == -1):
            return -1
        else:
            # check orientation threshold
            if (abs(rot_y) > a_th):
                # orienting the robot
                speedr = min(abs(rot_y / kr / T), 100)
                print("turning at speed: " + str(speedr))
                if (rot_y > 0):
                    turnRight(speedr, T)
                else:
                    turnLeft(speedr, T)

            # check linear distance threshold
            if(dist > d_th):
                # closing to the token
                print("driving")
                drive(speed,T)

            # check robot within both thresholds
            if(dist < d_th and rot_y < a_th):
                # token code within threshold
                return code

#----------------------------------
#           MAIN PROGRAM
#----------------------------------

# instance of the robot
R = Robot()

# lists for keeping track of the already sorted tokens
silver_token_done = []
gold_token_done = []

# token code initial values
code_silver = 0
code_gold = 0

# starting state of the finite state machine
state = GO_TO_SILVER

while 1:

    if (state == GO_TO_SILVER):
        code_silver = drive_to_silver(silver_token_done)
        if code_silver == -1:
            print("not any free silver token in sight")
            state = FINISH
        else:
            print("silver token reached")
            stop()
            state = GRAB_TOKEN

    elif (state == GRAB_TOKEN):
        if (R.grab()):
            print("grabbed")
            state = UPDATE_SILVER_LIST
        else:
            print("something went wrong while grabbing token code " + str(code_silver))
            state = FINISH

    elif (state == UPDATE_SILVER_LIST):
        silver_token_done.append(code_silver)
        state = GO_TO_GOLD

    elif (state == GO_TO_GOLD):
        code_gold = drive_to_gold(gold_token_done)
        if code_gold == -1:
            print("lost golden token on the way")
            state = FINISH
        else:
            print("gold token reached")
            state = RELEASE_SILVER

    elif (state == RELEASE_SILVER):
        if R.release():
            print("Silver token released")
            retreat_from_token()
            state = UPDATE_GOLD_LIST
        else:
            print("something happened while releasing the golden token")
            state = FINISH

    elif (state == UPDATE_GOLD_LIST):
        gold_token_done.append(code_gold)
        state = GO_TO_SILVER

    elif (state == FINISH):
        print("Done")
        exit()
