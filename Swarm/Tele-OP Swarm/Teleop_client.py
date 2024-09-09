import socket
import time
from pymavlink import mavutil

# Mavlink_connection
the_connection = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

# Wait for the first heartbeat
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

# Client configuration
HOST = '192.168.1.82'  # Server IP address
PORT = 12345        # Server port

# Arm the drone
the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
     0,
     1, # 0:disarm, 1:arm
     0,
     0 , 0, 0, 0, 0)
     
time.sleep(1)

# Send takeoff command
the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0 , 0, 0, 0,
    10) # desired altitude
the_connection.recv_match(type='COMMAND_ACK', blocking=True)


def move_forward():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10,the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
            int(0b110111000111),
            0, 0, 0, # position
            1, 0, 0, # velocity
            0, 0, 0, # acceleration
            0, 0))   # yaw, yaw_speed

def move_backward():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        int(0b010111000111),
        0, 0, 0,  # position
        -1, 0, 0, # velocity
        0, 0, 0,  # acceleration
        0, 0))    # yaw, yaw_speed

def turn_left():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
        int(0b110111000111), 
        0, 0, 0,  # position
        0, -1, 0, # velocity
        0, 0, 0,  # acceleration
        0, 0))    # yaw, yaw_speed

def turn_right():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, 
        int(0b110111000111), 
        0, 0, 0, 
        0, 1, 0, 
        0, 0, 0, 
        0, 0))

def move_up():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
            int(0b110111000111),
            0, 0, 0,  # position
            0, 0, 1, # velocity
            0, 0, 0,  # acceleration
            0, 0))    # yaw, yaw_speed

def move_down():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
            int(0b110111000111),
            0, 0, 0,  # position
            0, 0, -1,  # velocity
            0, 0, 0,  # acceleration
            0, 0))    # yaw, yaw_speed

def move_left():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        int(0b010111000111), 
        0, 0, 0,  # position
        0, -1, 0, # velocity
        0, 0, 0,  # acceleration
        0, 0))    # yaw, yaw_speed

def move_right():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 
        int(0b010111000111), 
        0, 0, 0, 
        0, 1, 0, 
        0, 0, 0, 
        0, 0))

def yaw_left():
    the_connection.mav.command_long_send(
        the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        15,  # YAW
        5, # YAW_SPEED
        -1, # -1: CCW, 1:CW
        1,  # 0:abs, 1:rel
        0, 0, 0)

def yaw_right():
    the_connection.mav.command_long_send(
        the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        15,  # YAW
        5,  # YAW_SPEED
        1,  # -1: CCW, 1:CW
        1,  # 0:abs, 1:rel
        0, 0, 0)

def teleop():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))

        while True:
            keystroke = s.recv(1024).decode()
            print("Server sent data ", keystroke)
            if not keystroke:
                break

            # Execute the same command locally
            if keystroke == "'w'":
                move_forward()
                print("Drone is moving forward")
            elif keystroke == "'s'":
                move_backward()
                print("Drone is moving backward")
            elif keystroke == "'a'":
                turn_left()
                print("Drone is turning & moving left")
            elif keystroke == "'d'":
                turn_right()
                print("Drone is turning & moving right")
            elif keystroke == "'q'":
                yaw_left()
                print("Drone is changing heading towards left")
            elif keystroke == "'e'":
                yaw_right()
                print("Drone is changing heading towards right")
            elif keystroke == "'u'":
                move_up()
                print("Drone is ascending")
            elif keystroke == "'n'":
                move_down()
                print("Drone is Descending")
            elif keystroke == "'z'":
                move_left()
                print("Drone is moving left")
            elif keystroke == "'c'":
                move_right()
                print("Drone is moving right")

            elif keystroke == "' '":
                print("Teleoperation Stopped")
                break

if __name__ == '__main__':
    teleop()