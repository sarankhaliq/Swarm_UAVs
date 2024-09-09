import sys
import time
import signal
import socket
import readchar
import threading
from pymavlink import mavutil

# Server's IP & Port
host = '192.168.1.81'
port = 12345

# Mavlink_connection
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,  # 0:disarm, 1:arm
    0,
    0, 0, 0, 0, 0)

time.sleep(1)

the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0,
    10)  # desired altitude

time.sleep(1)

def move_forward():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            int(0b010111000111),
            0, 0, 0,  # position
            1, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            0, 0))    # yaw, yaw_speed

def move_backward():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            int(0b010111000111),
            0, 0, 0,  # position
            -1, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            0, 0))    # yaw, yaw_speed

def move_left():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            int(0b010111000111),
            0, 0, 0,  # position
            0, -1, 0,  # velocity
            0, 0, 0,  # acceleration
            0, 0))    # yaw, yaw_speed

def move_right():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            int(0b010111000111),
            0, 0, 0,  # position
            0, 1, 0,  # velocity
            0, 0, 0,  # acceleration
            0, 0))

def move_up():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            int(0b110111111000),
            0, 0, -1,  # position
            0, 0, 0, # velocity
            0, 0, 0,  # acceleration
            0, 0))    # yaw, yaw_speed

def move_down():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            int(0b110111111000),
            0, 0, -1,  # position
            0, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            0, 0))    # yaw, yaw_speed

def turn_right():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            int(0b110111111000),
            0, 1, 0,  # position
            0, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            0, 0))    # yaw, yaw_speed
def turn_left():
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            int(0b110111111000),
            0, -1, 0,  # position
            0, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            0, 0))    # yaw, yaw_speed
def yaw_left():
    the_connection.mav.command_long_send(
        the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        15,  # YAW
        5,  # YAW_SPEED
        -1,  # -1: CCW, 1:CW
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

def print_guide():
    guide = """
    ==========================================================
                      Drone Teleoperation Guide
    ==========================================================
    Controls:
        w - Move Forward
        s - Move Backward
        a - Move Left
        d - Move Right
        u - Move Up
        n - Move Down
        z - Turn Left
        c - Turn Right
        q - Yaw Left
        e - Yaw Right
        space - Exit
    ----------------------------------------------------------
    Instructions:
    - Use 'w', 'a', 's', 'd' keys to control the drone's movement.
    - Use 'q' and 'e' keys to control the drone's yaw.
    - Press 'space' to exit the teleoperation mode.
    ==========================================================
    """
    print(guide)

# List to keep track of all connected clients
clients = []

def handle_client(conn, addr):
    global clients
    print(f"Connected by {addr}")
    clients.append(conn)
    try:
        while True:
            try:
                keystroke = repr(readchar.readchar())

                # Broadcast keystroke to all clients
                for client in clients:
                    try:
                        client.sendall(keystroke.encode())
                    except BrokenPipeError:
                        clients.remove(client)

                # Execute the same command locally
                if keystroke == "'w'":
                    move_forward()
                elif keystroke == "'s'":
                    move_backward()
                elif keystroke == "'a'":
                    move_left()
                elif keystroke == "'d'":
                    move_right()
                elif keystroke == "'q'":
                    yaw_left()
                elif keystroke == "'e'":
                    yaw_right()
                elif keystroke == "'u'":
                    move_up()
                elif keystroke == "'n'":
                    move_down()
                elif keystroke == "'z'":
                    turn_left()
                elif keystroke == "'c'":
                    turn_right()
                elif keystroke == "' '":
                    
                    break
            except BrokenPipeError:
                print(f"Connection lost with {addr}")
                break
    finally:
        conn.close()
        clients.remove(conn)
        print(f"Connection closed with {addr}")

def teleop():
    print_guide()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen(10)
        print(f"Server listening on {host}:{port}")
        while True:
            conn, addr = s.accept()
            client_thread = threading.Thread(target=handle_client, args=(conn, addr))
            client_thread.start()
        print("Shutting down server...")
        s.close()
        sys.exit(0)

def mod_gps(uav1):
    while True:
        msg = uav1.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt * 0.001
            print(f"GPS Data - Latitude: {lat}, Longitude: {lon}, Altitude: {alt}")


if __name__ == '__main__':
    teleop()