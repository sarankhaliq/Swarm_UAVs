import socket
import json
import time
from pymavlink import mavutil

# Initialize global variables for GPS data and waypoints
waypoints = [
    (33.771027, 72.823004, 10),  # Example waypoints (lat, lon, alt)
    (33.771018, 72.823444, 10),
    (33.770380, 72.823417, 10)
]
current_waypoint_index = 0
previous_gps_position = None


#Mavlink_connection
the_connection = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

# Wait for the first heartbeat
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

def mod_gps(uav1):
    msg = uav1.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat / 10**7
    lon = msg.lon / 10**7
    alt = msg.relative_alt * 0.001
    return lat, lon, alt

def send_to_waypoint(uav1, waypoint):
    lat, lon, alt = waypoint
    uav1.mav.set_position_target_global_int_send(
        0,  # time_boot_ms
        uav1.target_system, uav1.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,  # type_mask (only positions enabled)
        int(lat * 10**7), int(lon * 10**7), 30,
        0, 0, 0,  # x, y, z velocity (not used)
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0  # yaw, yaw_rate (not used)
    )

def is_moving_towards_waypoint(lat, lon, alt, waypoint):
    distance_threshold = 0.0001  # Define a small threshold to determine movement
    return abs(lat - waypoint[0]) > distance_threshold or abs(lon - waypoint[1]) > distance_threshold

def has_reached_waypoint(lat, lon, waypoint):
    distance_threshold = 0.0001
    return abs(lat - waypoint[0]) < distance_threshold and abs(lon - waypoint[1]) < distance_threshold

def start_mavlink_server():
    try:
        # Create a UDP socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Define server address and port
        host = ''
        port = 12345

        # Bind socket to address and port
        server_socket.bind((host, port))
        print(f'Server listening on {host}:{port}')

        global current_waypoint_index
        global previous_gps_position

        # Initially move to the first waypoint
        send_to_waypoint(the_connection, waypoints[current_waypoint_index])

        while True:
            # Receive data from client
            data, addr = server_socket.recvfrom(1024)
            print(f"Received data from {addr}, and data is {data}")

            # Fetch GPS data using MAVLink connection
            lat, lon, alt = mod_gps(the_connection)

            # Initialize previous_gps_position if it is None
            if previous_gps_position is None:
                previous_gps_position = (lat, lon, alt)

            # Send the current target waypoint to the client
            target_waypoint = waypoints[current_waypoint_index]
            waypoint_data = {'lat': target_waypoint[0], 'lon': target_waypoint[1], 'alt': target_waypoint[2]}
            gps = json.dumps(waypoint_data).encode('utf-8')
            server_socket.sendto(gps, addr)
            print(f"Sent target waypoint data to {addr}")

            # Print the current target waypoint
            print(f"Current GPS position the drone is going to: {target_waypoint}")

            # Check if the waypoint is reached
            if has_reached_waypoint(lat, lon, waypoints[current_waypoint_index]):
                if (lat, lon, alt) != previous_gps_position:
                    # Move to the next waypoint
                    if current_waypoint_index < len(waypoints) - 1:
                        current_waypoint_index += 1
                        send_to_waypoint(the_connection, waypoints[current_waypoint_index])
                        previous_gps_position = (lat, lon, alt)
                        print(f"Moving to the next waypoint: {waypoints[current_waypoint_index]}")
                        time.sleep(1)
                    
                    else:
                        print("Reached the last waypoint. Mission complete.")
                        break

            time.sleep(1)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        server_socket.close()

if __name__ == '__main__':
    start_mavlink_server()