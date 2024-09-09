import socket
from pymavlink import mavutil
import json


# Create MAVLink connection
mavlink_connection = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)
mavlink_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (mavlink_connection.target_system, mavlink_connection.target_component))

def mod_gps(uav1):
    while True:
        msg = uav1.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            lat = msg.lat / 10**7
            lon = msg.lon / 10**7
            alt = msg.relative_alt * 0.001
            print("Current lat = ", lat)
            return lat, lon, alt

def start_mavlink_client():
    try:
        # Create a UDP socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Server address and port
        host = '192.168.1.99'
        port = 12345

        # Get GPS data
        lat, lon, alt = mod_gps(mavlink_connection)
        print('Socket Created')
        data = {'lat': lat, 'lon': lon, 'alt': alt}
        gps = json.dumps(data).encode('utf-8')

        # Send the GPS data to the server
        client_socket.connect((host, port))
        client_socket.sendall(gps)
        print('Data Sent')

        # Receive acknowledgment from the server
        data, server = client_socket.recvfrom(1024)
        rcv_data = json.loads(data.decode('utf-8'))
        alt = rcv_data['alt']
        lon = rcv_data['lon']
        lat = rcv_data['lat']
        print("Data Received")
        print([lat, lon, alt])

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        client_socket.close()

if __name__ == '__main__':
    start_mavlink_client()
