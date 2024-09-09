import socket
import json
from pymavlink import mavutil

# Initialize global variables for GPS data
lat = 33.771027
lon = 72.822985
alt = 20.0

# Initialize MAVLink connection
#the_connection = mavutil.mavlink_connection('udpin:localhost:14551')
the_connection = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Function to fetch and return GPS coordinates
def mod_gps(uav1):
    global lat, lon, alt

    uav1.wait_heartbeat()
    msg = uav1.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    
    lat = msg.lat / 10**7
    lon = msg.lon / 10**7
    alt = msg.relative_alt * 0.001
    
    #print(f"Current GPS: Lat={lat}, Lon={lon}, Alt={alt}")
    
    return lat, lon, alt

# Function to start MAVLink server
def start_mavlink_server():
    try:
        # Create a UDP socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Define server address and port
        host = '192.168.1.82'
        port = 12345

        # Bind socket to address and port
        server_socket.bind(('', port))
        print(f'Server listening on {host}:{port}')

        #lat,lon,alt = mod_gps(the_connection)
        print(type(lat))

        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system, the_connection.target_component,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b110111111000) , int(lat * 10 **7), int(lon *10 **7), 20 ,0 ,0 ,0, 0, 0, 0, 0, 0))

        while True:
            # Receive data from client
            
            data1, addr = server_socket.recvfrom(1024)
            print(f"Received data from {addr}")
            print(data1)

            # Fetch GPS data using MAVLink connection
            #lat, lon, alt = mod_gps(the_connection)
            GPS = (lat, lon, alt)
            data = {'lat': lat, 'lon': lon, 'alt': alt}
            gps = json.dumps(data).encode('utf-8')
            # Send GPS data back to client
            server_socket.sendto(gps, addr)
            
            print(f"Sent GPS data to {addr}")

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        server_socket.close()

if __name__ == '__main__':
    start_mavlink_server()
