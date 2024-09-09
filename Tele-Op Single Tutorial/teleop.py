import sys
import socket
from pymavlink import mavutil
import time
import readchar
#import rospy
import threading
#from std_msgs.msg import Float64
#from std_msgs.msg import Float32MultiArray
#from std_msgs.msg import MultiArrayDimension


the_connection = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0 , 0, 0, 0, 0)
msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True)
time.sleep(1)
#the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0 , 0, 0, 0, 10)
#msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True)

def teleop():

      while True:
            keystroke=repr(readchar.readchar())
            
            #print(keystroke)
            if (keystroke=="'s'"):
                  the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                               the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_NED, int(0b010111000111), 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0))
            elif (keystroke=="'a'"):
                  the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                               the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_NED, int(0b010111000111), 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0))
            elif (keystroke=="'w'"):
                  the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                               the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_NED, int(0b010111000111), 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0))
            elif (keystroke=="'d'"):
                  the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                               the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_NED, int(0b010111000111), 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0))
            elif (keystroke=="'q'"):

                  the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 5, 60, -1, 1, 0, 0, 0)
            elif (keystroke=="'e'"):
                  the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 5, 60, 1, 1, 0, 0, 0)
            elif (keystroke=="'m'"):
                  break
      #mod_gps(the_connection)
            


'''def mod_gps(uav1):
	uav1.wait_heartbeat()
	#while not rospy.is_shutdown():
	msg = uav1.recv_match(type='HEARTBEAT', blocking=True)
	if msg:
		mode = mavutil.mode_string_v10(msg)
	lat = uav1.messages['GLOBAL_POSITION_INT'].lat/10**7
	lon = uav1.messages['GLOBAL_POSITION_INT'].lon/10**7
	alt = uav1.messages['GLOBAL_POSITION_INT'].relative_alt*0.001
	mat = Float32MultiArray()
	#mat.layout.dim.append(MultiArrayDimension())
	#mat.layout.dim[0].label = "gps"
	#mat.layout.dim[0].size = 2
	#mat.layout.dim[0].stride = 2
	#mat.layout.data_offset = 0
	mat.data = [lat, lon]
	# save a few dimensions:
	#dstride0 = mat.layout.dim[0].stride
	#offset = mat.layout.data_offset

	#msg =[float(lat), float(lon)]
	pub = rospy.Publisher('gps_pos', Float32MultiArray, queue_size=10)
	rospy.init_node('gps_pos', anonymous=True)
	rate = rospy.Rate(1000)
	#while not rospy.is_shutdown():
	pub.publish(mat)
	rospy.loginfo("I'm sending:")
	print(mat.data)

	return mode, lat, lon, alt'''

if __name__ == '__main__':
    
    	teleop()
    #except rospy.ROSInterruptException:



'''def start():
	

    while True:
  	
      
        #thread = threading.Thread(target=mod_gps, args=(the_connection))
        thread1 = threading.Thread(target=teleop)
        #thread.start()
        thread1.start()
        #counter = counter +1
        print("[ACTIVE CONNECTIONS] ",threading.activeCount() - 1)


start()'''
