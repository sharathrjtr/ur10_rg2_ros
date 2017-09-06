#!/usr/bin/env python

#from server_ur10.srv import *
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
import socket
import netifaces as ni

def server_ur10():
	ni.ifaddresses('eth0')
	ip = ni.ifaddresses('eth0')[2][0]['addr']
	rospy.init_node('rg2_ur10_server')
	global pub, data
	pub = rospy.Publisher('rg2_gripped', Bool, queue_size=1)
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server_address = (ip, 50002)
	sock.bind(server_address)
	sock.listen(1)
	while True:
		connection, client_address = sock.accept()
		try:
			data = connection.recv(64)
			data = float(data)
			if (data-10) > 5 :
				pub.publish(True)
			else :
				pub.publish(False)
		finally:
			connection.close()

if __name__ == "__main__":
        global gripper_width
	server_ur10()
