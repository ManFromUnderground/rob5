#!/usr/bin/env python3
import numpy as np
import rospy
#I could not get them to import from the other nodes so i just replicated them
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

msg_received = False

def get_msg(var):
	global msg_received
	global point_arr
	length = 0
	msg_received = True
	point_arr = []
	for i in var.points:
		point_arr.append((i.x, i.y, i.z))

def fit(points):
	B = []
	A = []
	for point in points:
		B.append([point[0]**2 + point[1]**2 + point[2]**2])
		
		A.append([2*point[0], 2*point[1], 2*point[2], 1])
	A = np.array(A)
	B = np.array(B)
	#print("1D B:", B)
	dif = len(B)
	B2 = B.reshape(dif,1)
	#print("2D B:", B)
	A2 = A.reshape(dif,4)
	P = np.linalg.lstsq(A2, B2, rcond=None)
	
	
	return P


def get_radius(P):
	rad = np.sqrt(P[0][3] + P[0][0]**2 + P[0][1]**2 + P[0][2]**2)
	return rad
	


if __name__ == '__main__':
	#define the node, subscribers, and publishers
	rospy.init_node('sphere_fit', anonymous=True)
	#break it down into individual messages
	sub = rospy.Subscriber('xyz_cropped_ball', XYZarray, get_msg)
	pub = rospy.Publisher("/sphere_params", SphereParams, queue_size=1)
	rate = rospy.Rate(10)
	#param = SphereParams()
	while not rospy.is_shutdown():
		if msg_received:
			P = fit(point_arr)
			rad = get_radius(P)
			param = SphereParams(float(P[0][0]), float(P[0][1]), float(P[0][2]), rad)
			pub.publish(param)
		rate.sleep()
