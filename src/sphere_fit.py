#!/usr/bin/env python3
import numpy as np
import rospy
#I could not get them to import from the other nodes so i just replicated them
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

#By Trey Castle

msg_received = False

def get_msg(var):
	#establish global variables
	global msg_received
	global point_arr
	length = 0
	msg_received = True
	point_arr = []
	#breaks down points into a tuple with a for loop
	for i in var.points:
		point_arr.append((i.x, i.y, i.z))

def fit(points):
	B = []
	A = []
	#iterate through the points to construct both the A and B matrix
	for point in points:
		B.append([point[0]**2 + point[1]**2 + point[2]**2])
		A.append([2*point[0], 2*point[1], 2*point[2], 1])
	A = np.array(A)
	B = np.array(B)
	dif = len(B)
	#makes them into the proper dimensions to calculate P
	B2 = B.reshape(dif,1)
	A2 = A.reshape(dif,4)
	P = np.linalg.lstsq(A2, B2, rcond=None)
	return P


def get_radius(P):
	#uses xc, yc, and zc to get rad with the radius formula
	rad = np.sqrt(P[0][3] + P[0][0]**2 + P[0][1]**2 + P[0][2]**2)
	return rad
	


if __name__ == '__main__':
	#define the node, subscriber, and publisher
	rospy.init_node('sphere_fit', anonymous=True)
	sub = rospy.Subscriber('xyz_cropped_ball', XYZarray, get_msg)
	pub = rospy.Publisher("/sphere_params", SphereParams, queue_size=1)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		#calls fit to get P. then calls the get radius function, before establishing and publishing sphere parameters
		if msg_received:
			P = fit(point_arr)
			rad = get_radius(P)
			param = SphereParams(float(P[0][0]), float(P[0][1]), float(P[0][2]), rad)
			pub.publish(param)
		rate.sleep()
