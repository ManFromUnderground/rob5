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
	msg_received = True
	point_arr = var

def fit(points):
	A = np.vstack([[2*points[0],2*points[1],2*points[2]], np.ones(len(points))]).T
	
	B = np.array(points[0]**2 + points[1]**2 + points[2]**2).T
	ATA = np.matmul(A.T, A)
	ATB = np.matmul(A.T, B)
	
	P = np.matmul(np.linalg.inv(ATA), ATB)
	x1 = P[0]
	y1 = P[1]
	z1 = P[2]
	r1 = P[3]
	
	return (x1, y1, z1, r1)

def get_radius(P):
	rad = sqrt(P[3] + P[1]**2 + P[2]**2 + P[0]**2)
	return rad
	


if __name__ == '__main__':
	#define the node, subscribers, and publishers
	rospy.init_node('sphere_fit', anonymous=True)
	sub = rospy.Subscriber('xyz_cropped_ball', XYZarray, get_msg)
	pub = rospy.Publisher("robot_vision_lectures/Sphere_Params", SphereParams, queue_size=1)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if msg_received:
			P = fit(point_arr)
			rad = get_radius(P)
			pub.publish(P[0], P[1], P[2], rad)
			#print("published")
		rate.sleep()
