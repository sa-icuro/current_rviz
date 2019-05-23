#!/usr/bin/env python

# Copyright (c) 2015, Carnegie Mellon University
# All rights reserved.
# Authors: David Butterworth <dbworth@cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
This is a demo of Rviz Tools for python which tests all of the
available functions by publishing lots of Markers in Rviz.
"""

# Python includes
import random
import re

# ROS includes
import roslib
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()
import sys, termios, tty, os, time
import rviz_tools_py as rviz_tools
import json

markers = rviz_tools.RvizMarkers('/map', 'visualization_marker')

##### FUNCTIONS TO LOAD FROM FILE #####

def byteify(input):
	if isinstance(input, dict):
		return {byteify(key): byteify(value)
				for key, value in input.iteritems()}
	elif isinstance(input, list):
		return [byteify(element) for element in input]
	elif isinstance(input, unicode):
		return input.encode('utf-8')
	else:
		return input

def isclose(a, b, rel_tol=1e-05, abs_tol=0.0):
	return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

##### END OF FUNCTIONS TO LOAD FROM FILE #####

#Load Points
list_points = []
if(os.path.getsize('/home/centaur/catkin_ws/src/magni_goal_sender/param/points.txt')) > 0:
	with open('/home/centaur/catkin_ws/src/magni_goal_sender/param/points.txt', 'rb') as f:
		for x in f.readlines():
			temp_tup1 = x.replace("(","")
			temp_tup2 = temp_tup1.replace(" ","")
			temp_tup1 = temp_tup2.replace(")","")
			temp_tup2 = temp_tup1.split(",")
			list_points.append((int(temp_tup2[0]), float(temp_tup2[1]), float(temp_tup2[2]), float(temp_tup2[3])))
print "\nLIST_POINTS: " + str(list_points)
#list_paths = []


#Load Paths
paths = dict((k,[]) for k in range(len(list_points)))
if(os.path.getsize('/home/centaur/catkin_ws/src/magni_goal_sender/param/paths.json')) > 0:
	with open('/home/centaur/catkin_ws/src/magni_goal_sender/param/paths.json', "r") as f:
		paths = byteify(json.load(f))
print "\nPATHS: " + str(paths)

#Global Variables for Callback_safepoint
vertex_count = len(list_points)

#Global Variables for Callback_path
first = 0
first_path_node = 0

#Global Variables/Setup for Callback_route
path_route = []
last_goal_id = -1

#open("/home/centaur/catkin_ws/src/magni_goal_sender/param/final_goals.yaml", 'w').close()
with open("/home/centaur/catkin_ws/src/magni_goal_sender/param/final_goals.yaml", 'w+') as f:
    f.write('goals:\n')
    f.close()

# Define exit handler
def cleanup_node():
	print "Shutting down node!"
	
	with open('/home/centaur/catkin_ws/src/magni_goal_sender/param/points.txt', 'w+') as f:
		for s in list_points:
			f.write(str(s) + '\n')

	with open('/home/centaur/catkin_ws/src/magni_goal_sender/param/paths.json', 'w+') as f:
		json.dump(paths, f)

rospy.on_shutdown(cleanup_node)

def callback_safepoint(msg):
	rospy.loginfo("Received a Safepoint!")

	# Copying for simplicity
	position = msg.pose.position
	quat = msg.pose.orientation
	rospy.loginfo("Safepoint data: [ %f, %f, %f ]"%(position.x, position.y, quat.z))

	# Publish a sphere using a ROS Pose
	P = Pose(Point(position.x,position.y,0),Quaternion(0,0,quat.z,1))
	cube_width = 0.2 # diameter
	color = 'orange'
	markers.publishCube(P, color, cube_width, 0) # pose, color, scale, lifetime

	tup = (globals()['vertex_count'], position.x, position.y, quat.z)
	print "safe tuple: ", tup
	list_points.append(tup)
	#print list_points
	paths[globals()['vertex_count']] = []
	globals()['vertex_count'] += 1

	#Write to Goals.yaml as used by the magni_goal_sender package
	f = open("/home/centaur/catkin_ws/src/magni_goal_sender/param/safe_waypoints.txt","a+")
	f.write("- %f, %f, %f\n" % (position.x, position.y, quat.z))
	f.close()

def callback_path(msg):

	rospy.loginfo("Received a Path!")

	# Copying for simplicity
	position = msg.pose.position
	quat = msg.pose.orientation
	rospy.loginfo("Path data (Appending to SafePath): [ %f, %f, %f ]"%(position.x, position.y, quat.z))

	# Publish a sphere using a ROS Pose
	P = Pose(Point(position.x,position.y,0),Quaternion(0,0,quat.z,1))
	scale = Vector3(0.275,0.275,0.275) # diameter
	color = 'purple'
	markers.publishSphere(P, color, scale, 0) # pose, color, scale, lifetime

	print "\nfirst_path_node[0]: " + str(globals()['first_path_node'])
	print "first[0]: " + str(globals()['first'])
	val_id = 0
	for x in list_points:
		if (isclose(x[1],position.x) and isclose(x[2],position.y)):
			print "\nMATCH!!! with value# " + str(x[0])
			val_id = x[0]
			break
			#globals()['first_path_node'] = x[0]

	if (globals()['first'] == 0):
		print "\nFirst value of PathPair"
		globals()['first_path_node'] = val_id
		print "first_path_node[0]: " + str(globals()['first_path_node'])
		print "first[0]: " + str(globals()['first'])
		#print "x[0] value: " + str(val_id)

		globals()['first_path_node'] = x[0]
		globals()['first'] = 1
	elif(globals()['first'] == 1):
		print "\nSecond Value of PathPair"
		print "first_path_node[1]: " + str(globals()['first_path_node'])
		print "first[1]: " + str(globals()['first'])
		print "x[0] value in [1]: " + str(val_id)
		print paths
		try:
			paths[str(globals()['first_path_node'])].append(val_id)
			paths[str(val_id)].append(globals()['first_path_node'])
		except:
			print "caught an exception!"
			paths[globals()['first_path_node']].append(val_id)
			paths[val_id].append(globals()['first_path_node'])
		print paths
		p1 = Point(list_points[globals()['first_path_node']][1],list_points[globals()['first_path_node']][2],0)
		p2 = Point(list_points[x[0]][1],list_points[x[0]][2],0)
		markers.publishLine(p1, p2, 'green', 0.05, 0)
		globals()['first'] = 0

	print "\nAt the end of callback"
	print "first_path_node[2]: " + str(globals()['first_path_node'])
	print "first[2]: " + str(globals()['first'])
	print "x[0] value @ end: " + str(x[0])

def callback_goal(msg):
	check = 0
	rospy.loginfo("Received a Goal!")

	# Copying for simplicity
	position = msg.pose.position
	quat = msg.pose.orientation
	
	# Publish a sphere using a ROS Pose
	P = Pose(Point(position.x,position.y,0),Quaternion(0,0,quat.z,1))
	scale = Vector3(0.33,0.33,0.33) # diameter
	color = 'blue'

	val_id = 0
	for x in list_points:
		if (isclose(x[1],position.x) and isclose(x[2],position.y)):
			print "\nThis Node is # " + str(x[0])
			val_id = x[0]
			break

	print "\nlast_goal_id(before first search): " + str(globals()['last_goal_id'])
	if (globals()['last_goal_id'] == -1):
		globals()['last_goal_id'] = val_id
		markers.publishSphere(P, color, scale, 0) # pose, color, scale, lifetime
		path_route.append( Point(position.x,position.y,0) )
		check = 1

	if ((globals()['last_goal_id'] > -1) and check == 0):
		try:
			if(val_id in paths[str(globals()['last_goal_id'])]):
				print "\nFound Val_id: " + str(val_id) + " in " "paths 1: " + str(paths[str(globals()['last_goal_id'])])
				print "Connecting Node " + str(val_id) + "to Node " + str(globals()['last_goal_id'])
				globals()['last_goal_id'] = val_id
				print "here"
				markers.publishSphere(P, color, scale, 0) # pose, color, scale, lifetime
				rospy.sleep(0.01)
				path_route.append( Point(position.x,position.y,0) )
				markers.publishPath(path_route, 'white', 0.1, 0) # path, color, width, lifetime
				print "did it print"
				check = 1
			#elif(globals()['first_path_node'] in paths[str(val_id)]):
			#    print "\nlast_goal_id: " + str(globals()['first_path_node']) + " in " "paths 2: " + str(paths[str(globals()[str(val_id)])]) 
			#    markers.publishSphere(P, color, scale, 0) # pose, color, scale, lifetime
			#    path_route.append( Point(position.x,position.y,0) )
			#    markers.publishPath(path_route, 'white', 0.1, 0) # path, color, width, lifetime
			#    check = 1
		except:
			print "caught an exception!"
			if(val_id in paths[globals()['last_goal_id']]):
				print "\nFound Val_id: " + str(val_id) + " in " "paths 3: " + str(paths[globals()['last_goal_id']])
				print "Connecting Node " + str(val_id) + "to Node " + str(globals()['last_goal_id'])
				globals()['last_goal_id'] = val_id
				markers.publishSphere(P, color, scale, 0) # pose, color, scale, lifetime
				path_route.append( Point(position.x,position.y,0) )
				markers.publishPath(path_route, 'white', 0.1, 0) # path, color, width, lifetime
				check = 1
			#elif(globals()['first_path_node'] in paths[val_id]):
			#    print "\nlast_goal_id: " + str(globals()['first_path_node']) + " in " "paths 4: " + str(paths[globals()[str(val_id)]]) 
			#    markers.publishSphere(P, color, scale, 0) # pose, color, scale, lifetime
			#    path_route.append( Point(position.x,position.y,0) )
			#    markers.publishPath(path_route, 'white', 0.1, 0) # path, color, width, lifetime
			#    check = 1

	if (check == 0):
		print "NO MATCH"
		print "Cannot connect Node " + str(val_id) + "to Node " + str(globals()['last_goal_id'])
	elif (check == 1):
		#add functionality to clear final_goals 
		if(os.path.lexists('/home/centaur/catkin_ws/src/magni_goal_sender/param/final_goals.yaml')):
			f = open("/home/centaur/catkin_ws/src/magni_goal_sender/param/final_goals.yaml","a+")
		else:
			f = open("/home/centaur/catkin_ws/src/magni_goal_sender/param/final_goals.yaml","a+")
			f.write("goals: \n")
		
		print "last_goal_id(after): " + str(globals()['last_goal_id'])
		rospy.loginfo("Goal data (Appending to route): [ %f, %f, %f ]"%(position.x, position.y, quat.z))
		f.write("- %f, %f, %f, 0\n" % (position.x, position.y, quat.z))
		f.close()
		check = 0

def populate():
	print "\nLOADING POINTS..."
	count = 0

	for item in list_points:
		if (count == 0):
			rospy.sleep(0.2)

		P = Pose(Point(item[1],item[2],0),Quaternion(0,0,item[3],1))
		scale = Vector3(0.275,0.275,0.275) # diameter

		rospy.sleep(0.02)
		#markers.publishSphere(P, 'purple', scale, 0) # pose, color, scale, lifetime
		markers.publishCube(P, 'orange', 0.2, 0) # pose, color, scale, lifetime
		rospy.sleep(0.02)
		count += 1

	print "...DONE LOADING POINTS\n"

	print "\nLoading PATHS:"

	for key, value in paths.iteritems():
		print "key: " + str(key)
		if(len(value) > 0):
			print "key (inside method): " + str(key)
			p1 = Point(list_points[int(key)][1],list_points[int(key)][2],0)
			markers.publishSphere(Pose(p1,Quaternion(0,0,list_points[int(key)][3],1)), 'purple', Vector3(0.275,0.275,0.275), 0) # pose, color, scale, lifetime

			for x in value:
				p2 = Point(list_points[x][1],list_points[x][2],0)
				markers.publishLine(p1, p2, 'green', 0.05, 0)
				rospy.sleep(0.005)
				markers.publishSphere(Pose(p2,Quaternion(0,0,list_points[x][3],1)), 'purple', Vector3(0.275,0.275,0.275), 0) # pose, color, scale, lifetime

	print "...DONE LOADING PATHS\n"

def listener():
	rospy.init_node('listener', anonymous=True)
	populate()
	rospy.Subscriber("/safepointz", PoseStamped, callback_safepoint)
	rospy.Subscriber("/goalz", PoseStamped, callback_goal)
	rospy.Subscriber("/safepathz", PoseStamped, callback_path)

	rospy.spin()

if __name__ == '__main__':
	listener()