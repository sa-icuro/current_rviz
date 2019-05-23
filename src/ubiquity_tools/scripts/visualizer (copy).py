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
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()
import sys, termios, tty, os, time
import rviz_tools_py as rviz_tools
import json

dirname = os.path.dirname(__file__)
safepoint_file = os.path.join(dirname, 'param/session_current/points.txt')
safepath_file = os.path.join(dirname, 'param/session_current/paths.json')
route_file = os.path.join(dirname, 'param/session_current/routes/goals_current.txt')

markers = rviz_tools.RvizMarkers('/map', 'visualization_marker')
pub = rospy.Publisher('save_data', String, queue_size = 1)


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

def isclose2(a, b, rel_tol=1e-09, abs_tol=0.0):
	return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

##### END OF FUNCTIONS TO LOAD FROM FILE #####

#Data Structures
list_points = [] #(Node_ID, Position_X, Position_Y, Angle?/Quat_Z? )
dict_paths = {}  # Node_ID:CONNECTING_IDs
list_goals = []  #(Position_X, Position_Y, Angle?/Quat_Z?, Wait_Time)
route_path = []  #(Node_ID, Position_X, Position_Y, Angle?/Quat_Z? ) to connect route points


#Global Variables/Setup for Callback_setup
populated = 0
request_type = ''

#Global Variables for Callback_safepoint
#vertex_count = len(list_points)
node_count = 0

#Global Variables for Callback_path
first = 0
first_path_node = 0

#Global Variables/Setup for Callback_route 
path_route = []
last_goal_id = -1

########### Visualize Tools ##########
def populatePoints():
	print "\nLOADING POINTS..."
	count = 0

	#print "\nlist_points: " + str(list_points)
	print "len_point: " + str(len(list_points))
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

def populatePaths():
	print "\nLOADING PATHS:"
	print "len_path: " + str(len(dict_paths))

	for key, value in dict_paths.iteritems():
		#print "key: " + str(key)
		if(len(value) > 0):
			#print "key (inside method): " + str(key)
			p1 = Point(list_points[int(key)][1],list_points[int(key)][2],0)
			markers.publishSphere(Pose(p1,Quaternion(0,0,list_points[int(key)][3],1)), 'purple', Vector3(0.275,0.275,0.275), 0) # pose, color, scale, lifetime

			for x in value:
				try:
					p2 = Point(list_points[x][1],list_points[x][2],0)
				except TypeError:
					print "caught!"
					p2 = Point(list_points[int(x)][1],list_points[int(x)][2],0)
				markers.publishLine(p1, p2, 'green', 0.05, 0)
				rospy.sleep(0.005)
				try:
					markers.publishSphere(Pose(p2,Quaternion(0,0,list_points[x][3],1)), 'purple', Vector3(0.275,0.275,0.275), 0) # pose, color, scale, lifetime
				except TypeError:
					markers.publishSphere(Pose(p2,Quaternion(0,0,list_points[int(x)][3],1)), 'purple', Vector3(0.275,0.275,0.275), 0) # pose, color, scale, lifetime

	print "...DONE LOADING PATHS\n"

def populateGoals():
	print "\nLOADING ROUTE..."
	count = 0
	print "len_route: " + str(len(list_goals))

	for item in list_goals:
		if (count == 0):
			rospy.sleep(0.2)

		P = Pose(Point(item[1],item[2],0),Quaternion(0,0,item[3],1))
		scale = Vector3(0.33,0.33,0.33) # diameter

		rospy.sleep(0.02)
		#markers.publishSphere(P, 'purple', scale, 0) # pose, color, scale, lifetime
		markers.publishSphere(P, 'blue', scale, 0) # pose, color, scale, lifetime
		rospy.sleep(0.01)
		route_path.append(Point(item[1],item[2],0))
		markers.publishPath(route_path, 'white', 0.1, 0)
		rospy.sleep(0.02)
		count += 1

	print "...DONE LOADING ROUTE\n"

def buildNewRoute():
	markers.deleteAllMarkers()
	populatePoints()
	populatePaths()
	globals()['populateGoals'] = []
	globals()['route_path'] = []
########### END of Visualize Tools ##########

########### Callbacks to handle setup ##########
def callback_requested(data):
	globals()['request_type'] = str(data.data)
	if (str(data.data) == "save_data"):
		msg_points = '|'.join(str(x) for x in globals()['list_points'])
		msg_paths = '|'.join(str(key)+ ":" + str(val) for (key,val) in globals()['dict_paths'].iteritems())
		msg_route = '|'.join(str(x) for x in globals()['list_goals'])
		final_msg = '~'.join((msg_points,msg_paths,msg_route))
		pub.publish(final_msg)
		rospy.sleep(0.1)
		print "Saving data onto Magni"
	else:
		print "Requested: " + str(data.data)
	
def callback_setup(data):
	#If setup is called, it is assumed user is reloading the session in some way so we will 
	#reset all datastructures and repopulate
	globals()['list_points'] = []
	globals()['dict_paths'] = {}
	globals()['list_goals'] = []
	globals()['route_path'] = []

	print "Inside callback_setup: "

	info = str(data.data).split('~')

	temp_points = info[1].split('|')
	temp_paths = info[2].split('|')
	temp_goals = info[3].split('|')

	if (globals()['request_type'] == "new_session"):
		print "NEW SESSION??"
	if (globals()['request_type'] == "safepoints"):
		if(temp_points != [''] and (str(temp_points) != "['']")):
			for x in temp_points:
				temp_tup1 = x.replace("(","")
				temp_tup1 = temp_tup1.replace(" ","")
				temp_tup1 = temp_tup1.replace(")","")
				temp_tup1 = temp_tup1.split(",")
				globals()['list_points'].append((int(temp_tup1[0]), float(temp_tup1[1]), float(temp_tup1[2]), float(temp_tup1[3])))
				globals()['dict_paths'][str(int(temp_tup1[0]))] = []
		else:
			print "No safepoints inside ...session_current/points_current.txt. Cannot load safepoints!"

	if (globals()['request_type'] == "safepaths"):
		if(temp_points != [''] and (str(temp_points) != "['']")):
			for x in temp_points:
				temp_tup1 = x.replace("(","")
				temp_tup1 = temp_tup1.replace(" ","")
				temp_tup1 = temp_tup1.replace(")","")
				temp_tup1 = temp_tup1.split(",")
				globals()['list_points'].append((int(temp_tup1[0]), float(temp_tup1[1]), float(temp_tup1[2]), float(temp_tup1[3])))

			if((temp_paths != {''}) and (str(temp_paths) != "['']")):
				for item in temp_paths:
					item = item.replace('"','')
					item = item.replace('\'','')

					tmp = item.split(':')
					val_id = int(str(tmp[0]).strip('\''))

					globals()['dict_paths'][str(val_id)] = []

					tmp[1] = tmp[1].replace('[','')
					tmp[1] = tmp[1].replace(']','')
					tmp = tmp[1].split(',')
					#print "tmp[1]: " + tmp[1] + "\n"
					for x in tmp:
						x = str(x).strip(" ")
						#print "\nx: " + str(x) + "\n" 
						try:
							#print "Node: " + str(x) + " connected to node " + str(val_id) + "\n"
							globals()['dict_paths'][str(val_id)].append(int(x))
						except:
							#To catch error thrown by Null values
							#print "issue with adding Node " + str(x) + " node to " + str(val_id)
							continue
			else:
				print "No safepaths inside ...session_current/paths_current.txt. Cannot load safepaths! Only loaded safepoints!"
		else:
			print "No safepoints inside ...session_current/points_current.txt. Cannot load safepoints!"
		
	if (globals()['request_type'][:5] == "goals"):
		if(temp_points != [''] and (str(temp_paths) != "['']")):
			for x in temp_points:
				temp_tup1 = x.replace("(","")
				temp_tup1 = temp_tup1.replace(" ","")
				temp_tup1 = temp_tup1.replace(")","")
				temp_tup1 = temp_tup1.split(",")
				globals()['list_points'].append((int(temp_tup1[0]), float(temp_tup1[1]), float(temp_tup1[2]), float(temp_tup1[3])))

			if(temp_paths != {''} and (str(temp_paths) != "['']")):
				for item in temp_paths:
					item = item.replace('"','')
					item = item.replace('\'','')

					tmp = item.split(':')
					val_id = int(tmp[0])
					globals()['dict_paths'][str(val_id)] = []
					#print "tmp[1]: " + tmp[1] + "\n"
					tmp = str(tmp[1]).strip('[')
					tmp = tmp.strip(']')
					tmp = tmp.split(",")
					for x in tmp:
						x = str(x).strip(" ")
						#print "\nx: " + str(x) + "\n" 
						try:
							#print "Node: " + str(x) + " connected to node " + str(val_id) + "\n"
							globals()['dict_paths'][str(val_id)].append(int(x))
						except:
							#To catch error thrown by Null values
							#print "issue with adding Node " + str(x) + " node to " + str(val_id)
							continue

				if(temp_goals != [''] and (str(temp_goals) != "['']")):
					for x in temp_goals:
						temp_tup1 = x.replace("(","")
						temp_tup1 = temp_tup1.replace(" ","")
						temp_tup1 = temp_tup1.replace(")","")
						temp_tup1 = temp_tup1.split(",")
						globals()['list_goals'].append((float(temp_tup1[0]), float(temp_tup1[1]), float(temp_tup1[2]), float(temp_tup1[3])))
				else:
					print "No route data inside ...session_current/routes/"+globals()['request_type']+". Cannot load route! Only loaded safepoints and safepaths"
			else:
				print "No safepaths inside ...session_current/paths_current.txt. Cannot load safepaths! Only loaded safepoints!"
		else:
			print "No safepoints inside ...session_current/points_current.txt. Cannot load safepoints!"
		
	print "\nlist_points: " + str(globals()['list_points'])
	print "\ndict_paths: " + str(dict_paths)
	print "\nlist_goals: " + str(globals()['list_goals'])

	#Before ending the callback, we should delete the on-screen markers and repopulate as specified by new data
	markers.deleteAllMarkers()
	globals()['node_count'] = len(list_points)
	populatePoints()
	populatePaths()
	populateGoals() 

def callback_safepoint(msg):
	# Copying for simplicity
	position = msg.pose.position
	quat = msg.pose.orientation

	# Publish a sphere using a ROS Pose
	P = Pose(Point(position.x,position.y,0),Quaternion(0,0,quat.z,1))
	cube_width = 0.2 # diameter
	color = 'orange'
	markers.publishCube(P, color, cube_width, 0) # pose, color, scale, lifetime

	tup = ((globals()['node_count']),position.x,position.y,quat.z)
	print "safe tuple: ", tup
	list_points.append(tup)
	dict_paths[str(globals()['node_count'])] = []
	(globals()['node_count']) += 1

def callback_safepath(msg):
	items = str(msg.data).split(" ")
	x = float(items[0])
	y = float(items[1])
	print "Path data (Appending to SafePath)" + str(x) + ", " + str(y)
	z = float(0)

	#Try to find a matching quat for z
	for item in globals()['list_points']:
		if (isclose(item[1],x) or isclose2(item[1],x)):
			if (isclose(item[2],y) or isclose2(item[2],y)):
				z = item[3]
				print "Found quat: " + str(z)
			else:
				print "y doesnt match"
		else:
			print "x doesnt match"

	# Publish a sphere using a ROS Pose
	P = Pose(Point(x,y,0),Quaternion(0,0,z,1))
	scale = Vector3(0.275,0.275,0.275) # diameter
	color = 'purple'
	markers.publishSphere(P, color, scale, 0) # pose, color, scale, lifetime
'''
	print "\nParent Node: " + str(globals()['first_path_node'])
	print "Connecting: " + str(globals()['first'])
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
		print "Parent Node: " + str(globals()['first_path_node'])
		print "Connecting: " + str(globals()['first'])
		#print "x[0] value: " + str(val_id)
		globals()['first_path_node'] = x[0]
		globals()['first'] = 1
		print paths
	elif(globals()['first'] == 1):
		print "\nSecond Value of PathPair"
		print "Connecting Parent Node: " + str(globals()['first_path_node'])
		print "\nTo Child Node: " + str(val_id)
		print "Connecting: " + str(globals()['first'])
		print paths
		print "parent Child is a " + str(type(globals()['first_path_node']) is int)
		print "Node Child is a " + str(type(val_id) is int)
		
		try:
			if(not((val_id) in paths[str(globals()['first_path_node'])])):
				paths[str(globals()['first_path_node'])].append((val_id))
		except KeyError:
			if(not((val_id) in paths[globals()['first_path_node']])):
				paths[(globals()['first_path_node'])].append((val_id))
		try:
			if(not((globals()['first_path_node']) in paths[str(val_id)])):		
				paths[str(val_id)].append((globals()['first_path_node']))
		except KeyError:
			if(not((globals()['first_path_node']) in paths[(val_id)])):		
				paths[(val_id)].append((globals()['first_path_node']))

		p1 = Point(list_points[globals()['first_path_node']][1],list_points[globals()['first_path_node']][2],0)
		p2 = Point(list_points[x[0]][1],list_points[x[0]][2],0)
		rospy.sleep(0.05)
		markers.publishLine(p1, p2, 'green', 0.05, 0)
		rospy.sleep(0.05)
		globals()['first'] = 0

		print "\nAt the end of callback"
		print "Connected Node: " + str(globals()['first_path_node'])
		print " to Node: " + str(val_id)
'''
########### END of Callbacks to handle setup ##########

def visualizer():
	rospy.init_node('visualizer', anonymous=True)
	rospy.Subscriber("/setup_requests", String, callback_requested)
	rospy.Subscriber("/setup_response", String, callback_setup)
	rospy.Subscriber("/safepoint_data", PoseStamped, callback_safepoint)
	rospy.Subscriber("/safepath_data", String, callback_safepath)
	#rospy.Subscriber("/safepath_data", PoseStamped, callback_safepath)
	#rospy.Subscriber("/goal_data", PoseStamped, callback_goal)
	rospy.spin()

if __name__ == '__main__':
	visualizer()