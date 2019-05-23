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

##### END OF FUNCTIONS TO LOAD FROM FILE #####

#For Verbose Funionality set to True
VERBOSE = True

#Data Structures
list_points = [] #(Node_ID, Position_X, Position_Y, Angle?/Quat_Z? )
dict_paths = {}  # Parent_Node_ID: [Connected_Node_IDs]
list_goals = []  #(Position_X, Position_Y, Quat_Z, Wait_Time)

route_path = []  #(Position_X, Position_Y, 0 ) to visualize route paths (WHITE LINE)

#Global Variables/Setup for Callback_setup
populated = 0
request_type = ''

#Global Variables for Callback_safepoint (to create entries in dict_paths{})
node_count = 0

#Global Variables for Callback_path
'''
Only need to remember the parent node's ID in the case we're setting the child, then both reset
#If equal to -1, then a parent node ID has not been set yet (1st safepath-pair member)
If equal to value >= 0, then a parent node ID has been set, and a child is now being connected (2nd safepath-pair member)
'''
parent_node_ID = -1 
p_pose = 0
c_pose = 0

#Global Variables/Setup for Callback_route 
last_route_id = -1
path_route = []

########### Loading Tools ##########
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

	if globals()['VERBOSE']:
		print "\nLOADING ROUTE..."
		print "len_route: " + str(len(list_goals))
		print list_goals

	if(len(list_goals) > 0):
		for item in list_goals[:-1]:
			P = Pose(Point(item[0],item[1],0),Quaternion(0,0,item[2],1))
			scale = Vector3(0.33,0.33,0.33) # diameter
			rospy.sleep(0.02)
			#markers.publishSphere(P, 'purple', scale, 0) # pose, color, scale, lifetime
			markers.publishSphere(P, 'blue', scale, 0) # pose, color, scale, lifetime
			rospy.sleep(0.01)
			route_path.append(Point(item[0],item[1],0))
			markers.publishPath(route_path, 'white', 0.1, 0)
			rospy.sleep(0.02)
			
		P = Pose(Point(list_goals[-1][0],list_goals[-1][1],0),Quaternion(0,0,list_goals[-1][2],1))
		scale = Vector3(0.33,0.33,0.33) # diameter
		rospy.sleep(0.02)
		#markers.publishSphere(P, 'purple', scale, 0) # pose, color, scale, lifetime
		markers.publishSphere(P, 'yellow', scale, 0) # pose, color, scale, lifetime
		rospy.sleep(0.01)
		route_path.append(Point(list_goals[-1][0],list_goals[-1][1],0))
		markers.publishPath(route_path, 'white', 0.1, 0)
		rospy.sleep(0.02)

	if globals()['VERBOSE']:
		print "...DONE LOADING ROUTE\n"

########### END of Loading Tools ##########

########### Callbacks to handle setup ##########
def callback_requested(data):
	globals()['request_type'] = str(data.data)
	
	if (str(data.data) == "save_session"):
		msg_points = '|'.join(str(x) for x in globals()['list_points'])
		msg_paths = '|'.join(str(key)+ ":" + str(val) for (key,val) in globals()['dict_paths'].iteritems())
		msg_route = '|'.join(str(x) for x in globals()['list_goals'])
		final_msg = '~'.join((msg_points,msg_paths,msg_route))
		pub.publish(final_msg)
		rospy.sleep(0.1)
		if globals()['VERBOSE']:
			print "Saving data onto Magni"
	else:
		if globals()['VERBOSE']:
			print "Requested: " + str(data.data)
		pass
	
def callback_setup(data):
	#If setup is called, it is assumed user is reloading the session in some way so we will 
	#reset all datastructures and repopulate
	globals()['list_points'] = []
	globals()['dict_paths'] = {}
	globals()['list_goals'] = []
	globals()['route_path'] = []

	info = str(data.data).split('~')
	if globals()['VERBOSE']:
		print "Inside callback_setup: "
		print "Visualizer_callback_setup_data: " + str(info)
	temp_points = info[1].split('|')
	temp_paths = info[2].split('|')
	temp_goals = info[3].split('|')

	check_points_empty = True
	check_paths_empty = True
	check_route_empty = True

	if(temp_points != [''] and (str(temp_points) != "['']")):
		if globals()['VERBOSE']:
			print "points = true"
		check_points_empty = False

	if((temp_paths != {''}) and (str(temp_paths) != "['']")):
		if globals()['VERBOSE']:
			print "paths = true"
		check_paths_empty = False

	if(temp_goals != [''] and (str(temp_goals) != "['']")):
		if globals()['VERBOSE']:
			print "route = true"
		check_route_empty = False

	if (globals()['request_type'] == "safepoints"):
		if(check_points_empty == False):
			for x in temp_points:
				temp_tup1 = x.replace("(","")
				temp_tup1 = temp_tup1.replace(" ","")
				temp_tup1 = temp_tup1.replace(")","")
				temp_tup1 = temp_tup1.split(",")
				globals()['list_points'].append((int(temp_tup1[0]), float(temp_tup1[1]), float(temp_tup1[2]), float(temp_tup1[3])))
				globals()['dict_paths'][str(int(temp_tup1[0]))] = []
		else:
			print "Error: Empty safepoints at ...session_current/points_current.txt"

	if (globals()['request_type'] == "safepaths"):
		if(check_points_empty == False):
			for x in temp_points:
				temp_tup1 = x.replace("(","")
				temp_tup1 = temp_tup1.replace(" ","")
				temp_tup1 = temp_tup1.replace(")","")
				temp_tup1 = temp_tup1.split(",")
				globals()['list_points'].append((int(temp_tup1[0]), float(temp_tup1[1]), float(temp_tup1[2]), float(temp_tup1[3])))
				globals()['dict_paths'][str(int(temp_tup1[0]))] = []
			if(check_paths_empty == False):
				for item in temp_paths:
					item = item.replace('"','')
					item = item.replace('\'','')

					tmp = item.split(':')
					val_id = int(str(tmp[0]).strip('\''))
					tmp[1] = tmp[1].replace('[','')
					tmp[1] = tmp[1].replace(']','')
					tmp = tmp[1].split(',')
					#print "tmp[1]: " + tmp[1] + "\n"
					globals()['dict_paths'][str(val_id)] = []

					for x in tmp:
						x = str(x).strip(" ")
						#print "\nx: " + str(x) + "\n" 
						try:
							if globals()['VERBOSE']:
								print "Node: " + str(x) + " connected to node " + str(val_id) + "\n"
							globals()['dict_paths'][str(val_id)].append(int(x))
						except:
							#To catch error thrown by Null values
							if globals()['VERBOSE']:
								print "issue with adding Node " + str(x) + " node to " + str(val_id)
							continue
			else:
				print "Error: Safepoints exist, but Empty safepaths at ...session_current/paths_current.txt"
		else:
			print "Error: Empty safepoints at ...session_current/points_current"

	if (globals()['request_type'][-5:] == ".yaml"):
		if(check_points_empty == False):
			for x in temp_points:
				temp_tup1 = x.replace("(","")
				temp_tup1 = temp_tup1.replace(" ","")
				temp_tup1 = temp_tup1.replace(")","")
				temp_tup1 = temp_tup1.split(",")
				globals()['list_points'].append((int(temp_tup1[0]), float(temp_tup1[1]), float(temp_tup1[2]), float(temp_tup1[3])))

			if(check_paths_empty == False):
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

				if(check_route_empty == False):
					for x in temp_goals:
						temp_tup1 = x.replace("(","")
						temp_tup1 = temp_tup1.replace(" ","")
						temp_tup1 = temp_tup1.replace(")","")
						temp_tup1 = temp_tup1.split(",")
						globals()['list_goals'].append((float(temp_tup1[0]), float(temp_tup1[1]), float(temp_tup1[2]), float(temp_tup1[3])))
				else:
					print "Error: Safepoints & safepaths exist, but Empty route at ...session_current/routes/"+globals()['request_type']
			else:
				print "Error: Safepoints exist, but Empty safepaths at ...session_current/paths_current.txt"
		else:
			print "Error: Empty safepoints at ...session_current/points_current"
	
	if (globals()['VERBOSE']):	
		print "\nlist_points: " + str(globals()['list_points'])
		print "\ndict_paths: " + str(dict_paths)
		print "\nlist_goals: " + str(globals()['list_goals'])

	#Before ending the callback, delete on-screen markers and re-populate new data
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
	list_points.append(tup)
	dict_paths[str(globals()['node_count'])] = []
	(globals()['node_count']) += 1
	if globals()['VERBOSE']:
		print "\n~~~~~~~~ Safepoint Tool ~~~~~~~~"
		print "Appending safepoint: " +  str(tup) + "!"
		print "node_count = " + str(node_count)
		#print list_points
		print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
	
def callback_safepath(msg):
	items = str(msg.data).split(" ")
	x, y, z  = float(items[0]), float(items[1]), float(0)
	current_node_ID = 0
	e_flag = 0

	#Locate Node_ID given x and Y point
	for point in list_points:
		if (isclose(point[1],x) and isclose(point[2],y)):
			current_node_ID = point[0]
			z = point[3]
			if globals()['VERBOSE']:
				print "\n~~~~~~~~~ Safepath Tool ~~~~~~~~~~"
				print "MATCH!!! with Node_ID# " + str(point[0])
			break
	
	if globals()['VERBOSE']:
		print "Parent_node_ID: " + str(globals()['parent_node_ID'])
		print "Current_node_ID: " + str(current_node_ID)
		if (globals()['parent_node_ID'] == -1):
			print "Setting Parent node."
		else:
			print "Connecting Child node."
		print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"


	if (globals()['parent_node_ID'] == -1):
		#Meaning parent node has not been set yet...

		#Create temporary parent node w/ ROS Pose
		globals()['p_pose'] = Pose(Point(x,y,0),Quaternion(0,0,z,1))
		markers.publishSphere(globals()['p_pose'], 'grey', Vector3(0.25,0.25,0.25), 0) # pose, color, scale, lifetime	

		globals()['parent_node_ID'] = current_node_ID
		if globals()['VERBOSE']:
			print "Setting Parent (1st value of safepath-pair) node: " + str(current_node_ID) + "\n"
			print dict_paths
	elif(globals()['parent_node_ID'] >= 0):
		globals()['c_pose'] = Pose(Point(x,y,0),Quaternion(0,0,z,1))
		#Meaning Parent node has been set, but child node has not...

		#If current node (child) is not a listed connection for the parent node, then append.
		#Exception catches issues with data being parsed in as strings and not converted properly, just in case. 
		try:
			if(not((current_node_ID) in dict_paths[str(globals()['parent_node_ID'])])):
				dict_paths[str(globals()['parent_node_ID'])].append((current_node_ID))
			e_flag = 0
		except KeyError:
			e_flag = 1
			if(not((current_node_ID) in dict_paths[globals()['parent_node_ID']])):
				dict_paths[(globals()['parent_node_ID'])].append((current_node_ID))

		#If parent node is not a listed connection for the current node (child), then append.
		#Exception catches issues with data being parsed in as strings and not converted properly, just in case.
		try:
			if(not((globals()['parent_node_ID']) in dict_paths[str(current_node_ID)])):		
				dict_paths[str(current_node_ID)].append((globals()['parent_node_ID']))
			e_flag = 0
		except KeyError:
			e_flag = 1
			if(not((globals()['parent_node_ID']) in dict_paths[(current_node_ID)])):		
				dict_paths[(current_node_ID)].append((globals()['parent_node_ID']))

		if globals()['VERBOSE']:
			if (e_flag == 1):		
				print "(Exception) Connecting Child node (= " + str(current_node_ID) + ")"
			else:
				print "Connecting Child node (= " + str(current_node_ID) + ")"
			print "to parent Node: " + str(globals()['parent_node_ID']) + "\n"
			#print "Parent is an 'int': " + str(type(globals()['parent_node_ID']) is int)
			#print "Child is a 'int': " + str(type(current_node_ID) is int) + "\n"

		#Create and connect Final Parent and Child nodes
		markers.publishSphere(globals()['p_pose'], 'purple', Vector3(0.275,0.275,0.275), 0) # pose, color, scale, lifetime	
		markers.publishSphere(globals()['c_pose'], 'purple', Vector3(0.275,0.275,0.275), 0) # pose, color, scale, lifetime	
		parent = Point(list_points[globals()['parent_node_ID']][1],list_points[globals()['parent_node_ID']][2],0)
		child = Point(list_points[current_node_ID][1],list_points[current_node_ID][2],0)
		rospy.sleep(0.05)
		markers.publishLine(parent, child, 'green', 0.05, 0)
		rospy.sleep(0.05)

		if globals()['VERBOSE']:
			print "Safepath-Pair successfully created!!!"
			print "Safepath-pair: " + str(globals()['parent_node_ID']) + "-" + str(current_node_ID) + " (Parent-Child) appended to safepath dict."
			print "~~~~~~ End of Safepath Tool ~~~~~~\n"
			print dict_paths

		#Reset parent variable to False, to indicate a new safepath-pair can be created now
		globals()['parent_node_ID'] = -1

def callback_route(msg):
	items = str(msg.data).split(" ")
	x = float(items[0])
	y = float(items[1])
	z = float(0)
	current_node_ID = 0

	#Locate Node_ID given x and Y point
	for point in globals()['list_points']:
		if (isclose(point[1],x) and isclose(point[2],y)):
			current_node_ID = point[0]
			z = point[3]
			if globals()['VERBOSE']:
				print "\n~~~~~~~~~~ Route Tool ~~~~~~~~~~"
				print "MATCH!!! with Node_ID# " + str(point[0])
				print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
			break

	P = Pose(Point(x,y,0),Quaternion(0,0,z,1))
	scale = Vector3(0.33,0.33,0.33) # diameter
	color = 'blue'

	if (globals()['last_route_id'] == -1):
		#Meaning this is the first route goal being set:
		#tup(goal_position_x, goal_position_y, goal_quat_z, goal_wait_time)
		#goal_wait_time default set to '0'
		globals()['list_goals'].append((x,y,z,float(0)))
		markers.publishSphere(P, 'yellow', scale, 0) # pose, color, scale, lifetime
		rospy.sleep(0.01)
		path_route.append( Point(x,y,0) )
		globals()['last_route_id'] = current_node_ID
		if globals()['VERBOSE']:
			print "First route goal (Node #" + str(current_node_ID)+ ") appended!"
			print "~~~~~~~ End of Route Tool ~~~~~~\n"
	else:
		#Meaning a route goal has already been set:
		try:
			if(current_node_ID in dict_paths[str(globals()['last_route_id'])]):
				#Check if current node has a safepath from last_node and can be appended to route
					markers.publishSphere(Pose(Point(globals()['list_points'][last_route_id][1],globals()['list_points'][last_route_id][2],0),Quaternion(0,0,globals()['list_points'][last_route_id][3],1)), color, scale, 0) # pose, color, scale, lifetime
					markers.publishSphere(P, 'yellow', scale, 0) # pose, color, scale, lifetime
					rospy.sleep(0.01)
					path_route.append( Point(x,y,0) )
					markers.publishPath(path_route, 'white', 0.1, 0) # path, color, width, lifetime
					if globals()['VERBOSE']:
						print "\nFound current_node (" + str(current_node_ID) + ") in Node " + str(globals()['last_route_id']) + "'s list of safepaths"
						print "Successfully appending current node to route!"
						print "~~~~~~~ End of Route Tool ~~~~~~\n"
					globals()['last_route_id'] = current_node_ID
					globals()['list_goals'].append((x,y,z,float(0)))
			else:
				if globals()['VERBOSE']:
					print "No safepath between current_node (" + str(current_node_ID) + ") and last_node (" + str(globals()['last_route_id']) + ")!"
					print "~~~~~~~ End of Route Tool ~~~~~~\n" 
				pass
		except:
			#Exception catches issues with data being parsed in as strings and not converted properly, just in case.
			try:
				if(current_node_ID in dict_paths[globals()['last_route_id']]):
					#Check if current node has a safepath from last_node and can be appended to route
						markers.publishSphere(Pose(Point(globals()['list_points'][last_route_id][1],globals()['list_points'][last_route_id][2],0),Quaternion(0,0,globals()['list_points'][last_route_id][3],1)), color, scale, 0) # pose, color, scale, lifetime
						markers.publishSphere(P, color, scale, 0) # pose, color, scale, lifetime
						rospy.sleep(0.01)
						path_route.append( Point(x,y,0) )
						markers.publishPath(path_route, 'white', 0.1, 0) # path, color, width, lifetime
						if globals()['VERBOSE']:
							print "\n(Exception)Found current_node (" + str(current_node_ID) + ") in Node " + str(globals()['last_route_id']) + "'s list of safepaths"
							print "Successfully appending current node to route!"
							print "~~~~~~~ End of Route Tool ~~~~~~\n"
						globals()['last_route_id'] = current_node_ID
						globals()['list_goals'].append((x,y,z,float(0)))
				else:
					if globals()['VERBOSE']:
						print "(Exception)No safepath between current_node (" + str(current_node_ID) + ") and last_node (" + str(globals()['last_route_id']) + ")!"
						print "~~~~~~~ End of Route Tool ~~~~~~\n" 
					pass
			except:
				if globals()['VERBOSE']:
						print "(Exception)No safepath between current_node (" + str(current_node_ID) + ") and last_node (" + str(globals()['last_route_id']) + ")!"
						print "~~~~~~~ End of Route Tool ~~~~~~\n" 
				pass
'''
def callback_run(msg):
	pass
'''
########### END of Callbacks to handle setup ##########

def visualizer():
	rospy.init_node('visualizer', anonymous=True)
	rospy.Subscriber("/setup_requests", String, callback_requested)
	rospy.Subscriber("/setup_response", String, callback_setup)
	rospy.Subscriber("/safepoint_data", PoseStamped, callback_safepoint)
	rospy.Subscriber("/safepath_data", String, callback_safepath)
	rospy.Subscriber("/route_data", String, callback_route)
	rospy.spin()

if __name__ == '__main__':
	visualizer()