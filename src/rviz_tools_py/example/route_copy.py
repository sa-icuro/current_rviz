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
import numpy
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

global X 
global Y
markers = rviz_tools.RvizMarkers('/map', 'visualization_marker')
path_route = []
path_populate = []
path_safe = []



list_points = []
if(os.path.getsize('/home/centaur/catkin_ws/src/magni_goal_sender/param/points.txt')) > 0:
    with open('/home/centaur/catkin_ws/src/magni_goal_sender/param/points.txt', 'r') as f:
        list_points = [line.rstrip('\n') for line in f]
print "LIST_POINTS: " + str(list_points)
#list_paths = []

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

paths = {}
if(os.path.getsize('/home/centaur/catkin_ws/src/magni_goal_sender/param/paths.json')) > 0:
    with open('/home/centaur/catkin_ws/src/magni_goal_sender/param/paths.json', "r") as f:
        paths = byteify(json.load(f))
print "PATHS: " + str(paths)

path_1 = 0
last_path = 0
last_val = 0
curr_val = 0
vertex_count = len(list_points)
edge_count = 0

# Define exit handler
def cleanup_node():
    print "Shutting down node!"
    
    with open('/home/centaur/catkin_ws/src/magni_goal_sender/param/points.txt', 'w+') as f:
        for s in list_points:
            f.write(str(s) + '\n')

    with open('/home/centaur/catkin_ws/src/magni_goal_sender/param/paths.json', 'w+') as f:
        json.dump(paths, f)

rospy.on_shutdown(cleanup_node)


def callback(msg):
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

def isclose(a, b, rel_tol=1e-05, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

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

    for x in list_points:
        temp_tup1 = x.replace("(","")
        temp_tup2 = temp_tup1.replace(" ","")
        tup_root = temp_tup2.replace(")","")
        data = tup_root.split(',')
        val_id = data[0]
        val_x = data[1]
        val_y = data[2]
        
        #print "x[2]: " + str(x[2])
        #print "position_x: " + str(posx)
        #print "position_y: " + str(y)
        #print str(x) + "     vs:: x= " + str(position.x) + ", y= " + str(position.y)
        print "x isClose: " + str(isclose(float(val_x),float(position.x))) + "y isClose: " + str(isclose(float(val_y),float(position.y)))
        if (isclose(float(val_x),float(position.x))) and (isclose(float(val_y),float(position.y))):
            print "MATHCH!!! with value# " + str(val_id)
            globals()['curr_val'] = val_id

    #For the first one
    if((globals()['path_1'] == 0)):
        globals()['last_val'] = globals()['curr_val']
        old = (val_id,val_x,val_y)
        globals()['path_1'] = 1
    elif(globals()['path_1'] == 1):
        #Add the ID of the connected node
        print "last val: " + str(old[0])
        print "curr_val" + str(val_id)
        print "path_1" + str(globals()['path_1'])
        paths[globals()['last_val']].append(globals()['curr_val'])
        globals()['last_val'] = globals()['curr_val']
        p1 = Point(float(old[1]),float(old[2]),0)
        p2 = Point(float(val_x),float(val_y),0)
        markers.publishLine(p1, p2, 'green', 0.05, 0)
        globals()['path_1'] = 0

    print paths

    f = open("/home/centaur/catkin_ws/src/magni_goal_sender/param/safe_paths.txt","a+")
    f.write("%f, %f, %f\n" % (position.x, position.y, quat.z))
    f.close()

def callback_goal(msg):
    rospy.loginfo("Received a Goal!")

    # Copying for simplicity
    position = msg.pose.position
    quat = msg.pose.orientation
    rospy.loginfo("Goal data (Appending to route): [ %f, %f, %f ]"%(position.x, position.y, quat.z))

    # Publish a sphere using a ROS Pose
    P = Pose(Point(position.x,position.y,0),Quaternion(0,0,quat.z,1))
    scale = Vector3(0.3,0.3,0.3) # diameter
    color = 'blue'
    markers.publishSphere(P, color, scale, 0) # pose, color, scale, lifetime

    path_route.append( Point(position.x,position.y,0) )
    width = 0.055
    markers.publishPath(path_route, 'white', width, 0) # path, color, width, lifetime
    
    if(os.path.lexists('/home/centaur/catkin_ws/src/magni_goal_sender/param/final_goals.yaml')):
        f = open("/home/centaur/catkin_ws/src/magni_goal_sender/param/final_goals.yaml","a+")
    else:
        f = open("/home/centaur/catkin_ws/src/magni_goal_sender/param/final_goals.yaml","a+")
        f.write("goals: \n")
        
    f.write("- %f, %f, %f, 0\n" % (position.x, position.y, quat.z))
    f.close()

def populate():
    print "LOADING POINTS..."
    count = 0

    for item in list_points:
        if (count == 0):
            rospy.sleep(0.2)
        print item
        print "PRINTING " + str(count) 
        newitem = item.replace("(","")
        temp = newitem.replace(" ","")
        item = temp.replace(")","")
        data = item.split(',')
        val_id = data[0].replace("'","")
        val_x = data[1].replace("'","")
        val_y = data[2].replace("'","")
        val_angle = data[3].replace("'","")

        P = Pose(Point(float(val_x),float(val_y),0),Quaternion(0,0,float(val_angle),1))
        scale = Vector3(0.275,0.275,0.275) # diameter

        rospy.sleep(0.04)
        markers.publishSphere(P, 'purple', scale, 0) # pose, color, scale, lifetime
        markers.publishCube(P, 'orange', 0.2, 0) # pose, color, scale, lifetime
        rospy.sleep(0.04)
        count += 1

    print "...DONE LOADING POINTS"
    
    print "Loading PATHS:"

    for key, value in paths.iteritems():
        print "key: " + str(key)

        if(len(value) > 0):
            print "key (inside method): " + str(key)
            tup_root = list_points[int(key)]
            temp_tup1 = tup_root.replace("(","")
            temp_tup2 = temp_tup1.replace(" ","")
            tup_root = temp_tup2.replace(")","")
            data = tup_root.split(',')
            val_x = data[1]
            val_y = data[2]
            p1 = Point(float(val_x),float(val_y),0)

            for x in value:
                tup_connect = list_points[x]
                temp_tup1 = tup_connect.replace("(","")
                temp_tup2 = temp_tup1.replace(" ","")
                tup_connect = temp_tup2.replace(")","")
                data = tup_connect.split(',')
                val_x = data[1]
                val_y = data[2]
                p2 = Point(float(val_x),float(val_y),0)
                markers.publishLine(p1, p2, 'green', 0.05, 0)

    print "...DONE LOADING PATHS"


def listener():
    rospy.init_node('listener', anonymous=True)
    populate()
    rospy.Subscriber("/safepointz", PoseStamped, callback)
    rospy.Subscriber("/goalz", PoseStamped, callback_goal)
    rospy.Subscriber("/safepathz", PoseStamped, callback_path)

    rospy.spin()

if __name__ == '__main__':
    listener()