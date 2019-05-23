# ROS includes
import roslib
import rospy
from std_msgs.msg import String
import sys, termios, tty, os, time
import json
import yaml
import shutil
from os.path import isfile, join

pub = rospy.Publisher('setup_response', String, queue_size = 1)
pub2 = rospy.Publisher('route_file', String, queue_size = 1)

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

#Data Structures 
list_points = []
dict_paths = {}
list_goals = []

#Messages
msg_points = ''
msg_paths = ''
msg_route = ''
msg = ''

first_goal_var = True
points_file = '/home/ubuntu/catkin_ws/src/param/session_current/points_current.txt'
paths_file = '/home/ubuntu/catkin_ws/src/param/session_current/paths_current.json'	
route_file = '/home/ubuntu/catkin_ws/src/param/session_current/routes/goals_current.yaml'

#For setup options: New_Session, Load_Safepts_Only, and Load_Safepts_only
def manage_filesystem(flag):

	if(flag == 0):
		#Create New_Session
		#Create new directory and move files from session_current to new_session
		for x in range(1,100):
			dest = "/home/ubuntu/catkin_ws/src/param/session_" + str(x)
			if(not os.path.exists(dest)):
				print "NOW ATTEMPTING TO CREATE::: " + dest
				break

		os.makedirs(dest)
		source = '/home/ubuntu/catkin_ws/src/param/session_current'
		files = os.listdir(source)
		for f in files:
			shutil.move(source+'/'+f,dest)	
		#create empty points file, paths file, and /routes directory
		open(globals()['points_file'], 'w+')
		open(globals()['paths_file'], "w+")
		os.makedirs(source+'/routes')
		globals()['msg'] = "Creating new session"
	if(flag == 1):
		#Safepoints only
		#Create new directory and move files from session_current to new_session
		for x in range(1,100):
			dest = "/home/ubuntu/catkin_ws/src/param/session_" + str(x)
			if(not os.path.exists(dest)):
				print "NOW ATTEMPTING TO CREATE::: " + dest
				break
		
		os.makedirs(dest)
		source = '/home/ubuntu/catkin_ws/src/param/session_current'
		files = os.listdir(source)
		for f in files:
			shutil.move(source+'/'+f,dest)
		#import safepoints from last /session_current
		pt_src = dest + '/points_current.txt'
		shutil.copyfile(pt_src, (source+'/points_current.txt'))
		#create empty paths file and /route directory
		open(globals()['paths_file'], "w+")
		os.makedirs(source+'/routes')
	if (flag == 2):
		#Safepaths only
		#Rename goals_current as goals_x.yaml, and allow user to build new route
		for x in range(1,100):
			dest = "/home/ubuntu/catkin_ws/src/param/session_current/routes/goals_" + str(x)+".yaml"
			if(not os.path.exists(dest)):
				break
		try:
			shutil.move(globals()['route_file'], dest)
		except OSError:
			shutil.move(globals()['route_file'], "/home/ubuntu/catkin_ws/src/param/session_current/routes/goals_" + str(x+1)+".yaml")
		except:
			pass

		content = open(globals()['route_file'],"w+")
		content.write("goals:\n")

def load_points():
	try:
		with open(globals()['points_file'], 'rb') as f:
			for x in f.readlines():
				temp_tup1 = x.replace("(","")
				temp_tup2 = temp_tup1.replace(" ","")
				temp_tup1 = temp_tup2.replace(")","")
				temp_tup2 = temp_tup1.split(",")
				globals()['list_points'].append((int(temp_tup2[0]), float(temp_tup2[1]), float(temp_tup2[2]), float(temp_tup2[3])))
	except:
		print "No safepoints in session_current/points_current.txt, or file does not exist!"
		open(globals()['points_file'], 'w+')

	globals()['msg_points'] = '|'.join(str(x) for x in globals()['list_points'])

def load_paths():
	globals()['dict_paths'] = dict((k,[]) for k in range(len(globals()['list_points'])))
	try:
		with open(globals()['paths_file'], "r") as f:
			globals()['dict_paths'] = byteify(json.load(f))
	except:
		print "No safepaths in session_current/paths_current.json, or file does not exist!"
		f = open(globals()['paths_file'], "w+")

	for key, value in globals()['dict_paths'].iteritems():
		key = int(key)
		for x in value:
			x = int(x)
	
	globals()['msg_paths'] = '|'.join(str(key)+ ":" + str(val) for (key,val) in globals()['dict_paths'].iteritems())
	print "Safepaths message being sent: " + str(globals()['msg_paths'])

def load_goals(data):
	content = ""
	globals()['route_file'] = '/home/ubuntu/catkin_ws/src/param/session_current/routes/' + str(data)
	#if an exception is thrown here, create file and load empty route.
	try:	
		with open(globals()['route_file'], 'r') as content:
			yml = yaml.load(content)
	except:
		content = open(globals()['route_file'],"w+")
		content.write("goals:\n")
		yml = yaml.load(content)

	try:
		for x in range(0,len(yml['goals'])):
			val = yml['goals'][x].split(',')
			try:
				globals()['list_goals'].append((float(val[0]), float(val[1]), float(val[2]), float(val[3])))
			except:
				#Here 0.0 is the default wait time at each goal/safepoint
				globals()['list_goals'].append((float(val[0]), float(val[1]), float(val[2]), 0.0))
	except:
		print "No route data in session_current/routes/" + str(data) + "!"

	globals()['msg_route'] = '|'.join(str(x) for x in globals()['list_goals'])
	print "Route message being sent: " + str(globals()['msg_route'])

#Method to repond with the loaded data structures to topic /setup_response. 
#This message is recieved by visualizer.py to visualize session and setup Data Structures on the local machine.
def respond():
	final_msg = '~'.join((globals()['msg'],globals()['msg_points'],globals()['msg_paths'],globals()['msg_route']))
	pub = rospy.Publisher('setup_response', String, queue_size = 1)
	rospy.sleep(0.1)
	pub.publish(final_msg)
	rospy.sleep(0.1)


def callback(data):

	globals()['list_points'] = []
	globals()['dict_paths'] = {}
	globals()['list_goals'] = []
	
	if (str(data.data) == "files"):
		#Get list of file names and send them
		files = [f for f in os.listdir('/home/ubuntu/catkin_ws/src/param/session_current/routes')]
		pub2.publish(str(files))
		rospy.sleep(0.2)
	elif (str(data.data) == "save_session"):
		pass #Visualizer.py (local machine) will also recieve request message, and if 'save_session', will respond to 'save_data' (recieved HERE)
	elif (str(data.data) == "new_session"):
		manage_filesystem(0)
	elif (str(data.data) == "safepoints"):
		load_points()			
		manage_filesystem(1)
		respond()
	elif (str(data.data) == "safepaths"):
		manage_filesystem(2)
		load_points()
		load_paths()
		respond()
	elif (str(data.data)[-5:] == ".yaml"):
		print str(data.data)
		load_points()
		load_paths()
		load_goals(str(data.data))
		respond()
	else: 
		print "Unknown Input: " + str(data.data)
		pass

def callback_save(data):
	#Temp DS's for saving
	new_points = []
	new_paths = {}
	new_goals = []

	#Parse Save Data string into temp data structures viz str.split()
	info = str(data.data).split('~')
	try:
		temp_points = info[0].split('|')
		temp_paths = info[1].split('|')
		temp_goals = info[2].split('|')
	except:
		print "Issue with temp data structues"

	#properly build list of points to save from temp_points (if not EMPTY)
	try:
		for x in temp_points:
			temp_tup1 = x.replace("(","")
			temp_tup1 = temp_tup1.replace(" ","")
			temp_tup1 = temp_tup1.replace(")","")
			temp_tup1 = temp_tup1.split(",")
			new_points.append((int(temp_tup1[0]), float(temp_tup1[1]), float(temp_tup1[2]), float(temp_tup1[3])))
		print "safepoint data to save: " + str(new_points)
		print "Done reading safepoints to save"
	except:
		print "No Safepoints set yet!"
		#Exception will be caught if list of points from User is empty, 
		#session_current's safepoint data will not be cleared.

	#properly build dict of paths to save from temp_paths (if not EMPTY)
	try:
		for item in temp_paths:
			print item
			tmp = str(item).split(':')
			val_id = int(tmp[0])
			new_paths[str(val_id)] = []
			#print "tmp[1]: " + tmp[1] + "\n"
			tmp = str(tmp[1]).strip('[')
			tmp = tmp.strip(']')
			tmp = tmp.split(",")
			for x in tmp:
				x = str(x).strip(" ")
				#print "\nx: " + str(x) + "\n" 
				try:
					#print "Node: " + str(x) + " connected to node " + str(val_id) + "\n"
					new_paths[str(val_id)].append(int(x))
				except:
					continue
					#To catch error thrown by Null values
					#print "issue with adding Node " + str(x) + " node to " + str(val_id)
		print "safepath data to save: " + str(new_paths)
		print "Done reading safepaths to save"
	except:
		print "No Safepaths set yet!"
		#Exception will be caught if dict of paths from User is empty, 
		#session_current's safepath data will not be cleared.

	#properly build list of goals to save from temp_goals (if not EMPTY)
	print "Pre-Parse route data recieved to SAVE: " + str(temp_goals)	
	for x in temp_goals:
		x = x.strip("(")
		x = x.strip(")")
		x = x.split(",")
		for item in x:
			item.replace(" ","")
		if (x == ['']):
			print "No Route set yet!"
			break
		else:
			new_goals.append((float(x[0]), float(x[1]), float(x[2]), float(x[3])))
	print "completed new_goals: " + str(new_goals)	
	
	#Finally, Save all data (override because sessions have been properly configured)
	with open(globals()['points_file'], 'w+') as f:
		for s in new_points:
			f.write(str(s) + '\n')

	with open(globals()['paths_file'], 'w+') as f:
		json.dump(new_paths, f)

	f = open(globals()['route_file'],"w+")
	f.write("goals:\n")
	for item in new_goals:
		f.write("- %f, %f, %f\n" % (item[0], item[1], item[2]))
	f.close()
	print "Done saving files"

	#Override current working DSs for any further *Setup* work done for 'session_current' 
	#(ie. loading safepts, safepaths, exclusively, then more edits, etc.)
	globals()['list_points'] = new_points
	globals()['dict_paths'] = new_paths
	globals()['list_goals'] = new_goals


def listener():
	rospy.init_node('setup_handler', anonymous=True)
	rospy.Subscriber("/setup_requests", String, callback)
	rospy.Subscriber("/save_data", String, callback_save)
	rospy.spin()

if __name__ == '__main__':
	listener()
