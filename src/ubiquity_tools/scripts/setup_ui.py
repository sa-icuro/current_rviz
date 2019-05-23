import sys 

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QDialog
from PyQt5.uic import loadUi
import roslib
import rospy
from std_msgs.msg import String
import termios, tty, os, time

pub_file_requestor = rospy.Publisher('setup_requests', String, queue_size = 1)


class myMenu(QDialog):

	def callback_files(self,data):
		rospy.sleep(0.5)
		msg_data = str(data.data).split(',')
		temp = []
		for x in msg_data:
			x = x.replace(" ","")
			x = x.replace("[","")
			x = x.replace("]","")
			x = x.replace("'","")
			temp.append(x)
		filenames = sorted(temp)
		for x in filenames:
			self.listWidget.addItem(QtWidgets.QListWidgetItem(x))

	def __init__(self):
		super(myMenu,self).__init__()
		filedir = os.path.dirname(os.path.abspath(__file__))
		print filedir
		uiFile_path = os.path.join(filedir,'setup.ui')
		loadUi(uiFile_path,self)
		rospy.Subscriber("route_file",String,self.callback_files)
		rospy.sleep(1)
		pub_file_requestor.publish("files")
		rospy.sleep(1)
		print "filename request published"
		self.save_session.clicked.connect(self.button_save)
		self.new_session.clicked.connect(self.button_new)
		self.safepoints.clicked.connect(self.button_safepoints)
		self.safepaths.clicked.connect(self.button_safepaths)
		self.listWidget.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
		self.route.clicked.connect(self.button_route)

	def mousePressEvent(self,event):
		super(myMenu, self).mousePressEvent(event)

	def mouseReleaseEvent(self, event):
		super(myMenu, self).mouseReleaseEvent(event)
		self.leftClick = False

	def button_save(self):
		msg = "save_session"
		pub.publish(msg)
		#rospy.sleep(0.2)
		self.close()

	def button_new(self):
		msg = "new_session"
		pub.publish(msg)
		#rospy.sleep(0.2)
		self.close()

	def button_safepoints(self):
		msg = "safepoints"
		pub.publish(msg)
		#rospy.sleep(0.2)
		self.close()
	
	def button_safepaths(self):
		msg = "safepaths"
		pub.publish(msg)
		#rospy.sleep(0.2)
		self.close()
	
	def button_route(self):
		item = self.listWidget.selectedItems()
		try:
			msg = str(self.listWidget.selectedItems()[0].text())
		except:
			msg = "goals_current.yaml"
		print msg
		pub.publish(msg)
		#rospy.sleep(0.2)
		self.close()


if __name__=='__main__':
	rospy.init_node('setup_requestor', anonymous=True)
	app = QApplication(sys.argv)
	window = myMenu()
	window.setWindowTitle('Ubiquity TeleOp Tool Setup')
	window.show()
	sys.exit(app.exec_())
