#!/usr/bin/env python3
#
# Author - Tony Willett
# Date - 20 06 2022
# Description - Diagnostic tool for ClubBot2
# Monitor published hardware nodes and display in GUI
#
# refer to http://tkdocs.com/tutorial/firstexample.html
#

from tkinter import *
from tkinter import ttk

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import UInt64
from sensor_msgs.msg import Range


class debugGUI:
	"""Diagnostic GUI for ClubBot2 functions.
	"""

	def __init__(self, gui):

		gui.title("ClubBot II Diagnostics")
		gui.columnconfigure(0, weight=1)
		gui.rowconfigure(0, weight=1)
		gui.geometry("300x450")
		mainframe = ttk.Frame(gui, padding="10 10", borderwidth=8, relief='sunken')
		mainframe.grid(column=0, row=0, sticky=(N, W, S, E))
		mainframe.columnconfigure(0, weight=1)
		mainframe.rowconfigure(0, weight=0)
		# Front Bumper Frame
		fBumper = ttk.Frame(mainframe, padding="5 5", borderwidth=4, relief='sunken')
		fBumper.grid(column=0, row=0, sticky=(N, W, E, S))
		fBumper.columnconfigure(0, weight=1)
		fBumper.columnconfigure(1, weight=1)
		fBumper.columnconfigure(2, weight=1)
		fBumper.rowconfigure(0, weight=1)
		# Sonar Frame
		snrFrame = ttk.Frame(mainframe, padding='5 5', borderwidth=4, relief='raised')
		snrFrame.grid(column=0, row=1, sticky=(N, W, S, E))
		snrFrame.columnconfigure(0, weight=1)
		snrFrame.columnconfigure(1, weight=1)
		snrFrame.columnconfigure(2, weight=1)
		snrFrame.columnconfigure(3, weight=1)
		snrFrame.rowconfigure(0, weight=1)
		snrlFrame = ttk.Frame(snrFrame, padding='10 2', borderwidth=4, relief='sunken')
		snrlFrame.grid(column=0, row=2, sticky=(N, W, S, E))
		snrlFrame.columnconfigure(0, weight=1)
		snrlmFrame = ttk.Frame(snrFrame, padding='10 2', borderwidth=4, relief='sunken')
		snrlmFrame.grid(column=1, row=2, sticky=(N, W, S, E))
		snrlmFrame.columnconfigure(0, weight=1)
		snrrmFrame = ttk.Frame(snrFrame, padding='10 2', borderwidth=4, relief='sunken')
		snrrmFrame.grid(column=2, row=2, sticky=(N, W, S, E))
		snrrmFrame.columnconfigure(0, weight=1)
		snrrFrame = ttk.Frame(snrFrame, padding='10 2', borderwidth=4, relief='sunken')
		snrrFrame.grid(column=3, row=2, sticky=(N, W, S, E))
		snrrFrame.columnconfigure(0, weight=1)
		# Encoder Frame
		encFrame = ttk.Frame(mainframe, padding='5 5', borderwidth=4,relief='raised')
		encFrame.grid(column=0, row=2, sticky=(N,W,S,E))
		encFrame.columnconfigure(0, weight=1)
		encFrame.columnconfigure(1, weight=1)
		encFrame.rowconfigure(0, weight=1)
		LencFrame = ttk.Frame(encFrame, padding='10 2', borderwidth=8, relief='raised')
		LencFrame.grid(column=0, row=1, sticky=(N,W,S,E))
		LencFrame.columnconfigure(0, weight=1)
		LFencFrame = ttk.Frame(LencFrame, padding='5 2', borderwidth=4, relief='raised')
		LFencFrame.grid(column=0, row=2, sticky=(N,W,S,E))
		LFencFrame.columnconfigure(0, weight=1)
		LBencFrame = ttk.Frame(LencFrame, padding='5 2', borderwidth=4, relief='sunken')
		LBencFrame.grid(column=0, row=4, sticky=(N,W,S,E))
		LBencFrame.columnconfigure(0, weight=1)
		RencFrame = ttk.Frame(encFrame, padding='10 2', borderwidth=8, relief='raised')
		RencFrame.grid(column=1, row=1, sticky=(N,W,S,E))
		RencFrame.columnconfigure(0, weight=1)
		RFencFrame = ttk.Frame(RencFrame, padding='5 2', borderwidth=4, relief='raised')
		RFencFrame.grid(column=0, row=2, sticky=(N,W,S,E))
		RFencFrame.columnconfigure(0, weight=1)
		RBencFrame = ttk.Frame(RencFrame, padding='5 2', borderwidth=4, relief='sunken')
		RBencFrame.grid(column=0, row=4, sticky=(N,W,S,E))
		RBencFrame.columnconfigure(0, weight=1)
		# Back bumper frame
		bBumper = ttk.Frame(mainframe, padding='5 5', borderwidth=4, relief='sunken')
		bBumper.grid(column=0, row=3, sticky=(N, W, E, S))
		bBumper.columnconfigure(0, weight=1)
		bBumper.columnconfigure(1, weight=1)
		bBumper.columnconfigure(2, weight=1)
		bBumper.rowconfigure(0, weight=1)

		self.bumpLF = StringVar()
		self.bumpMF = StringVar()
		self.bumpRF = StringVar()
		self.bumpLB = StringVar()
		self.bumpMB = StringVar()
		self.bumpRB = StringVar()
		self.bumpLF.set ('o')
		self.bumpMF.set ('o')
		self.bumpRF.set ('o')
		self.bumpLB.set ('o')
		self.bumpMB.set ('o')
		self.bumpRB.set ('o')
		self.sonarL = StringVar()
		self.sonarLM = StringVar()
		self.sonarRM = StringVar()
		self.sonarR = StringVar()
		self.sonarL.set ('0')
		self.sonarLM.set ('0')
		self.sonarRM.set ('0')
		self.sonarR.set ('0')
		self.encLF = StringVar()
		self.encLB = StringVar()
		self.encRF = StringVar()
		self.encRB = StringVar()
		self.encLF.set (0)
		self.encLB.set (0)
		self.encRF.set (0)
		self.encRB.set (0)
		self._bumperLF = rospy.Subscriber('bpr_lf', Bool, self.bumperLFCB)
		self._bumperMF = rospy.Subscriber('bpr_mf', Bool, self.bumperMFCB)
		self._bumperRF = rospy.Subscriber('bpr_rf', Bool, self.bumperRFCB)
		self._bumperLB = rospy.Subscriber('bpr_lb', Bool, self.bumperLBCB)
		self._bumperMB = rospy.Subscriber('bpr_mb', Bool, self.bumperMBCB)
		self._bumperRB = rospy.Subscriber('bpr_rb', Bool, self.bumperRBCB)
		self._sonarL = rospy.Subscriber('snr_1', Range, self.sonarLCB)
		self._sonarLM = rospy.Subscriber('snr_2', Range, self.sonarLMCB)
		self._sonarRM = rospy.Subscriber('snr_3', Range, self.sonarRMCB)
		self._sonarR = rospy.Subscriber('snr_4', Range, self.sonarRCB)
		self._encoderLF = rospy.Subscriber('enc_lf', UInt64, self.encoderLFCB)
		self._encoderLB = rospy.Subscriber('enc_lb', UInt64, self.encoderLBCB)
		self._encoderRF = rospy.Subscriber('enc_rf', UInt64, self.encoderRFCB)
		self._encoderRB = rospy.Subscriber('enc_rb', UInt64, self.encoderRBCB)

		# Text
		# Front Bumper
		ttk.Label(fBumper, text="Front Bumper").grid(column=0, columnspan=3, row=0)
		ttk.Label(fBumper, text="Left").grid(column=0, row=1)
		ttk.Label(fBumper, text="Middle").grid(column=1, row=1)
		ttk.Label(fBumper, text="Right").grid(column=2, row=1)
		ttk.Label(fBumper, textvariable=self.bumpLF).grid(column=0, row=2)
		ttk.Label(fBumper, textvariable=self.bumpMF).grid(column=1, row=2)
		ttk.Label(fBumper, textvariable=self.bumpRF).grid(column=2, row=2)

		# Sonar
		snrtxt = Label(snrFrame, text="Sonar").grid(column=0, columnspan=4, row=0)
		lsnrtxt = Label(snrFrame, text='L').grid(column=0, row=1)
		lmsnrtxt = Label(snrFrame, text='LM').grid(column=1, row=1)
		rmsnrtxt = Label(snrFrame, text='RM').grid(column=2, row=1)
		rsnrtxt = Label(snrFrame, text='R').grid(column=3, row=1)
		lsnrVal = Label(snrlFrame, textvariable=self.sonarL).grid(column=0, row=2)
		lmsnrVal = Label(snrlmFrame, textvariable=self.sonarLM).grid(column=1, row=2)
		rmsnrVal = Label(snrrmFrame, textvariable=self.sonarRM).grid(column=2, row=2)
		rsnrVal = Label(snrrFrame, textvariable=self.sonarR).grid(column=3, row=2)

		# Wheel Encoders
		ttk.Label(encFrame, text="Wheel Encoders").grid(column=0, row=0, columnspan=2)
		ttk.Label(LencFrame, text="LEFT").grid(column=0, row=0)
		ttk.Label(LencFrame, text="Forward").grid(column=0, row=1)
		ttk.Label(LencFrame, text="Backward").grid(column=0, row=3)
		ttk.Label(RencFrame, text="RIGHT").grid(column=0, row=0)
		ttk.Label(RencFrame, text="Forward").grid(column=0, row=1)
		ttk.Label(RencFrame, text="Backward").grid(column=0, row=3)
		ttk.Label(LFencFrame, textvariable=self.encLF).grid(column=0, row=2)
		ttk.Label(LBencFrame, textvariable=self.encLB).grid(column=0, row=4)
		ttk.Label(RFencFrame, textvariable=self.encRF).grid(column=0, row=2)
		ttk.Label(RBencFrame, textvariable=self.encRB).grid(column=0, row=4)

		# Back Bumper
		ttk.Label(bBumper, text="Back Bumper").grid(column=0, columnspan=3, row=0)
		ttk.Label(bBumper, text="Left").grid(column=0, row=1)
		ttk.Label(bBumper, text="Middle").grid(column=1, row=1)
		ttk.Label(bBumper, text="Right").grid(column=2, row=1)
		ttk.Label(bBumper, textvariable=self.bumpLB).grid(column=0, row=2)
		ttk.Label(bBumper, textvariable=self.bumpMB).grid(column=1, row=2)
		ttk.Label(bBumper, textvariable=self.bumpRB).grid(column=2, row=2)


	def bumperLFCB(self, msg):
	# rospy.loginfo("Left Front Bumper")
		if msg.data is True:
			self.bumpLF.set ('X')
		else:
			self.bumpLF.set ('o')

	def bumperMFCB(self, msg):
	# rospy.loginfo("Middle Front Bumper")
		if msg.data is True:
			self.bumpMF.set ('X')
		else:
			self.bumpMF.set ('o')

	def bumperRFCB(self, msg):
	# rospy.loginfo("Right Front Bumper")
		if msg.data is True:
			self.bumpRF.set ('X')
		else:
			self.bumpRF.set ('o')

	def bumperLBCB(self, msg):
	# rospy.loginfo("Left Back Bumper")
		if msg.data is True:
			self.bumpLB.set ('X')
		else:
			self.bumpLB.set ('o')

	def bumperMBCB(self, msg):
	#rospy.loginfo("Middle Back Bumper")
		if msg.data is True:
				self.bumpMB.set ('X')
		else:
				self.bumpMB.set ('o')

	def bumperRBCB(self, msg):
	# rospy.loginfo("Right Back Bumper")
		if msg.data is True:
			self.bumpRB.set ('X')
		else:
			self.bumpRB.set ('o')

	def sonarLCB(self, msg):
		self.sonarL.set (int(msg.range))

	def sonarLMCB(self, msg):
		self.sonarLM.set (int(msg.range))

	def sonarRMCB(self, msg):
		self.sonarRM.set (int(msg.range))

	def sonarRCB(self, msg):
		self.sonarR.set (int(msg.range))

	def encoderLFCB(self, msg):
		if msg.data > 0:
			self.encLF.set (msg.data)

	def encoderLBCB(self, msg):
		if msg.data > 0:
			self.encLB.set (msg.data)

	def encoderRFCB(self, msg):
		if msg.data > 0:
			self.encRF.set (msg.data)

	def encoderRBCB(self, msg):
		if msg.data > 0:
			self.encRB.set (msg.data)


def main():
	rospy.init_node("DebugGUI")
	rospy.loginfo("ClubBot2 Diagnostic GUI Started")
	gui=Tk()
	debugGUI(gui)
	gui.mainloop()
	rospy.spin

if __name__ == '__main__':
	main()
