

import rospy
from geometry_msgs.msg import PoseArray
import numpy as np


class Strategy:

	def __init__(self):

		self.pose_sub = rospy.Subscriber('/all_poses', PoseArray, self.pose_callback)
		self.ball_sub = rospy.Subscriber('/ball_loc', Pose, self.ball_callback)
		# Uncomment this when figure out how to get ref calls
		# self.ref_sub = rospy.Subscriber('/ref_calls', RefCall, self.ref_callback)

		# publishers to a controller for each robot 
		# could change to one with an ID for robot but only 6 robots so this might work fine
		
		self.robo_pub = [rospy.Publisher('/robot1', Task, queue_size=10),rospy.Publisher('/robot2', Task, queue_size=10),
						 rospy.Publisher('/robot3', Task, queue_size=10), rospy.Publisher('/robot4', Task, queue_size=10),
						 rospy.Publisher('/robot5', Task, queue_size=10), rospy.Publisher('/robot6', Task, queue_size=10)]

		self.posession_distance = 0.2 #?
		self.shooting_dist = 1
		self.interference_distance = 2

		self.state = 'DEFENSE'
		self.posession = None
		self.locs = None

	def check_clear_path(self, p1, p2):
		# check if there are any opposing teams along line
		for i in range(6, 12):
			other = self.locs[i]
			p1 = np.array([p1.x, p1.y])
			p2 = np.array([p2.x, p2.y])
			p3 = np.array([other.position.x, other.position.y])
			dist_from_line = np.cross(p2 - p1, p3 - p1) / np.linalg.norm(p2 -p1)
			if dist_from_line <= self.interference_distance:
				return False
		return True

	def can_score(self, goal_loc):
		# determine if robot with posession has a good chance of scoring
		# FUTURE: can train something to generate shot success probability
		# right now if within certain distance, shoot
		shooter_loc = self.locs[self.posession]
		dist_to_goal = (shooter_loc.position.x - goal_loc[0])**2 + (shooter_loc.position.y - goal.loc[1])**2
		if dist_to_goal < self.shooting_dist and self.check_clear_path(shooter_loc, goal_loc):
			return True
		return False

	def can_pass(self):
		# determine if robot with posession can pass
		# FUTURE: determine best location to pass to, not best current robot location to recieve
		for i in range(6):
			if i != self.posession:
				if self.check_clear_path(self.locs[self.posession], self.locs[i]):
					msg = Task(type='PASS', location=self.locs[i])
					msg2 = Task(type='RECIEVE', location=self.locs[i])
					self.robo_pub[self.posession].publish(msg)
					self.robo_pub[i].publish(msg2)
					return i
		return None

	def guard(self, available):
		# find nearest enemy
		for i in available:
			distances = [(i.position.x - self.locs[j].position.x)**2 + (i.position.y - self.locs[j])**2 
							for j in range(6, 12)]
			to_guard = np.argmin(distances)
			msg = Task(type='GUARD', location=self.locs[to_guard])
			self.robo_pub[i].publish(msg)

	def intercept(self):
		# determine best place to intercept 
		pass

	def pose_callback(self, data):
		self.locs = data.poses
		if state == 'OFFENSE':
			if self.can_score():
				msg = Task(type='SHOOT', location=goal_loc)
				self.robo_pub[self.posession].publish(msg)
			else:
				reciever = self.can_pass()
				available = copy(self.locs[:6])
				if receiver is not None:
					available.pop(reciever)
				self.guard(available)

			# Code for rest of robots to get open / guard
		elif state == 'DEFENSE':
			# send closest robot to intercept location
			intercept = self.intercept()
			# send closest robot to enemy with ball
			distances =  [(self.locs[self.posession].position.x - self.locs[j].position.x)**2 + 
								(self.locs[self.posession].position.y - self.locs[j])**2 for j in range(0, 5)]
			steal = np.argmin(distances)
			msg = Task(type='GUARD', location=self.locs[self.possession])
			self.robo_pub[steal].publish(msg)
			# send farthest back robots to guard goal
			# send rest of robots to guard players
			available = copy(self.locs[:6])
			avaialble.pop(intercept)
			available.pop(steal)
			self.guard(available)

	def ball_callback(self, data):
		# determin which if any robot has posession of ball
		bx, by = data.position.x, data.position.y
		posession = False
		for i in range(12):
			loc = self.locs[i]
			dist = (bx - loc.position.x) **2 + (by - loc.position.y)**2
			if dist < self.posession_distance:
				posession = True
				self.posession = i
				# assumes first 6 indices are our robots and last 6 are other team
				if i <=5:
					self.state = 'OFFENSE'
				else:
					self.state == 'DEFENSE'
		if not posession:
			self.posession = None