#!/usr/bin/env python

import rospy
import trixi_look.srv

from geometry_msgs.msg import PointStamped, Point, PoseArray, Vector3
from std_msgs.msg import Header

from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from actionlib import SimpleActionClient

import tf

DEFAULT_LOOK_FRAME= "high_def_optical_frame"

DEFAULT_STABLE_FRAME= "base_footprint"

tfl = None

# abstract interface for targetting different backends
class LookAction:
	def __init__(self, stable_frame= ""):
		if stable_frame == "":
			stable_frame = DEFAULT_STABLE_FRAME
		self.stable_frame= stable_frame
		self._target= PointStamped()

	def setTarget(self, target):
		# if a time is specified, transform to stable frame *once*, otherwise reinterpret on every cycle
		if target.header.stamp > rospy.Time():
			try:
				tfl.waitForTransform(self.stable_frame, target.header.frame_id, target.header.stamp, rospy.Duration(0.1))
				self._target= tfl.transformPoint(self.stable_frame, target)
				self._target.header.stamp= rospy.Time()
			except tf.Exception:
				# do not update the target if we can't transform to a stable frame
				pass
		else:
			self._target= target

	def target(self):
		if self._target.header.frame_id == "":
			return self._target
		else:
			return tfl.transformPoint(self.stable_frame, self._target)

	def goal():
		raise Exception("not implemented")

class LookDirection(LookAction):
	directions = {
		"straight" : Point(x= 1.3, y= 0.0, z= 1.6),
		"left" : Point(x= 1.3, y= 1.0, z= 1.6),
		"right" : Point(x= 1.3, y= -1.0, z= 1.6),
		"down" : Point(x= 1.3, y= 0.0, z= 0.5),
		"table" : Point(x= 1.1, y= 0.0, z= 0.4),
		}

	def __init__(self, direction):
		LookAction.__init__(self)
		try:
			self.setTarget(PointStamped(
				header= Header(frame_id= "base_footprint"),
				point= LookDirection.directions[direction]
				))
		except KeyError:
			raise Exception("unknown keyword '"+str(direction)+"'")

	def goal(self):
		return PointHeadGoal(
			target= self.target(),
			pointing_axis= Vector3(z= 1.0),
			pointing_frame= DEFAULT_LOOK_FRAME
			)

class LookPoint(LookAction):
	def __init__(self, target, stable_frame= ""):
		LookAction.__init__(self, stable_frame)
		self.target= target

	def goal(self):
		return PointHeadGoal(
			target= self.target,
			pointing_axis= Vector3(z= 1.0),
			pointing_frame= DEFAULT_LOOK_FRAME
			)

class LookInitialize(LookAction):
	def __init__(self, stable_frame= ""):
		LookAction.__init__(self, stable_frame)
		self.initialized= False

	def waitForInitialize(self):
		r= rospy.Rate(10)
		wait_start= rospy.Time.now()
		while not rospy.is_shutdown() and not self.initialized:
			if rospy.Time.now() - wait_start > rospy.Duration(5.0):
				rospy.logwarn_throttle(10, "Still initializing requested LookAction. Is the corresponding backend running?")
			r.sleep()

class LookVocus(LookInitialize):
	def __init__(self):
		LookInitialize.__init__(self)
		self.sub= rospy.Subscriber("saliency_poi", PointStamped, self.cb)
		self.waitForInitialize()

	def cb(self, point):
		self.target= point
		self.initialized= True

	def goal(self):
		goal = PointHeadGoal(
			target= self.target,
			pointing_axis= Vector3(z= 1.0),
			# important as the observed behavior might not converge with any other frame
			pointing_frame= self.target.header.frame_id
			)
		goal.target.header.stamp = rospy.Time() # avoid reported errors due to outdated targets
		return goal

class LookGazr(LookInitialize):
	def __init__(self, stable_frame= ""):
		LookInitialize.__init__(self, stable_frame)
		self.sub= rospy.Subscriber("gazr/detected_faces/poses", PoseArray, self.cb)
		self.waitForInitialize()

	def cb(self, poses):
		self.initialized= True
		if len(poses.poses) > 0:
			# TODO: select the best one
			self.setTarget( PointStamped(
				header= poses.header,
				point= poses.poses[0].position
				) )

	def goal(self):
		goal= PointHeadGoal(
			target= self.target(),
			pointing_axis= Vector3(z= 1.0),
			pointing_frame= DEFAULT_LOOK_FRAME
			)
		goal.target.header.stamp = rospy.Time() # avoid reported errors due to outdated targets
		return goal


class Look:
	def __init__(self):
		rospy.init_node("look")

		# ugly, but we need this as singleton
		global tfl
		tfl= tf.TransformListener()

		self.default_max_velocity= rospy.get_param("~velocity", 0.2)
		self.set_look_target(trixi_look.srv.SetTargetRequest(mode= rospy.get_param("~mode")))

		self.point_head = SimpleActionClient('head_traj_controller/point_head_action', PointHeadAction)
		if not self.point_head.wait_for_server(rospy.Duration(20) ):
			rospy.logwarn("still waiting for head_traj_controller/point_head_action")
			self.point_head_action.wait_for_server()

		self.srv= rospy.Service(rospy.get_name()+'/target', trixi_look.srv.SetTarget, self.set_look_target)
		# TODO: self.pub for publishing the current state (latch)

	def run(self):
		rospy.loginfo("look initialized")
		r= rospy.Rate(20)
		while not rospy.is_shutdown():
			goal = self.action.goal()
			if goal.max_velocity == 0.0:
				goal.max_velocity = self.default_max_velocity
			# skip invalid goals
			if goal.target.header.frame_id != "":
				self.point_head.send_goal(goal)
			r.sleep()

	def set_look_target(self, req):
		if req.mode in LookDirection.directions:
			try:
				self.action= LookDirection(req.mode)
			except Exception as e:
				rospy.logerr(str(e))
		elif req.mode == "point":
			self.action= LookPoint(req.target, req.stable_frame)
		elif req.mode == "vocus":
			self.action= LookVocus()
		elif req.mode == "gazr":
			self.action= LookGazr(req.stable_frame)
		else:
			rospy.logerr("unknown Look mode '"+str(req.mode)+"'")
		return trixi_look.srv.SetTargetResponse()

def run():
	Look().run()

if __name__  == '__main__':
	run()
