#!/usr/bin/env python

import rospy
import tams_pr2_look.srv
import tams_pr2_look.msg

from geometry_msgs.msg import PointStamped, Point, PoseArray, Vector3
from std_msgs.msg import Header

from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from actionlib import SimpleActionClient

import math
import tf

import pyquaternion

DEFAULT_LOOK_FRAME= "high_def_optical_frame"

DEFAULT_STABLE_FRAME= "base_footprint"

tfl = None

class LookAction:
	'''
	abstract interface for targetting different backends

	stable_frame         -> frame considered stable over time when following a target
	goal()               -> abstract interface method that has to return a PointHeadGoal in every cycle
	setTarget / target() -> general pointStamped relevant for some backends
	'''
	def __init__(self, stable_frame= ""):
		if stable_frame == "":
			stable_frame = DEFAULT_STABLE_FRAME
		self.stable_frame= stable_frame
		self._target= PointStamped()

		self.pub_target = rospy.Publisher("~target", PointStamped, queue_size= 1)

	def setTarget(self, target):
		# if a time is specified, transform to stable frame *once*, otherwise reinterpret on every cycle
		if target.header.stamp > rospy.Time():
			try:
				tfl.waitForTransform(self.stable_frame, target.header.frame_id, target.header.stamp, rospy.Duration(0.1))
				target_trans = tfl.transformPoint(self.stable_frame, target)
				if abs(math.atan2(target_trans.point.x, target_trans.point.y)+math.tau/4) > math.tau/6:
					self._target= target_trans
					self._target.header.stamp= rospy.Time()
				else:
					rospy.logwarn_throttle(30.0, "rejected invalid look at target behind robot")
			except tf.Exception:
				# do not update the target if we can't transform to a stable frame
				pass
		else:
			self._target= target

	def target(self):
		if self._target.header.frame_id == "":
			return self._target
		else:
			t = tfl.transformPoint(self.stable_frame, self._target)
			self.pub_target.publish(t)
			return t

	def goal():
		raise Exception("not implemented")

class LookDirection(LookAction):
	'''
	collection of modes to look in specific directions given by keywords
	'''

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
	'''
	general mode, targetting a specific point in space

	The point might move relative to stable_frame over time
	and might be specified in a moving frame too
	'''
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
	'''
	Abstract base class for modes that need to initialize

	Provide some basic logging while waiting
	'''
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
	'''
	mode looking at the most salient region according to vocus
	'''
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
	'''
	Look at human faces detected by gazr
	'''
	def __init__(self, stable_frame= ""):
		LookInitialize.__init__(self, stable_frame)
		self.sub= rospy.Subscriber("gazr/detected_faces/poses", PoseArray, self.cb)
		self.waitForInitialize()

	def cb(self, poses):
		if len(poses.poses) == 0:
			return

		poses.poses.sort(key=lambda pose: pose.position.y)
		# TODO: which face is the best to look at?
		self.setTarget( PointStamped(
			header= poses.header,
			point= poses.poses[int(len(poses.poses)/2)].position
			) )
		self.initialized= True

	def goal(self):
		goal= PointHeadGoal(
			target= self.target(),
			pointing_axis= Vector3(y= -0.1, z= 1.0), # slightly offset by y to account for human interpretation
			pointing_frame= DEFAULT_LOOK_FRAME
			)
		goal.target.header.stamp = rospy.Time() # avoid reported errors due to outdated targets
		return goal

class LookSoundSourceLocalization(LookInitialize):
	'''
	Look at primary sound source estimated by ssloc
	'''
	def __init__(self, stable_frame= ""):
		LookInitialize.__init__(self, stable_frame)
		self.sub= rospy.Subscriber("ssloc/unit_sphere_sst_poses", PoseArray, self.cb)
		self.waitForInitialize()

	def cb(self, poses):
		self.initialized= True
		if len(poses.poses) > 0:
			o= poses.poses[0].orientation
			pt = pyquaternion.Quaternion(o.w, o.x, o.y, o.z).rotate([2.0,0,0])
			pts = PointStamped(
				header= poses.header,
				point= Point(*pt)
				)
			try:
				level_frame = "base_footprint"
				tfl.waitForTransform(level_frame, pts.header.frame_id, pts.header.stamp, rospy.Duration(0.1))
				pts = tfl.transformPoint(level_frame, pts)
				pts.point.z = 1.8
				self.setTarget( pts )
			except tf.Exception:
				pass

	def goal(self):
		goal= PointHeadGoal(
			target= self.target(),
			pointing_axis= Vector3(y= -0.2, z= 1.0),
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

		# Initalize action attribute to allow testing for subscribers at first initialization
		self.action = None

		self.default_max_velocity= rospy.get_param("~velocity", 0.2)
		# TODO: allow setting point / stable_frame from parameters
		self.set_look_target(tams_pr2_look.srv.SetTargetRequest(mode= rospy.get_param("~mode")))

		self.point_head = SimpleActionClient('head_traj_controller/point_head_action', PointHeadAction)
		if not self.point_head.wait_for_server(rospy.Duration(20) ):
			rospy.logwarn("still waiting for head_traj_controller/point_head_action")
			self.point_head.wait_for_server()

		self.srv= rospy.Service(rospy.get_name()+'/target', tams_pr2_look.srv.SetTarget, self.set_look_target)
		# TODO: self.pub for publishing the current state (latch)

		self.state_pub = rospy.Publisher(rospy.get_name()+"/state", tams_pr2_look.msg.State, queue_size= 1)

	def run(self):
		'''
		run controlling loop
		'''

		rospy.loginfo("look initialized")

		r= rospy.Rate(20)
		while not rospy.is_shutdown():
			# current active mode provides LookAt action goal
			goal = self.action.goal()

			# use fixed velocity if the active mode does not specify it
			if goal.max_velocity == 0.0:
				goal.max_velocity = self.default_max_velocity

			# skip invalid goals
			if goal.target.header.frame_id != "":
				self.point_head.send_goal(goal)

			# publish current internal state
			self.state_pub.publish(tams_pr2_look.msg.State(mode= self.request.mode, target= self.request.target, stable_frame= self.request.stable_frame))

			r.sleep()

	def set_look_target(self, req):
		'''
		ROS service callback to change mode dynamically
		'''
		# Unregister subscriber from previous action to prevent "rejected invalid look at target behind robot" from its callbacks
		if hasattr(self.action, 'sub'):
			self.action.sub.unregister()

		if req.mode in LookDirection.directions:
			try:
				self.action= LookDirection(req.mode)
				self.request = tams_pr2_look.srv.SetTargetRequest(mode= req.mode)
			except Exception as e:
				rospy.logerr(str(e))
		elif req.mode == "point":
			self.action= LookPoint(req.target, req.stable_frame)
			self.request = req
		elif req.mode == "vocus":
			self.action= LookVocus()
			self.request = tams_pr2_look.srv.SetTargetRequest(mode= req.mode)
		elif req.mode == "gazr":
			self.action= LookGazr(req.stable_frame)
			self.request = tams_pr2_look.srv.SetTargetRequest(mode= req.mode)
		elif req.mode == "sound_source_localization":
			self.action= LookSoundSourceLocalization(req.stable_frame)
			self.request = tams_pr2_look.srv.SetTargetRequest(mode= req.mode)
		else:
			rospy.logerr("unknown Look mode '"+str(req.mode)+"'")
		return tams_pr2_look.srv.SetTargetResponse()

def run():
	Look().run()

if __name__  == '__main__':
	run()
