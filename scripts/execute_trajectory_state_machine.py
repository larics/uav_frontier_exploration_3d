#!/usr/bin/env python

__author__ = 'abatinovic'

import rospy, math, time
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64, Empty, Int32, Float32, String, Bool
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, \
  MultiDOFJointTrajectory, JointTrajectory, JointTrajectoryPoint

from larics_motion_planning.srv import MultiDofTrajectory, \
  MultiDofTrajectoryRequest, MultiDofTrajectoryResponse
import copy

class UavExplorationSm:

  def __init__(self):
    self.state = "start"
    self.state_previous = "none"
    self.target_pose = PoseStamped()
    self.current_pose = Pose()
    self.current_reference = MultiDOFJointTrajectoryPoint()
    self.first_measurement_received = False
    self.executing_trajectory = 0
    self.start_time = time.time()
    self.execution_start = time.time()
    self.confirmed = False
    self.service_called = False

    # Initialize ROS params
    # Distance of the UAV from the target pose at which we consider the 
    # trajectory to be executed
    self.r_trajectory = rospy.get_param('radius_trajectory_executed', 0.5)
    # Rate of the state machine.
    self.rate = rospy.get_param('rate', 10)
    # How long do we collect feedback and store it in array. Checks for execution
    #relyes on this parameter because they check through last x 
    # seconds to determine next state.
    self.feedback_collection_time = rospy.get_param('feedback_collection_time', 1.0)
    self.dt = 1.0/float(self.rate)
    self.n_feedback_points = int(self.rate*self.feedback_collection_time)
    # Set up array of feedback poses
    self.feedback_array = []
    self.feedback_array_index = 0
    for i in range(self.n_feedback_points):
      temp_pose = Pose()
      self.feedback_array.append(copy.deepcopy(temp_pose))

    # Initialize publishers
    self.state_pub = rospy.Publisher('exploration/current_state', String, 
      queue_size=1, latch=True)
    self.trajectory_pub = rospy.Publisher('joint_trajectory', JointTrajectory, 
      queue_size=1)
    self.point_reached_pub = rospy.Publisher('exploration/point_reached', Bool, 
      queue_size=1)

    # Initialize services
    print "Waiting for service multi_dof_trajectory."
    rospy.wait_for_service('multi_dof_trajectory', timeout=30)
    self.plan_trajectory_service = rospy.ServiceProxy(
      "multi_dof_trajectory", MultiDofTrajectory)
    
    rospy.Service('confirm_trajectory', SetBool, self.confirm_trajectory)
    
    # Initialize subscribers
    rospy.Subscriber('exploration/goal', PoseStamped, 
      self.targetPointCallback, queue_size=1)
    rospy.Subscriber('carrot/trajectory', MultiDOFJointTrajectoryPoint,
      self.referenceCallback, queue_size=1)
    rospy.Subscriber('odometry', Odometry, 
      self.globalPositionCallback, queue_size=1)
    rospy.Subscriber('executing_trajectory', Int32, 
      self.executingTrajectoryCallback, queue_size=1)

    time.sleep(0.2)

  def run(self):
    
    rate = rospy.Rate(self.rate)
    self.state_pub.publish(self.state)

    while not rospy.is_shutdown() and not self.first_measurement_received:
      print ("Waiting for the first pose...")
      time.sleep(1)
    print ("The first pose received. Starting exploration state machine.")

    while not rospy.is_shutdown():

      # Start state only waits for something to happen
      if self.state == "start":
        if self.state_previous != self.state:
          self.printStates()
          self.state_previous = "start"
          self.state_pub.publish(self.state)

      # Planning the obstacle free trajectory in the map
      if self.state == "plan":
        if self.state_previous != self.state:
          self.printStates()
          self.state_previous = "plan"
          self.state_pub.publish(self.state)
        else: 
          print("Planning again!")

        print ("Calling service!")
        # Call the obstacle free trajectory planning service
        request = MultiDofTrajectoryRequest()
        # Create start point from current position information
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [self.current_reference.transforms[0].translation.x, \
          self.current_reference.transforms[0].translation.y, \
          self.current_reference.transforms[0].translation.z, \
          self.quaternion2Yaw(self.current_reference.transforms[0].rotation)]
        request.waypoints.points.append(copy.deepcopy(trajectory_point))
        # Create start point from target position information
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [self.target_pose.position.x, \
          self.target_pose.position.y, self.target_pose.position.z, \
          self.quaternion2Yaw(self.target_pose.orientation)]
        request.waypoints.points.append(copy.deepcopy(trajectory_point))
        # Set up flags
        request.publish_path = False
        request.publish_trajectory = False
        request.plan_path = True
        request.plan_trajectory = True

        response = self.plan_trajectory_service.call(request)
        while not self.service_called and not rospy.is_shutdown():
          rospy.sleep(0.01)

        self.service_called = False
        # if trajectory is OK publish it
        if self.confirmed:
          # If we did not manage to obtain a successful plan then go to
          # appropriate state.
          if response.success == False:
            print ("**********************************************")
            print ("In state:", self.state)
            print ("Path planning failed!")
            print ("**********************************************")
            print (" ")
            self.state = ("end")
          # If plan was successful then execute it.
          else:
            self.trajectory_pub.publish(response.trajectory)
            self.state = "execute"
        # if trajectory is not OK, point is reached and go to "start" state
        else:
          self.state = "end"



      # While trajectory is executing we check if it is done 
      if self.state == "execute":
        if self.state_previous != self.state:
          self.printStates()
          self.state_previous = "execute"
          self.state_pub.publish(self.state)
          self.execution_start = time.time()

        while not rospy.is_shutdown():      
          # When trajectory is executed simply go to end state.
          if self.checkTrajectoryExecuted() == True:
            print ("**********************************************")
            print ("In state:", self.state)
            print ("Execution timeout factor triggered!")
            print ("**********************************************")
            print (" ")
            self.state = "end"
            break
          # If we want to send another point anytime
          if self.state == "plan":
            break
          rate.sleep()


      # End state, publish that you reached the point
      if self.state == "end":
        if self.state_previous != self.state:
          self.printStates()
          self.state_previous = "end"
          self.state_pub.publish(self.state)
          self.point_reached_pub.publish(True)

        time.sleep(0.05)
        # TODO: publish 
        self.state = "start"

      rate.sleep()

  def printStates(self):
    print ("----------------------------------------------------")
    print ("State changed. Previous state:", self.state_previous)
    print ("State changed. Current state:", self.state)
    print ("----------------------------------------------------")
    print (" ")

  def targetPointCallback(self, msg):
    self.target_pose = msg.pose
    self.state = "plan"
    print ("New goal accepted.")

  def globalPositionCallback(self, msg):
    self.current_pose = msg.pose.pose
    self.first_measurement_received = True

    # Collect array of data with rate of the loop. We can fill this list 
    # in a cyclic manner since we have info about first and last data point
    # stored in feedback_array_index
    if ((time.time()-self.start_time) > self.dt):
      self.start_time = time.time()
      self.feedback_array[self.feedback_array_index] = copy.deepcopy(self.current_pose)
      self.feedback_array_index = self.feedback_array_index + 1
      if self.feedback_array_index >= self.n_feedback_points:
        self.feedback_array_index = 0
  
  def referenceCallback(self, msg):
    self.current_reference = msg

  def executingTrajectoryCallback(self, msg):
    self.executing_trajectory = msg.data

  def quaternion2Yaw(self, quaternion):
    q0 = quaternion.w
    q1 = quaternion.x
    q2 = quaternion.y
    q3 = quaternion.z
    return math.atan2(2.0*(q0*q3 + q1*q2), 1.0-2.0*(q2*q2 + q3*q3))

  def checkTrajectoryExecuted(self):
    # Here we check if the UAV's has been within some radius from the target 
    # point for some time. If so we consider trajectory to be executed.
    for i in range(self.n_feedback_points):
      dx = self.target_pose.position.x - self.feedback_array[i].position.x
      dy = self.target_pose.position.y - self.feedback_array[i].position.y
      dz = self.target_pose.position.z - self.feedback_array[i].position.z

      delta = math.sqrt(dx*dx + dy*dy + dz*dz)

      if delta > self.r_trajectory:
        return False

    if self.executing_trajectory == 0:
      return True
    else:
      return False

  def confirm_trajectory(self, req):
    self.service_called = True
    self.confirmed = req.data
    if (req.data):
      print("Trajectory OK!")
    else:
      print("Trajectory is NOT OK!") 
    return SetBoolResponse(True, "Service confirm_trajectory called")

if __name__ == '__main__':

  rospy.init_node('execute_trajectory_state_machine')
  exploration = UavExplorationSm()
  exploration.run()
