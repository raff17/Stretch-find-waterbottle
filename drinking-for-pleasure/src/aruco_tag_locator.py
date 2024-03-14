#!/usr/bin/env python3

# Import modules
import rospy
import time
import tf2_ros
import tf
import numpy as np
import stretch_body.robot
import math
from math import pi

from socket import *
import struct
import sys
import Session as session # Our own file for handling error messages
#import 2D_navigation_code as navigation

#from movement import move_to_goal_point

# Import hello_misc script for handling trajectory goals with an action client
import hello_helpers.hello_misc as hm

# We're going to subscribe to a JointState message type, so we need to import
# the definition for it
from sensor_msgs.msg import JointState

# Import the FollowJointTrajectoryGoal from the control_msgs.msg package to
# control the Stretch robot
from control_msgs.msg import FollowJointTrajectoryGoal

# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories
from trajectory_msgs.msg import JointTrajectoryPoint

# Import TransformStamped from the geometry_msgs package for the publisher
from geometry_msgs.msg import TransformStamped, Twist, Pose, Point, Quaternion

from tf.transformations import quaternion_from_euler, euler_from_quaternion 
#from tf.TransformerRos import fromTranslationRotation
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import atan2



HOST = '127.0.0.1'
CLIENTPORT = 8000
SERVERPORT = 8001



class LocateArUcoTag(hm.HelloNode):
    """
    A class that actuates the RealSense camera to find the docking station's
    ArUco tag and returns a Transform between the `base_link` and the requested tag.
    """
    def __init__(self):
        """
        A function that initializes the subscriber and other needed variables.
        :param self: The self reference.
        """
        # Initialize the inhereted hm.Hellonode class
        hm.HelloNode.__init__(self)
        # rospy.init_node('move')

        # Initialize subscriber
        self.joint_states_sub = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

        # Initialize publisher
        self.transform_pub = rospy.Publisher('/ArUco_transform', TransformStamped, queue_size=10)
        self.pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1) #/stretch_diff_drive_controller/cmd_vel for gazebo

        # Initialize the variable that will store the joint state positions
        self.joint_state = None

        # Provide the min and max joint positions for the head pan. These values
        # are needed for sweeping the head to search for the ArUco tag
        self.min_pan_position = -3.90
        self.max_pan_position =  1.70

        # Define the number of steps for the sweep, then create the step size for
        # the head pan joint
        self.pan_num_steps = 10
        self.pan_step_size = abs(self.min_pan_position - self.max_pan_position)/self.pan_num_steps

        # Define the min tilt position, number of steps, and step size
        self.min_tilt_position = -0.7
        self.tilt_num_steps = 3
        self.tilt_step_size = pi/16

        # Define the head actuation rotational velocity
        self.rot_vel = 0.25 # radians per sec
        
        
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.theta = 0.0

        self.delta = 0.2
        self.delt_vert = 0.02
        
        self.rot_speed = 0.25
        self.forward_speed = 0.6
        self.move = Twist()
        
        #self.nav = navigation.StretchNavigation()
        # self.r = rospy.Rate(10)
        # rospy.init_node("speed_controller")

        # create publisher and subscriber
        ###############################
        # TO CHANGE:
        #   - Confirm reading points from the /odom topic,
        # or discover what topic is getting odometry data
        ####################################################
        self.sub = rospy.Subscriber("/odom", Odometry,
                                    self.newOdom)  # our odometry node is called /odom and not /odometry/filtered
    

    def newOdom(self, msg):
        # print(f"msg = {msg}")
        # get the current x and y position values for the robot
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        # rospy.loginfo("Current x-coordinate: " + str(self.cur_x))
        # rospy.loginfo("Current y-coordinate: " + str(self.cur_y))
        # rospy.loginfo("Current theta value: " + str(self.theta))
        #return self.cur_x



    def move_to_goal_point(self, curGoal):
        # print(self)
        reached = False
        x = curGoal.x
        y = curGoal.y
        z = curGoal.z
        reached = False
        rospy.loginfo("Inside move_to_goal_point()")
        # While the program is running or the end goal has not been the loop will continue indefinitely
        while not rospy.is_shutdown() and not reached:
            # self.cur_x = msg.pose.pose.position.x
            # self.cur_y = msg.pose.pose.position.y
            inc_x = x - self.cur_x
            inc_y = y - self.cur_y
            inc_z  = z
            rospy.loginfo("Incrementation of x: " + str(inc_x))
            rospy.loginfo("Incrementation of y: " + str(inc_y))
            rospy.loginfo("Incrementation of z: " + str(inc_z))
            rospy.loginfo("Current goal: " + str(curGoal))

            angle_to_goal = atan2(inc_y, inc_x)
            dist = math.sqrt(((x - self.cur_x) ** 2) + ((y - self.cur_y) ** 2))
            # rospy.loginfo("Current distance to goal: " + str(dist))

            # IS NOT UPDATING TO THE NEW ANGLE SEEN, SO IT IS STUCK AT 0.78
            if dist <= self.delta:  # and abs(self.theta) <= self.delta * 0.5:
                rospy.loginfo("Theta: " + str(self.theta))
                rospy.loginfo("Robot is close enough to the participants. Stopping now!")
                self.move.linear.x = 0.0
                self.move.angular.z = 0.0
                reached = True


            elif abs(angle_to_goal - self.theta) > 0.3:  # self.delta:
                if y > 0:
                    self.move.linear.x = 0.0
                    self.move.angular.z = self.rot_speed  # 0.25
                    # do something
                else:
                    self.move.linear.x = 0.0
                    self.move.angular.z = -1 * self.rot_speed  # -0.25

            else:
                self.move.linear.x = self.forward_speed  # 0.5
                self.move.angular.z = 0.0

            self.pub.publish(self.move)
            #break
        return dist
            # rospy.Rate(4).sleep()
    
            
    def joint_states_callback(self, msg):
        """
        A callback function that stores Stretch's joint states.
        :param self: The self reference.
        :param msg: The JointState message type.
        """
        self.joint_state = msg

    def send_command(self, command):
        '''
        Handles single joint control commands by constructing a FollowJointTrajectoryGoal
        message and sending it to the trajectory_client created in hello_misc.
        :param self: The self reference.
        :param command: A dictionary message type.
        '''
        if (self.joint_state is not None) and (command is not None):

            # Extract the string value from the `joint` key
            joint_name = command['joint']

            # Set trajectory_goal as a FollowJointTrajectoryGoal and define
            # the joint name
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.trajectory.joint_names = [joint_name]

            # Create a JointTrajectoryPoint message type
            point = JointTrajectoryPoint()

            # Check to see if `delta` is a key in the command dictionary
            if 'delta' in command:
                # Get the current position of the joint and add the delta as a
                # new position value
                joint_index = self.joint_state.name.index(joint_name)
                joint_value = self.joint_state.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta
                point.positions = [new_value]

            # Check to see if `position` is a key in the command dictionary
            elif 'position' in command:
                # extract the head position value from the `position` key
                point.positions = [command['position']]

            # Set the rotational velocity
            point.velocities = [self.rot_vel]

            # Assign goal position with updated point variable
            trajectory_goal.trajectory.points = [point]

            # Specify the coordinate frame that we want (base_link) and set the time to be now.
            trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
            trajectory_goal.trajectory.header.frame_id = 'base_link'

            # Make the action call and send the goal. The last line of code waits
            # for the result
            self.trajectory_client.send_goal(trajectory_goal)
            self.trajectory_client.wait_for_result()
    
    def move_forward(self, bottlePose):
        position = [bottlePose.transform.translation.x,
                    bottlePose.transform.translation.y,
                    bottlePose.transform.translation.z]
                    
        rotation = [bottlePose.transform.rotation.x,
                    bottlePose.transform.rotation.y,
                    bottlePose.transform.rotation.z,
                    bottlePose.transform.rotation.w]
        
        transformMatrix = tf.TransformListener.fromTranslationRotation(self, translation=position, rotation=rotation)
        point = [0, 0, 0, 1]
        
        bottlePoint = transformMatrix @ np.transpose(point)
        #rospy.loginfo(bottlePoint)
        newPoint = Point()
        newPoint.x = bottlePoint[0]
        newPoint.y = bottlePoint[1]
        newPoint.z = bottlePoint[2]

        self.move_to_goal_point(newPoint)
        self.arm_position(newPoint)
        
        #self.nav.go_to(position[0], position[1], rotation[2]) # position[2]
        
        #command = Twist()

	# A Twist has three linear velocities (in meters per second), along each of the axes.
	# For Stretch, it will only pay attention to the x velocity, since it can't
	# directly move in the y direction or the z direction
        #command.linear.x = 0.1
        #command.linear.y = 0.0
        #command.linear.z = 0.0

	# A Twist also has three rotational velocities (in radians per second).
	# The Stretch will only respond to rotations around the z (vertical) axis
        #command.angular.x = 0.0
        #command.angular.y = 0.0
        #command.angular.z = 0.0
        #rospy.loginfo("Trying to move")
        
        # Publish the Twist commands
        #self.pub.publish(command)
        
    def move_forward_one_second(self):
        command = Twist()

	# A Twist has three linear velocities (in meters per second), along each of the axes.
	# For Stretch, it will only pay attention to the x velocity, since it can't
	# directly move in the y direction or the z direction
        #start = time.get_time()
        #while time.Time() - start < 1:
        command.linear.x = 0.1
        command.linear.y = 0.0
        command.linear.z = 0.0
	
	# A Twist also has three rotational velocities (in radians per second).
	# The Stretch will only respond to rotations around the z (vertical) axis
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
        #rospy.loginfo("Trying to move")
        
        # Publish the Twist commands
        self.pub.publish(command)
        time.sleep(2.0)
        
        # directly move in the y direction or the z direction
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
	
	# A Twist also has three rotational velocities (in radians per second).
	# The Stretch will only respond to rotations around the z (vertical) axis
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
        #rospy.loginfo("Trying to move")
        self.pub.publish(command)
        

    def find_tag(self, tag_name='docking_station'):
        """
        A function that actuates the camera to search for a defined ArUco tag
        marker. Then the function returns the pose.
        :param self: The self reference.
        :param tag_name: A string value of the ArUco marker name.

        :returns transform: The docking station's TransformStamped message.
        """
        # Create dictionaries to get the head in its initial position
        pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
        self.send_command(pan_command)
        tilt_command = {'joint': 'joint_head_tilt', 'position': self.min_tilt_position}
        self.send_command(tilt_command)

        # Nested for loop to sweep the joint_head_pan and joint_head_tilt in increments
        for i in range(self.tilt_num_steps):
            for j in range(self.pan_num_steps):
                # Update the joint_head_pan position by the pan_step_size
                pan_command = {'joint': 'joint_head_pan', 'delta': self.pan_step_size}
                self.send_command(pan_command)

                # Give time for system to do a Transform lookup before next step
                rospy.sleep(0.2)

                # Use a try-except block
                try:
                    # Look up transform between the base_link and requested ArUco tag
                    transform = self.tf_buffer.lookup_transform('base_link',
                                                            tag_name,
                                                            rospy.Time())
                    rospy.loginfo("Found Requested Tag: \n%s", transform)

                    # Publish the transform
                    self.transform_pub.publish(transform)

                    # Return the transform
                    return transform
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    continue

            # Begin sweep with new tilt angle
            pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
            self.send_command(pan_command)
            tilt_command = {'joint': 'joint_head_tilt', 'delta': self.tilt_step_size}
            self.send_command(tilt_command)
            rospy.sleep(.25)

        # Notify that the requested tag was not found
        rospy.loginfo("The requested tag '%s' was not found", tag_name)



    def arm_position(self, data):
        '''Arm move up command'''
        reached = False
        z = data.z
        reached = False
        while not rospy.is_shutdown() and not reached:
            command = {'joint': 'joint_lift', 'delta': z}
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.1)

            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(0.25)
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]

            delta = command['delta']
            new_value = delta
            point.positions = [new_value]

            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.trajectory_client.send_goal(trajectory_goal)


    # TODO: test code as is, BUT REMOVE THE JOINT_LIFT from the open wrist. IF THIS DOES NOT WORK THEN WE ADD the data part into open wrist and change the JOINT_LIFT value
    
    def open_wrist(self):
        open_point = JointTrajectoryPoint()
        open_point.time_from_start = rospy.Duration(0.000)
        open_point.positions = [0.85, 0.1, 1.75] #  0.16
        
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw'] #, 'joint_gripper_finger_left'
        trajectory_goal.trajectory.points = [open_point]
        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'
        
        self.trajectory_client.send_goal(trajectory_goal)
        rospy.loginfo('Sent open wrist goal = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()    
        
    
    def close_wrist(self):
        
        close_point = JointTrajectoryPoint()
        close_point.time_from_start = rospy.Duration(0.000)
        close_point.positions = [-0.1]
        
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = ['joint_gripper_finger_left']
        trajectory_goal.trajectory.points = [close_point]
        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'
        
        self.trajectory_client.send_goal(trajectory_goal)
        rospy.loginfo('Sent open wrist goal = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()    
    
    def main(self):
        """
        Function that initiates the issue_command function.
        :param self: The self reference.
        """
        # The arguments of the main function of the hm.HelloNode class are the
        # node_name, node topic namespace, and boolean (default value is true)
        hm.HelloNode.main(self, 'aruco_tag_locator', 'aruco_tag_locator', wait_for_first_pointcloud=False)
        # hm.HelloNode.main(self, 'open_wrist', 'open_wrist', wait_for_first_pointcloud=False)

        # Create a StaticTranformBoradcaster Node. Also, start a Tf buffer that
        # will store the tf information for a few seconds.Then set up a tf listener, which
        # will subscribe to all of the relevant tf topics, and keep track of the information
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Give the listener some time to accumulate transforms
        rospy.sleep(1.0)

        # Notify Stretch is searching for the ArUco tag with `rospy.loginfo()`
        rospy.loginfo('Searching for bottle ArUco tag.')
        self.move_forward_one_second()

        # Search for the ArUco marker for the docking station
        self.open_wrist()
        time.sleep(1.0)
        #self.move_forward_one_second()
        #time.sleep(2)
        #self.close_wrist()
        
        pose = self.find_tag("bottle")
        return pose

if __name__ == '__main__':
    # Use a try-except block
    try:
        ########################
        # Create the socket 
        ########################
        #listenSocket = socket(AF_INET, SOCK_DGRAM)
        #listenSocket.bind(('', SERVERPORT))
        
        #addr = (HOST, SERVERPORT)
        
        #newSession = session.Session(listenSocket, addr)
        #rospy.loginfo("Socket created, waiting for message")
        #session_packet = newSession.receive_message()
        #rospy.loginfo("The received message '%s' from the client", session_packet['data'])
        #newSession.shutdown()
    	
    	
    	
    	
    	# Initialize the node, and call it "move"
    	# Instantiate the `LocateArUcoTag()` object	
        node = LocateArUcoTag()
    	# Run the `main()` method
        bottlePose = node.main()
        # robot = stretch_body.robot.Robot()
        # robot.startup()

        # robot.base.translate_by(x_m=0.5)
        # robot.push_command()
        # time.sleep(4.0)

        # robot.base.set_rotational_velocity(v_r=0.1) #switch to velocity controller
        # robot.push_command()
        # time.sleep(4.0)

        # robot.base.set_rotational_velocity(v_r=0.0) #stop motion
        # robot.push_command()

        # robot.stop()
    	# For this example, we want to publish at 10Hz
       
    	# This will loop until ROS shuts down the node.  This can be done on the
    	# command line with a ctrl-C, or automatically from roslaunch
        while not rospy.is_shutdown():
            # Run the move_foward function in the Move class
            #while bottlePose is None:
                #node.move_forward_one_second()
                #bottlePose = node.main()
            # node.move_forward()
            if bottlePose is not None:
                node.move_forward(bottlePose)
            else:
                node.move_forward_one_second()
            
                #rospy.loginfo(str(bottlePose.transform.translation))
            # Do an idle wait to control the publish rate. If we don't control the
            # rate, the node will publish as fast as it can, and consume all of the
            # available CPU resources. This will also add a lot of network traffic,
            # possibly slowing down other things
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
