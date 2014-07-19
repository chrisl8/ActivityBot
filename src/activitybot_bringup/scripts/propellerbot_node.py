#!/usr/bin/env python
'''
This is a copy of PropellerComm.py adapted to replace turtlebot_node
in the turtlebot stack.
This is a copy of arduino.py by Dr. Rainer Hessmer
https://code.google.com/p/drh-robotics-ros/
Edited and stripped down for my use. All brilliance contained herein is credited
to Dr. Rainer Hessmer, and all mistakes and poor code are my own.

  Copyright (c) 2011 Dr. Rainer Hessmer.  All right reserved.
  Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''
# NOTE: This script REQUIRES parameters to be loaded from param/encoders.yaml!
#import roslib; roslib.load_manifest('activitybot') # http://wiki.ros.org/roslib
import rospy
import tf
import math
from math import sin, cos, pi
import sys

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
#from activitybot.srv import *
#from activitybot.msg import *

#for Turtlebot stack from turtlebot_node.py
from create_driver import Turtlebot, MAX_WHEEL_SPEED, DriverError
from create_node.msg import TurtlebotSensorState, Drive, Turtle
from create_node.srv import SetTurtlebotMode,SetTurtlebotModeResponse, SetDigitalOutputs, SetDigitalOutputsResponse
from create_node.diagnostics import TurtlebotDiagnostics
from geometry_msgs.msg import Point, Pose, Pose2D, PoseWithCovariance, TwistWithCovariance, Vector3
from create_node.covariances import ODOM_POSE_COVARIANCE, ODOM_POSE_COVARIANCE2, ODOM_TWIST_COVARIANCE, ODOM_TWIST_COVARIANCE2
from sensor_msgs.msg import JointState
import dynamic_reconfigure.server
from create_node.cfg import TurtleBotConfig
import create_node.robot_types as robot_types

from SerialDataGateway import SerialDataGateway

class PropellerComm(object):
    '''
    Helper class for communicating with a Propeller board over serial port
    '''

    CONTROLLER_INITIALIZING = 1;
    CONTROLLER_IS_READY = 2;

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        '''
        Initializes the receiver class. 
        port: The serial port to listen to.
        baudrate: Baud rate for the serial communication
        '''

        self.default_port = '/dev/ttyUSB0' # Note that the Propeller board must be plugged in BEFORE anything else to secure ttyUSB0
        self.default_update_rate = 30.0

        self.robot = Turtlebot()
        self.sensor_handler = None
        self.sensor_state = TurtlebotSensorState()
        self.req_cmd_vel = None

        rospy.init_node('turtlebot')
        self._init_params()
        self._init_pubsub()
        
        self._pos2d = Pose2D() # 2D pose for odometry

        self._diagnostics = TurtlebotDiagnostics()
        if self.has_gyro:
            from create_node.gyro import TurtlebotGyro
            self._gyro = TurtlebotGyro()
        else:
            self._gyro = None
            
        #dynamic_reconfigure.server.Server(TurtleBotConfig, self.reconfigure)

        self._Counter = 0

        rospy.init_node('turtlebot')

        port = rospy.get_param("~port", "")
        baudRate = int(rospy.get_param("~baudRate", 0))

        rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))

        # subscriptions
        rospy.Subscriber("cmd_vel", Twist, self._HandleVelocityCommand)
        rospy.Subscriber("cmd_vel_mux/input/teleop", Twist, self._HandleVelocityCommand)
        self._SerialPublisher = rospy.Publisher('serial', String)

        # The Odometry Transform is done in/with the robot_pose_ekf now
        #self._OdometryTransformBroadcaster = tf.TransformBroadcaster()
        self._OdometryPublisher = rospy.Publisher("odom", Odometry)

        # We don't need to broadcast a transform, as it is static and contained within the URDF files
        #self._SonarTransformBroadcaster = tf.TransformBroadcaster()
        self._SonarPublisher = rospy.Publisher("sonar_scan", LaserScan)

        self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)

    def _init_params(self):
        self.port = rospy.get_param('~port', self.default_port)
        self.robot_type = rospy.get_param('~robot_type', 'create')
        #self.baudrate = rospy.get_param('~baudrate', self.default_baudrate)
        self.update_rate = rospy.get_param('~update_rate', self.default_update_rate)
        self.drive_mode = rospy.get_param('~drive_mode', 'twist')
        self.has_gyro = rospy.get_param('~has_gyro', False) # gyro Change if you get one
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
        self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
        self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))
        self.stop_motors_on_bump = rospy.get_param('~stop_motors_on_bump', True)
        self.min_abs_yaw_vel = rospy.get_param('~min_abs_yaw_vel', None)
        self.max_abs_yaw_vel = rospy.get_param('~max_abs_yaw_vel', None)
        self.publish_tf = rospy.get_param('~publish_tf', False)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.operate_mode = rospy.get_param('~operation_mode', 3)

        rospy.loginfo("serial port: %s"%(self.port))
        rospy.loginfo("update_rate: %s"%(self.update_rate))
        rospy.loginfo("drive mode: %s"%(self.drive_mode))
        rospy.loginfo("has gyro: %s"%(self.has_gyro))

    def _init_pubsub(self):
        # Instead of publishing a stream of pointless transforms,
        # How about if I just make the joint static in the URDF?
        # create.urdf.xacro:
        # <joint name="right_wheel_joint" type="fixed">
        # NOTE This may prevent Gazebo from working with this model
        #self.joint_states_pub = rospy.Publisher('joint_states', JointState)

        # This is the Turtlebot node instance, the Propeller code uses another line above.
        #self.odom_pub = rospy.Publisher('odom', Odometry)

        self.sensor_state_pub = rospy.Publisher('~sensor_state', TurtlebotSensorState)
        self.operating_mode_srv = rospy.Service('~set_operation_mode', SetTurtlebotMode, self.set_operation_mode)
        self.digital_output_srv = rospy.Service('~set_digital_outputs', SetDigitalOutputs, self.set_digital_outputs)

        '''
        # This is from the Turtlebot node code.
        # These drive input commands are handled elsewhere in
        # the Propeller code. You can find it by searching for the cmd_vel subscription.
        if self.drive_mode == 'twist':
            self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel)
            self.drive_cmd = self.robot.direct_drive
        elif self.drive_mode == 'drive':
            self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Drive, self.cmd_vel)
            self.drive_cmd = self.robot.drive
        elif self.drive_mode == 'turtle':
            self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Turtle, self.cmd_vel)
            self.drive_cmd = self.robot.direct_drive
        else:
            rospy.logerr("unknown drive mode :%s"%(self.drive_mode))
        '''

        self.transform_broadcaster = None
        if self.publish_tf:
            self.transform_broadcaster = tf.TransformBroadcaster()

    def set_operation_mode(self,req):
        if not self.robot.sci:
            rospy.logwarn("Create : robot not connected yet, sci not available")
            return SetTurtlebotModeResponse(False)

        self.operate_mode = req.mode

        if req.mode == 1: #passive
            self._robot_run_passive()
        elif req.mode == 2: #safe
            self._robot_run_safe()
        elif req.mode == 3: #full
            self._robot_run_full()
        else:
            rospy.logerr("Requested an invalid mode.")
            return SetTurtlebotModeResponse(False)
        return SetTurtlebotModeResponse(True)

    def _set_digital_outputs(self, outputs):
        assert len(outputs) == 3, 'Expecting 3 output states.'
        byte = 0
        for output, state in enumerate(outputs):
            byte += (2 ** output) * int(state)
        self.robot.set_digital_outputs(byte)
        self.sensor_state.user_digital_outputs = byte

    def set_digital_outputs(self,req):
        if not self.robot.sci:
            raise Exception("Robot not connected, SCI not available")
            
        outputs = [req.digital_out_0,req.digital_out_1, req.digital_out_2]
        self._set_digital_outputs(outputs)
        return SetDigitalOutputsResponse(True)

    def reconfigure(self, config, level):
        self.update_rate = config['update_rate']
        self.drive_mode = config['drive_mode']
        self.has_gyro = config['has_gyro']
        if self.has_gyro:
            self._gyro.reconfigure(config, level)
        self.odom_angular_scale_correction = config['odom_angular_scale_correction']
        self.odom_linear_scale_correction = config['odom_linear_scale_correction']
        self.cmd_vel_timeout = rospy.Duration(config['cmd_vel_timeout'])
        self.stop_motors_on_bump = config['stop_motors_on_bump']
        self.min_abs_yaw_vel = config['min_abs_yaw_vel']
        self.max_abs_yaw_vel = config['max_abs_yaw_vel']
        return config

    def cmd_vel(self, msg): # This is the Turtlebot node drive code, not used by Propeller
        # This could be interesting to study if we have trouble with ArloBot
        # Clamp to min abs yaw velocity, to avoid trying to rotate at low
        # speeds, which doesn't work well.
        if self.min_abs_yaw_vel is not None and msg.angular.z != 0.0 and abs(msg.angular.z) < self.min_abs_yaw_vel:
            msg.angular.z = self.min_abs_yaw_vel if msg.angular.z > 0.0 else -self.min_abs_yaw_vel
        # Limit maximum yaw to avoid saturating the gyro
        if self.max_abs_yaw_vel is not None and self.max_abs_yaw_vel > 0.0 and msg.angular.z != 0.0 and abs(msg.angular.z) > self.max_abs_yaw_vel: 
            msg.angular.z = self.max_abs_yaw_vel if msg.angular.z > 0.0 else -self.max_abs_yaw_vel 
        if self.drive_mode == 'twist':
            # convert twist to direct_drive args
            ts  = msg.linear.x * 1000 # m -> mm
            tw  = msg.angular.z  * (robot_types.ROBOT_TYPES[self.robot_type].wheel_separation / 2) * 1000 
            # Prevent saturation at max wheel speed when a compound command is sent.
            if ts > 0:
                ts = min(ts,   MAX_WHEEL_SPEED - abs(tw))
            else:
                ts = max(ts, -(MAX_WHEEL_SPEED - abs(tw)))
            self.req_cmd_vel = int(ts - tw), int(ts + tw)
        elif self.drive_mode == 'turtle':
            # convert to direct_drive args
            ts  = msg.linear * 1000 # m -> mm
            tw  = msg.angular  * (robot_types.ROBOT_TYPES[self.robot_type].wheel_separation / 2) * 1000 
            self.req_cmd_vel = int(ts - tw), int(ts + tw)
        elif self.drive_mode == 'drive':
            # convert twist to drive args, m->mm (velocity, radius)
            self.req_cmd_vel = msg.velocity * 1000, msg.radius * 1000

    def _HandleReceivedLine(self,  line): # This is Propeller specific
        self._Counter = self._Counter + 1
        #rospy.logdebug(str(self._Counter) + " " + line)
        #if (self._Counter % 50 == 0):
        self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))

        if (len(line) > 0):
            lineParts = line.split('\t')
            if (lineParts[0] == 'o'):
                self._BroadcastOdometryInfo(lineParts)
                return
            if (lineParts[0] == 'i'):
                self._InitializeDriveGeometry()
                return

    def _BroadcastOdometryInfo(self, lineParts):
        # This broadcasts ALL info from the Propeller based robot every time data comes in
        partsCount = len(lineParts)

        #rospy.logwarn(partsCount)
        if (partsCount  < 10): # Just discard short lines, increment this as lines get longer
            pass
        
        try:
            x = float(lineParts[1])
            y = float(lineParts[2])
            theta = float(lineParts[3])
            
            vx = float(lineParts[4])
            omega = float(lineParts[5])
        
            #quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(theta / 2.0)
            quaternion.w = cos(theta / 2.0)
            
            
            rosNow = rospy.Time.now()
            
            # First, we'll publish the transform from frame odom to frame base_link over tf
            # Note that sendTransform requires that 'to' is passed in before 'from' while
            # the TransformListener' lookupTransform function expects 'from' first followed by 'to'.
            #This transform conflicts with transforms built into the Turtle stack
            # http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
            # This is done in/with the robot_pose_ekf because it can integrate IMU/gyro data
            # using an "extended Kalman filter"
            '''
            self._OdometryTransformBroadcaster.sendTransform(
                (x, y, 0), 
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rosNow,
                "base_link",
                "odom"
                )
            '''

            # next, we'll publish the odometry message over ROS
            odometry = Odometry()
            odometry.header.frame_id = "odom"
            odometry.header.stamp = rosNow
            odometry.pose.pose.position.x = x
            odometry.pose.pose.position.y = y
            odometry.pose.pose.position.z = 0
            odometry.pose.pose.orientation = quaternion

            odometry.child_frame_id = "base_link"
            odometry.twist.twist.linear.x = vx
            odometry.twist.twist.linear.y = 0
            odometry.twist.twist.angular.z = omega

            #for Turtlebot stack from turtlebot_node.py
            odometry.pose.covariance = ODOM_POSE_COVARIANCE
            odometry.twist.covariance = ODOM_TWIST_COVARIANCE

            self._OdometryPublisher.publish(odometry)

            # Joint State for Turtlebot stack
            # Note without this transform publisher the wheels will
            # be white, stuck at 0, 0, 0 and RVIZ will tell you taht
            # there is no transform from the wheel_links to the base_
            '''
            # Instead of publishing a stream of pointless transforms,
            # How about if I just make the joint static in the URDF?
            # create.urdf.xacro:
            # <joint name="right_wheel_joint" type="fixed">
            # NOTE This may prevent Gazebo from working with this model
            js = JointState(name = ["left_wheel_joint", "right_wheel_joint", "front_castor_joint", "back_castor_joint"],
                            position=[0,0,0,0], velocity=[0,0,0,0], effort=[0,0,0,0])
            js.header.stamp = rosNow
            self.joint_states_pub.publish(js)
            '''

            # Fake laser from "PING" Ultrasonic Sensor input:
            # http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
            # Transform: http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
            '''
            # We don't need to broadcast a transform,
                as it is static and contained within the URDF files
            self._SonarTransformBroadcaster.sendTransform(
                (0.1, 0.0, 0.2), 
                (0, 0, 0, 1),
                rosNow,
                "sonar_laser",
                "base_link"
                )
                '''
            # Some help: http://books.google.com/books?id=2ZL9AAAAQBAJ&pg=PT396&lpg=PT396&dq=fake+LaserScan+message&source=bl&ots=VJMfSYXApG&sig=s2YgiHTA3i1OjVyPxp2aAslkW_Y&hl=en&sa=X&ei=B_vDU-LkIoef8AHsooHICA&ved=0CG0Q6AEwCQ#v=onepage&q=fake%20LaserScan%20message&f=false
            num_readings = 360 # How about 1 per degree?
            laser_frequency = 100 # I'm not sure how to decide what to use here.
            #ranges = [1] * num_readings # Fill array with fake "1" readings for testing
            ranges = [0] * num_readings # Fill array with 0 and then overlap with real readings
            
            pingRange0 = int(lineParts[6]) / 100.0
            ranges[0] = pingRange0
            ranges[1] = pingRange0
            ranges[2] = pingRange0
            ranges[3] = pingRange0
            ranges[4] = pingRange0
            ranges[5] = pingRange0
            ranges[359] = pingRange0
            ranges[358] = pingRange0
            ranges[357] = pingRange0
            ranges[356] = pingRange0
            ranges[355] = pingRange0
            # Is there a more concise way to code that array fill?
            # LaserScan: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
            sonar_scan = LaserScan()
            sonar_scan.header.stamp = rosNow
            sonar_scan.header.frame_id = "ping_sensor_array"
            # For example:
            #scan.angle_min = -45 * M_PI / 180; // -45 degree
            #scan.angle_max = 45 * M_PI / 180;   // 45 degree
            # if you want to receive a full 360 degrees scan, you should try setting min_angle to -pi/2 and max_angle to 3/2 * pi.
            # Radians: http://en.wikipedia.org/wiki/Radian#Advantages_of_measuring_in_radians
            sonar_scan.angle_min = 0
            sonar_scan.angle_max = 2 * 3.14159 # Full circle
            sonar_scan.scan_time = 1 # I think this is only really applied for 3D scanning
            # Make sure the part you divide by num_readings is the same as your angle_max!
            # Might even make sense to use a variable here?
            sonar_scan.angle_increment = (2 * 3.14) / num_readings
            sonar_scan.time_increment = (1 / laser_frequency) / (num_readings)
            # From: http://www.parallax.com/product/28015
            # Range: approximately 1 inch to 10 feet (2 cm to 3 m)
            # This should be adjusted based on the imaginary distance between the actual laser
            # and the laser location in the URDF file. Or else the adjustment somewhere else?
            sonar_scan.range_min = 0.02 # in Meters Distances below this number will be ignored
            sonar_scan.range_max = 3 # in Meters Distances above this will be ignored
            sonar_scan.ranges = ranges
            # "intensity" is a value specific to each laser scanner model.
            # It can safely be ignored
            
            self._SonarPublisher.publish(sonar_scan)

        except:
            rospy.logwarn("Unexpected error:" + str(sys.exc_info()[0]))

    def _WriteSerial(self, message):
        self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
        self._SerialDataGateway.Write(message)

    def Start(self):
        rospy.logdebug("Starting")
        self._SerialDataGateway.Start()

    def Stop(self):
        rospy.logdebug("Stopping")
        self._SerialDataGateway.Stop()
        
    def _HandleVelocityCommand(self, twistCommand): # This is Propeller specific
        """ Handle movement requests. """
        v = twistCommand.linear.x        # m/s
        omega = twistCommand.angular.z      # rad/s
        rospy.logdebug("Handling twist command: " + str(v) + "," + str(omega))
        message = 's,%.3f,%.3f\r' % (v, omega)
        rospy.logdebug("Sending speed command message: " + message)
        self._WriteSerial(message)

    def _InitializeDriveGeometry(self): # This is Propeller specific
        #wheelDiameter = rospy.get_param("~driveGeometry/wheelDiameter", "0")
        trackWidth = rospy.get_param("~driveGeometry/trackWidth", "0")
        #countsPerRevolution = rospy.get_param("~driveGeometry/countsPerRevolution", "0")
        distancePerCount = rospy.get_param("~driveGeometry/distancePerCount", "0")

        #wheelDiameterParts = self._GetBaseAndExponent(wheelDiameter)
        #trackWidthParts = self._GetBaseAndExponent(trackWidth)

        message = 'd,%f,%f\r' % (trackWidth, distancePerCount)
        rospy.logdebug("Sending drive geometry params message: " + message)
        self._WriteSerial(message)

if __name__ == '__main__':
    propellercomm = PropellerComm()
    try:
        propellercomm.Start()
        rospy.spin()

    except rospy.ROSInterruptException:
        propellercomm.Stop()


