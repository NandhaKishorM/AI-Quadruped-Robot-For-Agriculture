#!/usr/bin/env python3

"""
Class for sending keyboard commands to spot micro walk node, control velocity, yaw rate, and walk event 
"""
import rospy
import sys, select, termios, tty # For terminal keyboard key press reading
from std_msgs.msg import Float32, Bool 
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from math import pi
import cv2
import depthai as dai
import numpy
import time
import math
import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
d = {'w': 0, 'x': 0, 'y': 0,'z':0}
d1 = {'w': 0, 'x': 0, 'y': 0,'z':0}
values = {'val':0}
yaw_val = {"val":0}
 
def resizewin(width, height):
    """
    For resizing window
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)






def draw(w, nx, ny, nz):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    drawText((-2.6, 1.8, 2), "OAK-D", 18)
    drawText((-2.6, 1.6, 2), "Module to visualize quaternion from OAK-D IMU", 16)
    drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

    [yaw, pitch , roll] = quat_to_ypr([w, nx, ny, nz])
   

   
    drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
    glRotatef(2 * math.acos(w) * 180.00/math.pi, -1 * nx, nz, ny)
   

    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def quat_to_ypr(q):
    yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    pitch = -math.sin(2.0 * (q[1] * q[3] - q[0] * q[2]))
    roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    yaw   -= -1.31  # Declination at Pilicode, Kerala is - 1 degress 31 min
    roll  *= 180.0 / math.pi
    return [yaw, pitch, roll]
def diff(q):
    yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    pitch = -math.sin(2.0 * (q[1] * q[3] - q[0] * q[2]))
    roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    yaw   -= -1.31  # Declination at Pilicode, Kerala is - 1 degress 31 min
    roll  *= 180.0 / math.pi
    
    
    return [yaw,pitch,roll]






# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
imu = pipeline.createIMU()
xlinkOut = pipeline.createXLinkOut()

xlinkOut.setStreamName("imu")

# enable ROTATION_VECTOR at 400 hz rate
imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)
# above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
imu.setBatchReportThreshold(1)
# maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
# if lower or equal to batchReportThreshold then the sending is always blocking on device
# useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
imu.setMaxBatchReports(10)

# Link plugins IMU -> XLINK
imu.out.link(xlinkOut.input)

msg = """
Spot Micro Walk Command

Enter one of the following options:
-----------------------------
quit: stop and quit the program
walk: Start walk mode and keyboard motion control
stand: Stand robot up

Keyboard commands for body motion 
---------------------------
   q   w   e            u
   a   s   d    
            

  u: Quit body motion command mode and go back to rest mode
  w: Increment forward speed command / decrease pitch angle
  a: Increment left speed command / left roll angle
  s: Increment backward speed command / increase pitch angle
  d: Increment right speed command / right roll angle
  q: Increment body yaw rate command / left yaw angle (negative left, positive right) 
  e: Increment body yaw rate command / right yaw angle (negative left, positive right) 
  f: In walk mode, zero out all rate commands.

  anything else : Prompt again for command


CTRL-C to quit
"""
valid_cmds = ('quit','Quit','walk','stand','idle', 'angle_cmd')

# Global body motion increment values
speed_inc = 0.02
yaw_rate_inc = 3*pi/180
angle_inc = 2.5*pi/180

class SpotMicroKeyboardControl():
    def __init__(self):

        self._angle_cmd_msg = Vector3()
        self._angle_cmd_msg.x = 0
        self._angle_cmd_msg.y = 0
        self._angle_cmd_msg.z = 0

        self._speed_cmd_msg = Vector3()
        self._speed_cmd_msg.x = 0
        self._speed_cmd_msg.y = 0
        self._speed_cmd_msg.z = 0

        self._vel_cmd_msg = Twist()
        self._vel_cmd_msg.linear.x = 0
        self._vel_cmd_msg.linear.y = 0
        self._vel_cmd_msg.linear.z = 0
        self._vel_cmd_msg.angular.x = 0
        self._vel_cmd_msg.angular.y = 0
        self._vel_cmd_msg.angular.z = 0
        
        self._walk_event_cmd_msg = Bool()
        self._walk_event_cmd_msg.data = True # Mostly acts as an event driven action on receipt of a true message

        self._stand_event_cmd_msg = Bool()
        self._stand_event_cmd_msg.data = True
       
        self._idle_event_cmd_msg = Bool()
        self._idle_event_cmd_msg.data = True

        rospy.loginfo("Setting Up the Spot Micro Keyboard Control Node...")

        # Set up and title the ros node for this code
        rospy.init_node('spot_micro_keyboard_control')

        # Create publishers for commanding velocity, angle, and robot states
        self._ros_pub_angle_cmd      = rospy.Publisher('/angle_cmd',Vector3,queue_size=1)
        self._ros_pub_vel_cmd        = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self._ros_pub_walk_cmd       = rospy.Publisher('/walk_cmd',Bool, queue_size=1)
        self._ros_pub_stand_cmd      = rospy.Publisher('/stand_cmd',Bool,queue_size=1)
        self._ros_pub_idle_cmd       = rospy.Publisher('/idle_cmd',Bool,queue_size=1)

        rospy.loginfo("Keyboard control node publishers corrrectly initialized")

        # Setup terminal input reading, taken from teleop_twist_keyboard
        self.settings = termios.tcgetattr(sys.stdin)

        rospy.loginfo("Keyboard control node initialization complete")

    def reset_all_motion_commands_to_zero(self):
        '''Reset body motion cmd states to zero and publish zero value body motion commands'''
        
        self._vel_cmd_msg.linear.x = 0
        self._vel_cmd_msg.linear.y = 0
        self._vel_cmd_msg.linear.z = 0
        self._vel_cmd_msg.angular.x = 0
        self._vel_cmd_msg.angular.y = 0
        self._vel_cmd_msg.angular.z = 0

        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

    def reset_all_angle_commands_to_zero(self):
        '''Reset angle cmd states to zero and publish them'''

        self._angle_cmd_msg.x = 0
        self._angle_cmd_msg.y = 0
        self._angle_cmd_msg.z = 0

        self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        # Publish all body motion commands to 0
        self.reset_all_motion_commands_to_zero()
        rospy.loginfo('Main keyboard control loop started.')

        # Main loop
        while not rospy.is_shutdown():
            # Prompt user with keyboard command information, and wait for input keystroke
            print(msg)
            userInput = input("Command?: ")

            if userInput not in valid_cmds:
                rospy.logwarn('Invalid keyboard command entered: %s', userInput)
            else:
                if userInput == 'quit':
                    rospy.loginfo("Exiting keyboard control node...")
                    break
                
                elif userInput == 'stand':
                    #Publish stand command event
                    self._ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                    rospy.loginfo('Stand command issued from keyboard.')
                
                elif userInput == 'idle':
                    #Publish idle command event
                    self._ros_pub_idle_cmd.publish(self._idle_event_cmd_msg)
                    rospy.loginfo('Idle command issued from keyboard.')

                elif userInput == 'angle_cmd':
                    # Reset all angle commands
                    self.reset_all_angle_commands_to_zero()
                    rospy.loginfo('Entering keyboard angle command mode.')
                    
                    # Enter loop to act on user command
                    print('Enter command, u to go back to command select: ')

                    while (1):
                        print('Cmd Values: phi: %1.3f deg, theta: %1.3f deg, psi: %1.3f deg '\
                                %(self._angle_cmd_msg.x*180/pi, self._angle_cmd_msg.y*180/pi, self._angle_cmd_msg.z*180/pi))
                        with dai.Device(pipeline) as device:


                                def timeDeltaToMilliS(delta) -> float:
                                    return delta.total_seconds()*1000

                                # Output queue for imu bulk packets
                                imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
                                baseTs = None
                                counter = 0
                                while True:
                                    imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

                                    imuPackets = imuData.packets
                                    for imuPacket in imuPackets:
                                        rVvalues = imuPacket.rotationVector

                                        rvTs = rVvalues.timestamp.get()
                                        if baseTs is None:
                                            baseTs = rvTs
                                        rvTs = rvTs - baseTs

                                        imuF = "{:.06f}"
                                        tsF  = "{:.03f}"

                                        print(f"Rotation vector timestamp: {tsF.format(timeDeltaToMilliS(rvTs))} ms")
                                        print(f"Quaternion: i: {imuF.format(rVvalues.i)} j: {imuF.format(rVvalues.j)} "
                                            f"k: {imuF.format(rVvalues.k)} real: {imuF.format(rVvalues.real)}")
                                        print(f"Accuracy (rad): {imuF.format(rVvalues.accuracy)}")
                                          
                                        
                                        if counter == 100:
                                            print("updating d1")
                                            d1.update({'w':float("{0:.06f}".format(rVvalues.real)),'x':float("{0:.06f}".format(rVvalues.i)),'y':float("{0:.06f}".format(rVvalues.j)),'z':float("{0:.06}".format(rVvalues.k))})
                                        counter +=1
                                        if counter ==200:
                                            counter =101
                                        print("counter",counter)
                                            


                                        d.update({'w':float("{0:.06f}".format(rVvalues.real)),'x':float("{0:.06f}".format(rVvalues.i)),'y':float("{0:.06f}".format(rVvalues.j)),'z':float("{0:.06}".format(rVvalues.k))})
                                        video_flags = OPENGL | DOUBLEBUF
                                        pygame.init()
                                        screen = pygame.display.set_mode((640, 480), video_flags)
                                        pygame.display.set_caption("OAK-D IMU orientation visualization")
                                        resizewin(640, 480)
                                        init()
                                        frames = 0
                                        ticks = pygame.time.get_ticks()
                                    
                                        event = pygame.event.poll()

                                        w1 = d1["w"]
                                        x1 = d1["x"]
                                        y1 = d1["y"]
                                        z1 = d1["z"]
                                    
                                        w = d["w"]
                                        x = d["x"]
                                        y = d["y"]
                                        z = d["z"]
                                        [val1,val2,val3] = diff([w1,x1,y1,z1])
                                        [val4,val5,val6] = diff([w,x,y,z])
                                        print("initial", [val1,val2,val3])
                                        print("final",[val4,val5,val6] )
                                        yaw_res = int(val4-val1)
                                        print("resultant yaw",yaw_res)
                                        pitch_res = int(val5-val2)
                                        print("resultant pitch",pitch_res)
                                        roll_res = int(val6-val3)
                                        print("resultant roll",roll_res)
                                        if w1 != 0 :
                                            if val4>val1:
                                                if yaw_res >=20:

                                                 
                                                    for i in range(1,3):
                                                        print("press a and move right")

                                                        self._angle_cmd_msg.x = self._angle_cmd_msg.x + angle_inc
                                                        self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)
                                                    time.sleep(1)
                                            
                                                    for i in range(1,3):
                                                        print("press d and move left")
                                                        self._angle_cmd_msg.x = self._angle_cmd_msg.x - angle_inc
                                                        self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)
                                                            #time.sleep(1)
                                            else:
                                                if abs(yaw_res)>=50:
                                                 
                                                    for i in range(1,3):
                                                        print("press d and move left")
                                                        self._angle_cmd_msg.x = self._angle_cmd_msg.x - angle_inc
                                                        self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)
                                                    time.sleep(1)
                                                    
                                                
                                                    for i in range(1,3):
                                                    
                                                        self._angle_cmd_msg.x = self._angle_cmd_msg.x + angle_inc
                                                        self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

                                                        #time.sleep(1)
                       

                        
                   

                                        draw(w, x, y, z)
                                    
                                        pygame.display.flip()
                                        frames += 1
                                    if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                                        break

                       
                        userInput = self.getKey()

                        if userInput == 'u':
                            # Break out of angle command mode 
                            break

                        elif userInput not in ('w','a','s','d','q','e','u'):
                            rospy.logwarn('Invalid keyboard command issued in angle command mode: %s', userInput)
                        else:
                            if userInput == 'w':
                                self._angle_cmd_msg.y = self._angle_cmd_msg.y - angle_inc
                                self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)
                            
                            elif userInput == 's':
                                self._angle_cmd_msg.y = self._angle_cmd_msg.y + angle_inc
                                self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

                            elif userInput == 'q':
                                self._angle_cmd_msg.z = self._angle_cmd_msg.z + angle_inc
                                self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

                            elif userInput == 'e':
                                self._angle_cmd_msg.z = self._angle_cmd_msg.z - angle_inc
                                self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)
                            # Pipeline is defined, now we can connect to the device
                            
                          
                          

                elif userInput == 'walk':
                    # Reset all body motion commands to zero
                    self.reset_all_motion_commands_to_zero()

                    # Publish walk event
                    self._ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)
                    rospy.loginfo('Idle command issued from keyboard.')

                    # Prompt user with info and enter loop to act on user command
                    print('Enter command, u to go back to stand mode: ')

                    while (1):
                        print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s '\
                                %(self._vel_cmd_msg.linear.x,self._vel_cmd_msg.linear.y,self._vel_cmd_msg.angular.z*180/pi))
                       
                        userInput = self.getKey()

                        if userInput == 'u':
                            # Send stand event message, this will take robot back to standing mode
                            self._ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                            rospy.loginfo('Stand command issued from keyboard.')
                            break

                        elif userInput not in ('w','a','s','d','q','e','u','f'):
                            print('Key not in valid key commands, try again')
                            rospy.logwarn('Invalid keyboard command issued in walk mode: %s', userInput)
                        else:
                            if userInput == 'w':
                                self._vel_cmd_msg.linear.x = self._vel_cmd_msg.linear.x + speed_inc
                                self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                            
                            elif userInput == 's':
                                self._vel_cmd_msg.linear.x = self._vel_cmd_msg.linear.x - speed_inc
                                self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

                            elif userInput == 'a':
                                self._vel_cmd_msg.linear.y = self._vel_cmd_msg.linear.y - speed_inc
                                self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                            
                            elif userInput == 'd':
                                self._vel_cmd_msg.linear.y = self._vel_cmd_msg.linear.y + speed_inc
                                self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

                            elif userInput == 'q':
                                self._vel_cmd_msg.angular.z = self._vel_cmd_msg.angular.z - yaw_rate_inc
                                self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

                            elif userInput == 'e':
                                self._vel_cmd_msg.angular.z = self._vel_cmd_msg.angular.z + yaw_rate_inc
                                self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

                            elif userInput == 'f':
                                self._vel_cmd_msg.linear.x = 0
                                self._vel_cmd_msg.linear.y = 0
                                self._vel_cmd_msg.angular.z = 0

                                self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                                rospy.loginfo('Command issued to zero all rate commands.')

                                
if __name__ == "__main__":
    smkc     = SpotMicroKeyboardControl()
    smkc.run()
