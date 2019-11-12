"""
Controls Baxter using any game pad / joystick via the logitech and Motion modules.
Designed for Logitech controllers, may work for other controllers too.
Converted by Kris Hauser Oct 2015 from a script written by Peter Moran,
July 2015.job
SETUP:
Before running this script, you must run the system state service and the
controller dispatcher.
CONTROLS:
Controls are modified when special control buttons are pressed to create combinations, noted below.
Always:
    Right trigger -- switch between precision grip and power grip
No modifier (End effector velocity mode, ie translations):
    Right stick x -- move end effector along Baxter's y axis (ie side to side)
    Right stick y -- move end effector along Baxter's x axis (ie outward)
    Left stick x -- gripper open and close
    Left stick y -- move end effector along Baxter's z axis (ie vertically)
Left bumper (End effector rotational velocity mode):
    Right stick x -- rotate end effector about its x axis
    Right stick y -- rotate end effector about its y axis
    Left stick x -- rotate end effector about its z axis
Right bumper (Joint velocity mode):
    Right stick x -- rotate gripper
    Right stick y -- bend wrist
    Left stick x -- rotate elbow
    Left stick y -- bend elbow
Buttons:terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
    A is used to hold the position of the hand
    B is used to change hand pershape
    X is used to get the arm to initial position
    Y is used to switch between arm and base
    Dpad switch camera view
"""
pick = None
workpls = None
J = [0, 0, 0, 0, 0, 0, 0]
xb = None
yb = None
zb = None
pickLimb = "Out"
holdingObj = False
hasMoved = {}
marker_state = None
pickId = 9999
placeId = 9999
grabbing = False
grabAmount = 0
left_pose = None
z_height = 999
got_to_waypoint = False
command = ""
offsetx = 0.0
offsety = 0.0
offsetz = 0.0
placing = False
hand = None

# Import Modules
import os
import sys
import threading
import rospy
import asyncore
import subprocess

ebolabot_root = os.getenv("EBOLABOT_PATH", ".")
from Common.system_config import EbolabotSystemConfig

sys.path.append(ebolabot_root)
# for logitech module
sys.path.append(os.path.join(ebolabot_root, 'InputDevices/USBControllers'))
import gamepad
from task_generator import TaskGenerator
import time
import csv
from sspp.service import Service
from sspp.topic import MultiTopicListener
from OpenGL.GL import *
from klampt.vis import gldraw
from klampt.vis.glinterface import GLPluginInterface as GLPluginBase
from klampt.vis.glcommon import GLWidgetPlugin
from klampt.math import so3, se3, vectorops
from std_msgs.msg import String, Int16, Float32MultiArray, Int8, Int64, Bool
from geometry_msgs.msg import Pose
# from baxter_pykdl import baxter_kinematics
import numpy as np
from UI.utils.gripper_controller import *
from Trina-Point-And-Click.msg import Marker, MarkerArray

# imaging stuff
try:
    from PIL import Image
except ImportError as err:
    import Image

# set this -1 for view-centric control, looking at the face of the robot
viewToWorldScaleXY = 1
''' gripper Mode: power, precision '''
GripperMode = {'left': 'power', 'right': 'power'}

''' Hold Mode: free, hold '''
HoldMode = {'left': 'free', 'right': 'free'}

HoldPose = {'left': [1.0, 1.0, 1.0, 1.0], 'right': [1.0, 1.0, 1.0, 1.0]}

TuckPose = {}
TuckPose['left'] = [-0.05897088559570313, -0.9675583808532715, -1.2034079267211915, 1.7132575355041506,
                    0.6776360122741699, 1.0166457660095216, 2.475]
TuckPose['right'] = [0.05897088559570313, -0.9675583808532715, 1.2034079267211915, 1.7132575355041506,
                     -0.6776360122741699, 1.0166457660095216, -2.475]

TuckStatus = {'left': False, 'right': False}

gamepad_switch = "/transcript"  # to switch camera view
M_limit = 0.04
system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip', ('localhost', 4568))

class MarkerTaskGenerator(TaskGenerator):
    def __init__(self):
        self.serviceThread = None
        self.gripperController = None
        self.j = None
        self.limb = 'left'
        self.controlSet = 'arm'
        self.lastState = {}
        self.plugin = None
        # set initial values
        self.baseSensedVelocity = [0.0, 0.0, 0.0]
        self.baseCommandVelocity = [0.0, 0.0, 0.0]
        self.log = False
        # === joint control ===
        self.jointControlRatio = 0.4

        # == arm position ===
        self.ArmPosition = [[0.0] * 7, [0.0] * 7]
        self.robotEndEffectorPosition = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        self.robotEndEffectorTransform = [se3.identity(), se3.identity()]
        self.gripperPosition = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
        self.kin = {}

        # self.kin['left'] = baxter_kinematics('left')
        # self.kin['right'] = baxter_kinematics('right')
        self.last_driveVel = [0.0, 0.0, 0.0]
        self.near_singularity = False
        self.last_sigulatiry = False

        # flag to mark the data
        self.flag = 0
        # open log file
        timestr = time.strftime("%Y%m%d-%H%M%S")
        """self.csvfile_gamepad = open('data/gamepad_log' + timestr + '.csv', 'wb')
        fieldnames = ['rstick', 'lstick', 'LB', 'RB', 'LT', 'RT', 'B', 'A', 'X', 'Y', 'Dpad_x', 'Dpad_y',
                      'jointAngles', 'gripperStatus', 'eePosition', 'eeTransformation',
                      'baseSensedVelocity', 'baseCommandVelocity', 'time', 'Flag']
        self.gamepad_csv = csv.DictWriter(self.csvfile_gamepad, fieldnames=fieldnames)
        self.gamepad_csv.writeheader()
        self.switch_pub = None"""

        # == Autonomous ==
        self.lastPick = "Out"
        self.isAuto = False
        rospy.Subscriber('Detect', String, callback_pos)
        rospy.Subscriber('/MarkerArray', MarkerArray, callback_state)
        rospy.Subscriber('/Offsets', Pose, callback_offsets)
        rospy.Subscriber('/PickID', Int64, callback_pick)
        rospy.Subscriber('/PlaceID', Int64, callback_place)
        rospy.Subscriber('/Command', String, callback_command)
        rospy.Subscriber('/GripperSpeed', Int64, callback_speed)
        rospy.Subscriber('/GripperClosePercent', Int64, callback_percent)
        rospy.Subscriber('/GripperState', String, callback_gripper)
        rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, self.callback_pose)
        #rospy.Subscriber('/robot/limb/left/endpoint_state/pose', Pose, callback_pose)
        self.pub_l = rospy.Publisher('/left/UbirosGentle', Int8, queue_size = 1)
        self.pub_r = rospy.Publisher('/right/UbirosGentle', Int8, queue_size = 1)
        self.pub_state = rospy.Publisher('/CurrentStatus', Bool, queue_size = 1)
        
    def callback_pose(self, data):
        global hand
        hand = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        

    def name(self):
        return "Marker_autonomous"

    def init(self, world):
        assert self.j == None, "Init may only be called once"
        self.world = world
        #rospy.init_node("gamepad_node")
        # Connect to controller
        return True

    def start(self):
        global gamepad_switch
        try:
            self.j = gamepad.Gamepad()

        except:
            print
            "Gamepad not found"
            print
            "Note: Pygame reports there are " + str(gamepad.Gamepad.numJoys()) + " joysticks"
            return False

        if self.serviceThread is None:
            self.serviceThread = ServiceThread()
            self.serviceThread.start()
        if self.gripperController is None:
            self.gripperController = GripperController()
            self.gripperController.start()

        # if not self.j: return False
        self.limb = 'left'
        self._status = 'ok'
        # self.plugin = MyWidgetPlugin(self)
        self.lastState = {}

        # self.switch_pub = rospy.Publisher(gamepad_switch, String, queue_size=1)
        return True

    def status(self):
        # if self.j:
        #    return 'ok'
        # else:
        #    return 'error'
        return 'ok'

    def messages(self):
        return ["Controlling " + self.limb]

    def controlMode(self):
        # if len(self.lastState) == 0: return 'None'
        # if self.lastState['RB']:
        #    return 'Joint angles'
        # elif self.lastState['LB']:
        #    return "Cartesian rotation"
        # else:
        #    return "Cartesian position"
        return "Cartesian position"

    def stop(self):
        if self.serviceThread:
            self.serviceThread.kill()
            print("Waiting for thread join...")
            self.serviceThread.join()
            print("Done")
            self.serviceThread = None
        if self.gripperController:
            self.gripperController.kill()
            print("gripper control thread killed")
            self.gripperController = None
        self._status = ''
        self.plugin = None
        self.j.quit()

    def get(self):
        global marker_state
        
        j = self.j
        j.updateState()

        state = {}
        resp = marker_state
        
        if resp == None:
            # print "No data"
            return None

        state['workspace-markers'] = {}
        state['cup-markers'] = {}
        state['workspace-markers-vis'] = 0
        state['cup-markers-vis'] = 0

        for marker in resp:
            if not marker.id_number in hasMoved:
                    hasMoved[marker.id_number] = False
            if marker.workspace:
                state['workspace-markers'][marker.id_number] = marker
                if marker.visible:
                    state['workspace-markers-vis'] += 1

            else:
                state['cup-markers'][marker.id_number] = marker
                if marker.visible:
                    state['cup-markers-vis'] += 1

        if self.log:
            self.gamepad_csv.writerow(state)

        if len(self.lastState) > 0:
            res = self.do_logic(self.lastState, state)
        else:
            res = None

        self.lastState = state
        return res

    def do_logic(self, lastState, state):
        global GripperMode
        global HoldMode
        global HoldPose
        global pickLimb
        global xb, yb, zb
        global holdingObj
        global hasMoved
        global grabbing
        global grabAmount
        global left_pose
        global got_to_waypoint
        global command
        global pickId, placeId
        global offsetx, offsety, offsetz
        global placing

        # get robot state data
        self.getRobotStatus()
        robot_state = {'jointAngles': self.ArmPosition, 'gripperStatus': self.gripperPosition,
                       'eePosition': self.robotEndEffectorPosition, 'eeTransformation': self.robotEndEffectorTransform,
                       'baseSensedVelocity': self.baseSensedVelocity, 'baseCommandVelocity': self.baseCommandVelocity,
                       'time': int(round(time.time() * 1000)), 'Flag': self.flag}
        
        
        if self.log:
            print("definitely logging")
        # self.gamepad_csv.writerow(robot_state)
        # rstick = state['rstick']
        # lstick = state['lstick']
            
        if grabbing:
            if grabAmount < 90:
                grabAmount = grabAmount+0.3
        else:
            if grabAmount > 0:
                grabAmount = grabAmount -0.3
        
        gripPercent = grabAmount
        
        xpick = state['cup-markers'][pickId].transform.translation.x
        ypick = state['cup-markers'][pickId].transform.translation.y
        zpick = state['cup-markers'][pickId].transform.translation.z
        xplace = state['workspace-markers'][placeId].transform.translation.x
        yplace = state['workspace-markers'][placeId].transform.translation.y
        zplace = state['workspace-markers'][placeId].transform.translation.z
        zcalcpick = 3.281511*(xpick**2)*(ypick**2) + 2.962022*(xpick**2)*ypick - 0.239726*(xpick**2) - 3.52277*xpick*(ypick**2) - 4.048555*xpick*ypick + 0.153533*xpick + 0.632763*(ypick**2) + 1.195625*ypick + 0.999591
        zcalcplace = 3.281511*(xplace**2)*(yplace**2) + 2.962022*(xplace**2)*yplace - 0.239726*(xplace**2) - 3.52277*xplace*(yplace**2) - 4.048555*xplace*yplace + 0.153533*xplace + 0.632763*(yplace**2) + 1.195625*yplace + 0.999591
        
        if command == "home":
            TuckStatus[self.limb] = True
            self.pub_state.publish(False)
        else:
            TuckStatue[self.limb] = False
            if command == "act":
                if placing:
                    //place
                    if got_to_waypoint:
                        grabbing = False
                        self.pub_state.publish(True)
                        if grabAmount >= 90:
                            placing = False
                            got_to_waypoint = False
                    else:
                        self.pub_state.publish(False)
                        pos_msg = {"type": "CartesianPose",
                                   "limb": "left",
                                   "position": [xplace+offsetx+.015, yplace+offsety+.015, zplace+zcalcplace+offsetz],
                                   "rotation": [0, 0, 1, -1, 0, 0, 0, -1, 0],
                                   # "rotation":[1,0,0,0,1,0,0,0,1],
                                   # "rotation":[0,-1,0,1,0,0,0,0,1],   #90 deg rotation about z axis
                                   # "rotation":[1,0,0,0,0,-1,0,1,0],   #90 deg rotation about x axis
                                   # "rotation":[0,0,1,0,1,0,-1,0,0],   #90 deg rotation about y axis
                                   "speed": 1,
                                   "maxJointDeviation": 0.5,
                                   "safe": 0}
                        print(pos_msg)
                        print "picking up cup"
                        print(marker.id_number)
                        dist = np.sqrt(np.square(hand[0]-xplace+offsetx+.015) + np.square(hand[1]-yplace+offsety+.015) + np.square(hand[2]-zplace+zcalcplace+offsetz))
                        if dist < 0.005:
                            got_to_waypoint = True
                        return pos_msg
                else:
                    //pick up cup
                    if got_to_waypoint:
                        self.pub_state.publish(False)
                        grabbing = True
                        if grabAmount >= 90:
                            placing = True
                            got_to_waypoint = False
                    else:
                        self.pub_state.publish(False)
                        marker = state['cup-markers'][pickId]
                        pos_msg = {"type": "CartesianPose",
                                   "limb": "left",
                                   "position": [xpick+offsetx+.015, ypick+offsety+.015, zpick+zcalcpick+offsetz],
                                   "rotation": [0, 0, 1, -1, 0, 0, 0, -1, 0],
                                   # "rotation":[1,0,0,0,1,0,0,0,1],
                                   # "rotation":[0,-1,0,1,0,0,0,0,1],   #90 deg rotation about z axis
                                   # "rotation":[1,0,0,0,0,-1,0,1,0],   #90 deg rotation about x axis
                                   # "rotation":[0,0,1,0,1,0,-1,0,0],   #90 deg rotation about y axis
                                   "speed": 1,
                                   "maxJointDeviation": 0.5,
                                   "safe": 0}
                        print(pos_msg)
                        print "picking up cup"
                        print(marker.id_number)
                        dist = np.sqrt(np.square(hand[0]-xpick+offsetx+.015) + np.square(hand[1]-ypick+offsety+.015) + np.square(hand[2]-zpick+zcalcpick+offsetz))
                        if dist < 0.005:
                            got_to_waypoint = True
                        return pos_msg
                            
        
        if TuckStatus[self.limb]:
            Jointmsg = {}
            Jointmsg['type'] = "JointPose"
            Jointmsg['part'] = self.limb
            Jointmsg['position'] = TuckPose[self.limb]
            Jointmsg['speed'] = 1
            Jointmsg['safe'] = 0
            # TuckStatus[self.limb] = False
            # print Jointmsg
            return Jointmsg
            
                
        #publish grip command to the correct hand
        if (self.limb == 'right'):
            self.pub_r.publish(gripPercent)
        else:
            self.pub_l.publish(gripPercent)
            
        # if TuckStatus[self.limb]:
            # Jointmsg = {}
            # Jointmsg['type'] = "JointPose"
            # Jointmsg['part'] = self.limb
            # Jointmsg['position'] = TuckPose[self.limb]
            # Jointmsg['speed'] = 1
            # Jointmsg['safe'] = 0
            # TuckStatus[self.limb] = False
            # print Jointmsg
            # return Jointmsg

        # tweak = lambda x: 0 if abs(x * self.jointControlRatio) < 0.001 else x * self.jointControlRatio

        # if state['RT'] > 0.1 and lastState['RT'] <= 0.1:
        # return {'type':'Gripper','limb':self.limb,'command':'close'}
        # elif state['RT'] <= 0.1 and lastState['RT'] > 0.1:
        # return {'type':'Gripper','limb':self.limb,'command':'open'}
        
        print "Congratulations for finishing successfully/knocking all the cups over!"
        if pickLimb != "Out" and self.lastPick == "Out":
            self.isAuto = True
            print("here")

        self.lastPick = pickLimb
        # print(self.isAuto)
        global xb, yb, zb
        if self.isAuto:
            # print("IN")
            rospy.Subscriber('get_pos_from_img', Float32MultiArray, callback_img)
            # rospy.Subscriber('Limbz', Float32MultiArray, callback_ik)
            # ik_test('right')
            # print("In Bounding Box")
            # print(pickLimb)
            # print(self.isAuto)
            if pickLimb == "left":
                if xb is not None and yb is not None and zb is not None:
                    print(1)
                    pos_msg = {"type": "CartesianPose",
                               "limb": "left", "position": [xb + 0.07, yb - 0.07, zb + 1.25],
                               "rotation": [0, 0, 1, -1, 0, 0, 0, -1, 0],
                               # "rotation":[1,0,0,0,1,0,0,0,1],
                               # "rotation":[0,-1,0,1,0,0,0,0,1],   #90 deg rotation about z axis
                               # "rotation":[1,0,0,0,0,-1,0,1,0],   #90 deg rotation about x axis
                               # "rotation":[0,0,1,0,1,0,-1,0,0],   #90 deg rotation about y axis
                               "speed": 1,
                               "maxJointDeviation": 0.5,
                               "safe": 0}
                    print(pos_msg)
                    return pos_msg

                '''Jointmsg = {}
                Jointmsg['type'] = "JointPose"
                Jointmsg['part'] = pickLimb
                Jointmsg['position'] = TuckPose[pickLimb]
                Jointmsg['speed'] = 1
                Jointmsg['safe'] = 0
                # TuckStatus[self.limb] = False
                # print Jointmsg
                return Jointmsg'''

            if pickLimb == "right":
                if xb != None and yb != None and zb != None:
                    print(2)
                    pos_msg = {"type": "CartesianPose",
                               # "limb":"right", "position":[xb+0.1,yb-0.15,zb+1.25],
                               "limb": "right", "position": [xb, yb, zb],
                               "rotation": [1, 0, 0, 0, 1, 0, 0, 0, 1],

                               # "limb":"right", "position":[xb+0.1,yb-0.15,zb+1.4], #(stacking)
                               # "rotation": [1,0,0,0,-1,0,0,0,-1], #(stacking)

                               # "rotation":[0,-1,0,1,0,0,0,0,1],   #90 deg rotation about z axis
                               # "rotation":[1,0,0,0,0,-1,0,1,0],   #90 deg rotation about x axis
                               # "rotation":[0,0,1,0,1,0,-1,0,0],   #90 deg rotation about y axis
                               "speed": 1,
                               "maxJointDeviation": 0.5,
                               "safe": 0}
                    print(pos_msg)
                    return pos_msg
                '''Jointmsg = {}
                Jointmsg['type'] = "JointPose"
                Jointmsg['part'] = "right"
                Jointmsg['position'] = TuckPose["right"]
                Jointmsg['speed'] = 1
                Jointmsg['safe'] = 0
                # TuckStatus[self.limb] = False
                # print Jointmsg
                return Jointmsg'''
        # else:
        #    print("Not in BOunding Box")

            if self.controlSet == 'base':
                self.baseCommandVelocity = [-viewToWorldScaleXY * float(lstick[1]) / 5,
                                            -viewToWorldScaleXY * float(lstick[0]) / 5, -float(rstick[0]) / 5]
                return {'type': 'BaseVelocity',
                        'velocity': self.baseCommandVelocity,
                        'safe': 0}

    def check_veloctiy_direction(self, velocity):
        for i in range(3):
            if self.last_driveVel[i] * velocity[i] < 0:
                return True

    def check_manipulability(self):
        pass
        # # return true if near sigularity
        # global M_limit
        # J = self.kin[self.limb].jacobian()
        # J_trans = self.kin[self.limb].jacobian_transpose()
        # # print J
        # M = np.sqrt(np.linalg.det(np.matmul(J, J_trans)))
        # # print M
        # if M < M_limit:
        #     self.near_singularity = True
        #     self.near_singularity = False  # comment it out later on
        #     self.last_sigulatiry = False
        # else:
        #     self.near_singularity = False
        #     self.last_sigulatiry = False

    def getRobotStatus(self):
        T1 = self.serviceThread.eeGetter_left.get()
        if T1 is None:
            T1 = se3.identity()
        R1, t1 = T1
        T2 = self.serviceThread.eeGetter_right.get()
        if T2 is None:
            T2 = se3.identity()
        R2, t2 = T2

        self.baseSensedVelocity = self.serviceThread.baseVelocitySensed.get()
        self.baseCommandVelocity = self.serviceThread.baseVelocityCommand.get()

        self.ArmPosition[0] = self.serviceThread.ArmPosition_left.get()
        self.ArmPosition[1] = self.serviceThread.ArmPosition_right.get()

        self.robotEndEffectorPosition[0] = t1
        self.robotEndEffectorPosition[1] = t2

        self.robotEndEffectorTransform[0] = T2
        self.robotEndEffectorTransform[1] = T2

        self.gripperPosition[0] = self.serviceThread.gripperGetter_left.get()
        self.gripperPosition[1] = self.serviceThread.gripperGetter_right.get()

    def glPlugin(self):
        return self.plugin


class ServiceThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._kill = False
        self.stateUpdateFreq = 50

        self.state_listener = MultiTopicListener(system_state_addr, topics=None, rate=self.stateUpdateFreq)
        self.baseVelocitySensed = self.state_listener.listen('.robot.sensed.base.velocity')
        self.baseVelocityCommand = self.state_listener.listen('.robot.command.base.velocity')
        self.ArmPosition_left = self.state_listener.listen('.robot.sensed.left.q')
        self.ArmPosition_right = self.state_listener.listen('.robot.sensed.right.q')
        self.eeGetter_left = self.state_listener.listen('.robot.endEffectors.0.xform.destination')
        self.eeGetter_right = self.state_listener.listen('.robot.endEffectors.1.xform.destination')
        self.gripperGetter_left = self.state_listener.listen('.robot.gripper.left.positionCommand')
        self.gripperGetter_right = self.state_listener.listen('.robot.gripper.right.positionCommand')
        self.state_listener.setName("GamepadGUIListener")

    def run(self):
        """Note: don't call ServiceThread.run(), call ServiceThread.start()"""
        # self.hapticupdater.run(1)
        while not self._kill:
            # listen to state topics
            asyncore.loop(timeout=1.0 / self.stateUpdateFreq, count=10)

    def kill(self):
        self._kill = True


def callback_pos(data):
    global pick, pickLimb
    pickLimb = data.data
    # if pickLimb != "Out":
    #    playsound(auto_switch_sounds[0])
    # pick=1
    # print(type(pick))


# print(pick.data)
# if pick.data == 1:
# print("This works")

def callback_img(data):
    global xb, yb, zb
    VALUES = data.data
    xb = (VALUES[0] / 1000)  # +0.09
    yb = (VALUES[1] / 1000)  # -0.156
    zb = (VALUES[2] / 1000)  # +0.228
    # print(xb,yb,zb)
    # print(type(pick))


def callback_state(data):
    global marker_state
    marker_state = data.markers
    
def callback_pick(data):
    global pickId
    pickId = data.data
    
def callback_place(data):
    global placeId
    placeId = data.data
    
def callback_command(data):
    global command
    command = data.data
    
def callback_offsets(data):
    global offsetx, offsety, offsetz
    offsetx = data.position.x
    offsety = data.position.y
    offsetz = data.position.z
    
def callback_speed(data):
    #hello
    
def callback_percent(data):
    #percent
    
def callback_gripper(data):
    #gripper state i guess
    
'''
#warning: we have problems where if you subscribe it will stop publishing
def callback_pose(data):
    global left_pose
    left_pose = data
    print data
'''
def midpoint(pos1, pos2, xoff, yoff, zoff):
    newx = (pos1.transform.translation.x + pos2.transform.translation.x)/2 + .015 + xoff
    newy = (pos1.transform.translation.y + pos2.transform.translation.y)/2 + yoff
    newz = (pos1.transform.translation.z + pos2.transform.translation.z)/2 + 1.13 + zoff
    return [newx, newy, newz]


# print(pick.data)
# if pick.data == 1:
# print("This works")
def make():
    return MarkerTaskGenerator()
