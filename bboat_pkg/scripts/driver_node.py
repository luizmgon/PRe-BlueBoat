#!/usr/bin/env python3

# Import required Python code.
import rospy
import sys
import numpy as np

from std_msgs.msg import Bool, Float64

from sensor_msgs.msg import Joy, BatteryState

from mavros_msgs.msg import State, OverrideRCIn, ManualControl

from bboat_pkg.srv import reset_lamb_serv, mode_serv, mode_servResponse, lambert_ref_serv, gain_serv, reset_vsb_serv, gain_servResponse
from bboat_pkg.msg import cmd_msg
from lib.bboat_lib import *

from subprocess import call
import os



Missions = ["VSB", "CAP", "PTN"]
max_fwrd = 1700
min_fwrd = 1300
max_turn = 1700
min_turn = 1300

class DriverNode():
    '''
        Blueboat driver node - Communicates with mavros
        Subscribers
            - Mavros state - Armed Disarmed
            - Marvros Battery - Battery level
            - Joystick node
            - Command topic from controller node
        Publishers
            - Mavros override - Sens forward and turning commands
        Service Clients
            - Reset Lambert Ref to current position client
            - Request current Lambert ref
        Service Providers
            - Mode - AUTO or MANUAL
    '''
    def __init__(self):

        self.rate =rospy.Rate(10) #10Hz

        self.joy_fwrd = 0
        self.joy_turn = 0
        self.joy_cam = 0

        self.mode = "MANUAL" #MANUAL - AUTO
        self.mission_index = 0
        self.auto_mission = Missions[self.mission_index]
        self.count_print = 0
        
        self.auto_cmd = np.zeros((2,1))

        self.battery = 0.0

        self.armed_flag = False
        self.should_be_armed_flag = False

        self.buttons_prev = []

        self.prev_fwrd = 1500
        self.prev_turn = 1500
        self.servo_cam_auto = 1500

        self.flag_rc = False # Tracks RC Chan 2 activity to let RC take over

        self.gains = np.array([[1.5], [0], [0], [0.5], [0], [0]])

        self.index_gain = 0

        # --- Subs
        rospy.Subscriber("/joy", Joy, self.Joy_callback)

        rospy.Subscriber("/mavros/state", State, self.State_callback)
        rospy.wait_for_message("/mavros/state", State, timeout=None)
        rospy.Subscriber("/mavros/battery", BatteryState, self.Battery_callback)

        rospy.Subscriber("/cam_cmd", Float64, self.Camera_cmd_callback)

        self.sub_cmd = rospy.Subscriber('/command', cmd_msg, self.Command_callback)

        # --- Pubs
        self.pub_rc_override = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)


        # --- Services
        rospy.wait_for_service('/reset_lamb_ref')
        connected = False
        while not connected:
            try:
                self.client_reset_lamb = rospy.ServiceProxy('/reset_lamb_ref', reset_lamb_serv)
                connected = True
            except rospy.ServiceException as exc:
                rospy.logwarn(f'[DRIVER] Reset Lambert ref service cannot be reached - {str(exc)}')
                connected = False


        rospy.wait_for_service('/lambert_ref')
        connected = False
        while not connected:
            try:
                self.client_ref_lambert = rospy.ServiceProxy('/lambert_ref', lambert_ref_serv)
                connected = True
            except resoyServiceException as exc:
                rospy.logwarn(f'[DRIVER] Lambert ref service cannot be reached - {str(exc)}')
                connected = False

        self.client_reset_vsb = rospy.ServiceProxy('/reset_vsb', reset_vsb_serv)


        rospy.Service('/mode', mode_serv, self.Mode_Service_callback)


        rospy.Service('/gains', gain_serv, self.Gain_Service_callback)

        # --- Init done
        rospy.loginfo('[DRIVER] Driver Node Start')


    def loop(self): 
        # Main while loop.
        while not rospy.is_shutdown():
            #rospy.loginfo('[DRIVER] Heartbeat')

            # Build and publish override message depending on mode
            rc_msg = OverrideRCIn()
            if self.mode == "MANUAL":
                fwrd = (1500+500*self.joy_fwrd) #int
                turn = (1500-500*self.joy_turn) #int

                fwrd = int(0.15*fwrd + 0.85*self.prev_fwrd)
                turn = int(0.5*turn + 0.5*self.prev_turn)
                self.prev_fwrd = fwrd
                self.prev_turn = turn

                camera = int(1500+500*self.joy_cam)


            elif self.mode == "AUTO":
                fwrd = int(self.auto_cmd[0,0])
                turn = int(self.auto_cmd[1,0])
                if self.auto_mission == "VSB":
                    camera = self.servo_cam_auto
                else:
                    camera = 1500

            # print(f'{fwrd}, {turn}, {camera}')
            # All unused channels set to neutral value seems safe
            rc_msg.channels = [turn,1500,fwrd,1500,1500,1500,camera,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]
            self.pub_rc_override.publish(rc_msg)

            if self.count_print == 50: 
                if self.armed_flag:
                    rospy.loginfo(f'[DRIVER] Status : Armed | Mode : {self.mode} | Mission : {self.auto_mission} | Battery : {round(self.battery, 2)}')
                
                else:
                    rospy.loginfo(f'[DRIVER] Status : Disarmed | Mode : {self.mode} | Mission : {self.auto_mission} | Battery : {round(self.battery, 2)}')

                if self.battery < 13.5:
                    rospy.logwarn('[DRIVER] LOW BATTERY LEVEL')
                self.count_print = 0
            else:
                self.count_print+=1

            self.rate.sleep()


    def Joy_callback(self, msg): 
        '''
            Parse Jaystick message
        '''
        #rospy.loginfo('[DRIVER] Joystick')

        buttons = msg.buttons
        axes = msg.axes
        if not (buttons == self.buttons_prev):
            # Arm - Pause
            if buttons[11]: #7
                rospy.loginfo('[DRIVER] Arming Thrusters')
                #call(["rosrun", "mavros", "mavsafety", "arm"])
                os.system("rosrun mavros mavsafety arm")
                self.should_be_armed_flag = True
            # Disarm - Reset   
            if buttons[10]: #6
                rospy.loginfo('[DRIVER] Disarming Thrusters')
                # call(["rosrun", "mavros", "mavsafety", "disarm"])
                os.system("rosrun mavros mavsafety disarm")
                self.should_be_armed_flag = False

            # Reset Lambert reference - A
            if buttons[0] : 
                rospy.loginfo('[DRIVER] Reset Lambert')
                resp = self.client_reset_lamb(True)

            # Switch mode - B
            if buttons[1]: 
                rospy.loginfo('[DRIVER] Switching mode to: ')    
                if self.mode == "MANUAL":
                    self.mode = "AUTO"
                elif self.mode == "AUTO":
                    self.mode = "MANUAL"
                rospy.loginfo(f'[DRIVER] {self.mode}')

            if buttons[2]: 
                self.client_reset_vsb(True)

            if buttons[5] : 
                self.index_gain = self.index_gain + 1
                rospy.loginfo(f'Gains : index = {self.index_gain} | kp1 = {self.gains[0,0]} | ki1 = {self.gains[1,0]} | kd1 = {self.gains[2,0]} | kp2 = {self.gains[3,0]} | ki2 = {self.gains[4,0]} | kd2 = {self.gains[5,0]}')


                if self.index_gain > 5: 
                    self.index_gain = 0

            if axes[7] == 1 : 
                self.gains[self.index_gain, 0] = self.gains[self.index_gain, 0] + 0.05
                rospy.loginfo(f'Gains : index = {self.index_gain} | kp1 = {self.gains[0,0]} | ki1 = {self.gains[1,0]} | kd1 = {self.gains[2,0]} | kp2 = {self.gains[3,0]} | ki2 = {self.gains[4,0]} | kd2 = {self.gains[5,0]}')

            if axes[7] == -1 : 
                self.gains[self.index_gain, 0] = self.gains[self.index_gain, 0] - 0.05
                rospy.loginfo(f'Gains : index = {self.index_gain} | kp1 = {self.gains[0,0]} | ki1 = {self.gains[1,0]} | kd1 = {self.gains[2,0]} | kp2 = {self.gains[3,0]} | ki2 = {self.gains[4,0]} | kd2 = {self.gains[5,0]}')

            if axes[6] == -1:
                if self.mission_index < len(Missions)-1:
                    self.mission_index += 1
                else:
                    self.mission_index = 0
                self.auto_mission = Missions[self.mission_index]
                rospy.loginfo(f'[DRIVER] Switching Mission to {self.auto_mission}')
            elif axes[6] == 1:
                if self.mission_index > 0:
                    self.mission_index -=1
                else:
                    self.mission_index = len(Missions)-1
                self.auto_mission = Missions[self.mission_index]            
                rospy.loginfo(f'[DRIVER] Switching Mission to {self.auto_mission}')


        # Joystick values between -1 and 1
        self.joy_fwrd = axes[1]
        self.joy_turn = axes[0]

        self.joy_cam = axes[3]

    def State_callback(self, msg):
        '''
            Parse robot state msg - armed disarmed - Raise Warnin if state doesn't match with required state
        '''
        #rospy.loginfo('[DRIVER] State callback')

        self.armed_flag = msg.armed
        # if self.should_be_armed_flag and (self.should_be_armed_flag != self.armed_flag):
        #     rospy.logwarn('[DRIVER] BBoat should be armed but is NOT')
        # elif not self.should_be_armed_flag and (self.should_be_armed_flag != self.armed_flag): 
        #     rospy.logwarn('[DRIVER] BBoat should NOT be armed but is armed')

    def Mode_Service_callback(self, req):
        '''
            Sends mode on request
        '''
        resp = mode_servResponse(self.mode, self.auto_mission)
        return resp

    def Command_callback(self, msg): 
        '''
            Parse command msg
            Turn forward and turning speed to 1100 - 2000 values to override
        '''
        u1, u2 = msg.u1.data, msg.u2.data

        self.auto_cmd = np.array([[ int(1500+(u1/MAX_SPEED_FWRD)*500)], [int(1500+(u2/MAX_SPEED_TURN)*500)]])

    def Battery_callback(self, msg):
        '''
            Parse battery msg
        '''
        self.battery = msg.voltage

    def Camera_cmd_callback(self, msg):
        cam_angle = msg.data #rad

        # print(cam_angle)

        self.servo_cam_auto = int(1500 + (cam_angle/(pi/2))*500)

    def Gain_Service_callback(self, req): 
        resp = gain_servResponse()

        resp.kp_1 = Float64(self.gains[0,0])

        resp.kd_1 = Float64(self.gains[1,0])

        resp.ki_1 = Float64(self.gains[2,0])

        resp.kp_2 = Float64(self.gains[3,0])

        resp.kd_2 = Float64(self.gains[4,0])

        resp.ki_2 = Float64(self.gains[5,0])
        # resp = self.gains
        return resp


# Main function.
if __name__ == '__main__':
    rospy.init_node('driver')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        driver = DriverNode()
        driver.loop()
        os.system("rosrun mavros mavsafety disarm")
    except rospy.ROSInterruptException: pass
