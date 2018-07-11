#!/usr/bin/env python

import numpy as np
import rospy
from snake_control.msg import snake_head_rel_pos
from SNN import Two_Layer_SNN
from snake_control.srv import *

class SNN_Node(object):
    def __init__(self):
        self.snn = Two_Layer_SNN(hidden_dim = 10)
        rospy.wait_for_service('/snake/publish_joint_commands')
        self.commands_srv = rospy.ServiceProxy('/snake/publish_joint_commands', PublishJointCmds)
        self.measured_data = [0, 0]
        self.control_cmd = [60 * np.pi /180, 40 * np.pi / 180, 0, 0, 0.5]
        self.dataset = {}
    

    def preprocess(self):
        '''
        output
        --------------------
        {'input':input, 'output':output}

        input[0]: y_pos
        input[1]: x_neg
        input[2]: y_neg

        output[0]: alphaL
        output[1]: alphaR
        '''
        rel_x = self.measured_data[0]
        rel_y = self.measured_data[1]
        dist = np.sqrt(rel_x * rel_x + rel_y * rel_y)
        input = [0, 0, 0]
        if rel_y > 0:
            input[0] = rel_y
        else:
            input[2] = -rel_y
        if rel_x < 0:
            input[1] = -rel_x

        if rel_x < 0 or (rel_x > 0 and rel_x < 1.7 * abs(rel_y)):
            if rel_y > 0:
                alphaR = 30
                alphaL = -180
            else:
                alphaR = 180
                alphaL = -30
        else:
            val = abs(rel_y) / abs(rel_x)
            alpha = np.arctan(val) * 180 / np.pi
            if rel_y > 0:
                alphaR = alpha
                alphaL = -180
            else:
                alphaR = 180
                alphaL = -alpha
        output = [alphaL, alphaR]
        self.dataset = {'input':input, 'output':output}

    def callback(self, data):
        self.measured_data = [data.x_rel, data.y_rel]
        rospy.loginfo("%f,%f"%(data.x_rel, data.y_rel))
        self.preprocess()
        alphaL, alphaR = self.snn.simulate(self.dataset, 10)
        if abs(alphaL) < abs(alphaR):
            if abs(alphaL) > 30:
                C_o = -30
            else:
                C_o = alphaL
        else:
            if alphaR > 30:
                C_o = 30
            else:
                C_o = alphaR
        self.control_cmd = [60 * np.pi /180, 40 * np.pi / 180, C_o *np.pi / 180, 0, 0.5]

    def listener(self):
        rospy.Subscriber("/snake_head_pos", snake_head_rel_pos, self.callback)
        rospy.loginfo("[SNN, listener]:x_rel:%f, y_rel:%f"%(self.measured_data[0], self.measured_data[1]))

    def publish_cmd(self):
        self.commands_srv(100, rospy.Duration(2*np.pi,0),self.control_cmd,False)


if __name__ == "__main__":
    try:
        rospy.init_node('SNN')
        snn_node = SNN_Node()
        rate = rospy.Rate(1/(2*np.pi))
        while not rospy.is_shutdown():
            snn_node.listener()
            snn_node.publish_cmd()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


