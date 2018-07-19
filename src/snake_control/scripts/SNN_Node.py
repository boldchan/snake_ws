#!/usr/bin/env python

import numpy as np
import rospy
from snake_control.msg import snake_head_rel_pos
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from SNN import Two_Layer_SNN
from snake_control.srv import *

class SNN_Node(object):
    def __init__(self):
        self.snn = Two_Layer_SNN(hidden_dim = 10)
        self.snn.W1 = np.loadtxt('Weights1.txt')
        self.snn.W2 = np.loadtxt('Weights2.txt')
        rospy.wait_for_service('/snake/publish_joint_commands')
        self.commands_srv = rospy.ServiceProxy('/snake/publish_joint_commands', PublishJointCmds)
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.pos = []
        self.model = GetModelStateRequest()
        self.model.model_name = 'robot'
        self.model.relative_entity_name = 'target::link'
        
        self.model2 = GetModelStateRequest()#to get coordinate of snake head
        self.model2.model_name='robot'
        self.model2.relative_entity_name = ''
        # self.measured_data = [0, 0]
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
        rel_x = self.pos[0]
        rel_y = self.pos[1]
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

    def callback(self):
        rospy.loginfo("%f,%f"%(self.pos[0], self.pos[1]))
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

    # def listener(self):
    #     rospy.Subscriber("/snake_head_pos", snake_head_rel_pos, self.callback, queue_size = 1)
    #     rospy.loginfo("[SNN, listener]:x_rel:%f, y_rel:%f"%(self.measured_data[0], self.measured_data[1]))

    def query(self):
            result = self.get_model_srv(self.model)
            result2 = self.get_model_srv(self.model2)

            x_rel_world = -result.pose.position.x
            y_rel_world = -result.pose.position.y
            z_rel_world = -result.pose.position.z

            qx = result.pose.orientation.x
            qy = result.pose.orientation.y
            qz = result.pose.orientation.z
            qw = result.pose.orientation.w

            rotation_matrix = np.array([[1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
            [2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz,2*qy*qz-2*qx*qw],
            [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy]])  
            
            pos_rel = np.dot(rotation_matrix.T,np.array([x_rel_world, y_rel_world, z_rel_world]))
            self.pos = [pos_rel[0], pos_rel[1]]


            time = rospy.get_rostime()
            rospy.loginfo("%s, %f, %f"%(time.secs, self.pos[0], self.pos[1]))

    def publish_cmd(self):
        self.commands_srv(100, rospy.Duration(2*np.pi,0),self.control_cmd,False)


if __name__ == "__main__":
    try:
        rospy.init_node('SNN')
        snn_node = SNN_Node()
        rate = rospy.Rate(1/(2*np.pi))
        while not rospy.is_shutdown():
            snn_node.query()
            snn_node.callback()
            snn_node.publish_cmd()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


