#!/usr/bin/env python
"""a server that publishes parameterized joint commands on request.
"""

import os
import rospy
from std_msgs.msg import Float64
from snake_control.srv import *
import numpy as np


class JointCmds:
    """ The class provides a dictionary mapping joints to command values.
    """

    def __init__(self, yaml_file_name=os.path.abspath(
            'src/snake_control/config/modules.yaml')):

        import yaml

        yaml_file = file(yaml_file_name)
        yaml_data = yaml.load(yaml_file)
        yaml_file.close()

        self.modules_dict = yaml_data['module_map']
        self.num_modules = len(self.modules_dict)
        self.sorted_joints_list = sorted(self.modules_dict.keys())
        self.jnt_cmd_dict = {}
        self.joints_list = []
        self.t = 0.0

        for i in range(self.num_modules):
            leg_str = 'S_{:02}'.format(i)
            self.joints_list += [leg_str]

    def update(self, dt,
               A_o=60 * np.pi / 180,
               A_e=40 * np.pi / 180,
               C_o=0,
               C_e=0,
               w=0.5):
        """ Publishes snake joint commands for tracking with PID controller
            :param dt: The time elapsed since the last command
            :param cmd_params: A vector of control parameters
        """

        def sidewinding():
            spat_freq = 0.08
            TPO = 3.0/8.0
            A = 0.38*np.pi/2.0
            d = 1

            for i, jnt in enumerate(self.joints_list):
                self.jnt_cmd_dict[jnt] = A * \
                    np.sin(2.0*np.pi*(d*self.t + (i % 2)*TPO + i*spat_freq))

        def slithering(A_o, A_e, C_o, C_e, w):
            N = self.num_modules
            y = 0.3
            z = 1.0
            speed = 2.2
            for n, jnt in enumerate(self.joints_list):
                if n % 2 == 1:
                    x = 1.75
                    o = 1
                    A = A_o
                    delta = 0
                    C = C_o
                else:
                    x = 3.5
                    o = 2
                    A = A_e
                    delta = np.pi / 2
                    C = C_e
                Omega = (w + x * 2 / N) * np.pi
                P = z * n / N + y

                self.jnt_cmd_dict[jnt] = C + P * A * \
                    np.sin(Omega * n + o * speed * self.t + delta)

        self.t += dt
        slithering(A_o, A_e, C_o, C_e, w)
        return self.jnt_cmd_dict

    def zero(self):
        """ Zero the snake joints and reset time
        """

        self.t = 0.0

        for jnt in self.modules_dict.keys():
            self.jnt_cmd_dict[jnt] = 0.0

        return self.jnt_cmd_dict


class JointCmdSrvr:
    """ a server that publishes parameterized joint commands on request
    """

    def __init__(self):

        rospy.init_node('joint_cmd_srvr')

        self.pub = {}
        ns_str = '/snake'
        cont_str = 'eff_pos_controller'
        self.joint_cmds = JointCmds()

        for jnt in self.joint_cmds.modules_dict.keys():
            self.pub[jnt] = rospy.Publisher(ns_str + '/' + jnt + '_'
                                            + cont_str + '/command',
                                            Float64, queue_size=10)

        s = rospy.Service('/snake/publish_joint_commands',
                          PublishJointCmds, self.publish_joint_commands)

    def publish_joint_commands(self, req):
        """ Server callback publishes joint commands for tracking with PID controller
        """

        hz = req.rate
        duration = req.T
        cmd_params = req.params
        reset = req.reset

        rospy.loginfo(cmd_params)

        rate = rospy.Rate(hz)
        dt = 1.0/float(hz)
        final_time = rospy.Time.now() + duration

        while not rospy.is_shutdown() and rospy.Time.now() < final_time:
            jnt_cmd_dict = self.joint_cmds.update(dt, *cmd_params)
            for jnt in jnt_cmd_dict.keys():
                self.pub[jnt].publish(jnt_cmd_dict[jnt])
            rate.sleep()

        stamp = rospy.Time.now()  # the time of the final command message
        # print "dt = ", stamp.to_sec() - (final_time - duration).to_sec(), "  its = ", its

        if (reset):
            jnt_cmd_dict = self.joint_cmds.zero()
            for jnt in jnt_cmd_dict.keys():
                self.pub[jnt].publish(jnt_cmd_dict[jnt])

        reply = PublishJointCmdsResponse()
        reply.header.stamp = stamp
        reply.success = True

        return reply



# run the server
if __name__ == "__main__":
    jnt_cmd_srvr = JointCmdSrvr()
    print "joint_cmd_srvr started, waiting for queries"
    rospy.spin()
