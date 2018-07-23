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

    # def __init__(self, yaml_file_name='/home/attakorn/projects/'
    #              'snake_ws/src/snake_control/config/modules.yaml'):
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
        # self.default_cmd_params = [0.38*np.pi/2.0, 3.0/8.0, 0.08]
        self.default_cmd_params = [60*np.pi/180,
                                   40*np.pi/180, 0, 0]  # A_o, A_e, C_o, C_e
        self.joints_list = []
        self.t = 0.0

        for i in range(self.num_modules):
            leg_str = 'S_'
            if i < 10:
                leg_str += '0' + str(i)
            else:
                leg_str += str(i)
            self.joints_list += [leg_str]

    def update(self, dt, cmd_params=None):
        """ Publishes snake joint commands for tracking with PID controller
            :param dt: The time elapsed since the last command
            :param cmd_params: A vector of control parameters
        """

#         self.t += dt

#         if cmd_params is None :
#             cmd_params = default_cmd_params

#         A = cmd_params[0]        # the amplitude of the serpenoid equation
#         omega_t = cmd_params[1]  # the temporal frequency of the serpenoid equation
#         omega_s = cmd_params[2]  # the spatial frequency of the serpenoid equation

#         ## sidewinding gait ##
#         d = 1  # direction

#         for i, jnt in enumerate( self.sorted_joints_list ) :
#             self.jnt_cmd_dict[jnt] = A*np.sin( 2.0*np.pi*(d*self.t + (i%2)*omega_t + i*omega_s) )

#         return self.jnt_cmd_dict
        def sidewinding():
            # spatial frequency
            spat_freq = 0.08

            # temporal phase offset between horizontal and vertical waves
            TPO = 3.0/8.0

            # amplitude
            A = 0.38*np.pi/2.0

            # direction
            d = 1

            for i, jnt in enumerate(self.joints_list):
                self.jnt_cmd_dict[jnt] = A * \
                    np.sin(2.0*np.pi*(d*self.t + (i % 2)*TPO + i*spat_freq))

        def slithering(cmd_params):
            N = self.num_modules
            # w = 0.5
            w = cmd_params[4]
            y = 0.3
            z = 0.7
            # A_o = 60 * np.pi / 180
            # A_e = 40 * np.pi / 180
            # C_o = 20 * np.pi / 180
            # C_e = 0
            A_o = cmd_params[0]
            A_e = cmd_params[1]
            C_o = cmd_params[2]
            C_e = cmd_params[3]

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
                    np.sin(Omega * n + o * self.t + delta)

        self.default_cmd_params = [60*np.pi/180,
                                   40*np.pi/180, 0, 0]  # A_o, A_e, C_o, C_e
        self.t += dt
        if cmd_params is None:
            cmd_params = default_cmd_params
        slithering(cmd_params)
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

        rate = rospy.Rate(hz)
        dt = 1.0/float(hz)
        final_time = rospy.Time.now() + duration

        while not rospy.is_shutdown() and rospy.Time.now() < final_time:
            jnt_cmd_dict = self.joint_cmds.update(dt=dt, cmd_params=cmd_params)
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
