#!/usr/bin/env python
"""
The node publishes joint position commands to effort position controllers.
The controllers should already be spawned and running and communicating
with the appropriate robot hardware interface.
"""

import rospy
from std_msgs.msg import Float64

import numpy as np


class JointCmds:
    """
    The class provides a dictionary mapping joints to command values.
    """

    def __init__(self, num_mods):

        self.num_modules = num_mods
        self.jnt_cmd_dict = {}
        self.joints_list = []
        self.t = 0.0

        for i in range(self.num_modules):
            leg_str = 'S_'
            if i < 10:
                leg_str += '0' + str(i)
            else:
                leg_str += str(i)
            self.joints_list += [leg_str]

    def update(self, dt, mode):

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

        def slithering():
            N = self.num_modules
            w = 0.5
            y = 0.3
            z = 0.7
            A_o = 60 * np.pi / 180
            A_e = 40 * np.pi / 180
            C_o = 30 * np.pi / 180
            C_e = 0
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

        self.t += dt
        slithering()
        return self.jnt_cmd_dict


def publish_commands(num_modules, hz):
    cnt=100000
    pub = {}
    ns_str = '/snake'
    cont_str = 'eff_pos_controller'
    for i in range(num_modules):
        leg_str = 'S_'
        if i < 10:
            leg_str += '0' + str(i)
        else:
            leg_str += str(i)
        pub[leg_str] = rospy.Publisher(ns_str + '/' + leg_str + '_'
                                       + cont_str + '/command',
                                       Float64, queue_size=10)
    rospy.init_node('snake_controller', anonymous=True)
    rate = rospy.Rate(hz)
    jntcmds = JointCmds(num_mods=num_modules)
    while not rospy.is_shutdown():
        jnt_cmd_dict = jntcmds.update(1./hz, 0)
        for jnt in jnt_cmd_dict.keys():
            pub[jnt].publish(jnt_cmd_dict[jnt])
        rate.sleep()


if __name__ == "__main__":
    try:
        num_modules = 16
        hz = 100
        publish_commands(num_modules, hz)
    except rospy.ROSInterruptException:
        pass
