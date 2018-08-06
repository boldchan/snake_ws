#!/usr/bin/env python
"""
The node publishes joint position commands to effort position controllers.
The controllers should already be spawned and running and communicating
with the appropriate robot hardware interface.
"""

import rospy
from std_msgs.msg import Float64

import numpy as np

from gazebo_msgs.msg import ModelStates
from snake_control.msg import TargetPosition


class JointCmds:
    """
    The class provides a dictionary mapping joints to command values.
    """

    def __init__(self, num_mods, C_t):

        self.num_modules = num_mods
        self.jnt_cmd_dict = {}
        self.joints_list = []
        self.t = 0.0
        self.C_t = C_t
        for i in range(self.num_modules):
            leg_str = 'S_{:02}'.format(i)
            self.joints_list += [leg_str]

    def update(self, dt):

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
            z = 2.0
            A_o = 60 * np.pi / 180
            A_e = 40 * np.pi / 180
            C_o = self.C_t * np.pi / 180
            C_e = 0
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
        slithering()
        return self.jnt_cmd_dict


class HeadPosition(object):

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

    def callback(self, msg):
        # name: [robot]
        robot = msg.pose[1]
        self.x = robot.position.x
        self.y = robot.position.y
        self.z = robot.position.z


def publish_commands(num_modules, hz, monitor):
    T = 100
    C_t = 0
    pub = {}
    ns_str = '/snake'
    cont_str = 'eff_pos_controller'
    for i in range(num_modules):
        leg_str = 'S_{:02}'.format(i)
        pub[leg_str] = rospy.Publisher(ns_str + '/' + leg_str + '_'
                                       + cont_str + '/command',
                                       Float64, queue_size=10)
    rospy.init_node('snake_controller', anonymous=True)
    rate = rospy.Rate(hz)
    jntcmds = JointCmds(num_mods=num_modules, C_t=C_t)
    with open('position-{}.txt'.format(C_t), 'w') as writer:
        while not rospy.is_shutdown() and jntcmds.t <= T:
            jnt_cmd_dict = jntcmds.update(1./hz)
            for jnt in jnt_cmd_dict.keys():
                pub[jnt].publish(jnt_cmd_dict[jnt])
            writer.write('{} {} {} {}\n'.format(
                jntcmds.t, monitor.x, monitor.y, monitor.z))
            rate.sleep()


if __name__ == "__main__":
    try:
        num_modules = 16
        hz = 100
        monitor = HeadPosition()
        rospy.Subscriber(
            '/gazebo/model_states',
            ModelStates,
            monitor.callback,
            queue_size=1
        )
        publish_commands(num_modules, hz, monitor)
    except rospy.ROSInterruptException:
        pass
