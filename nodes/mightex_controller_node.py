#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import print_function,division

from mightex_device import MightexDevice

from serial_device2 import ReadError

import rospy

from mightex_controller.msg import CmdCurrent,CmdChannel

from std_msgs.msg import Empty


class MightexController(object):
    def __init__(self,*args,**kwargs):
        rospy.loginfo('Initializing mightex_controller_node...')
        self._initialized = False

        self._cmd_current_sub = rospy.Subscriber('~cmd_current',CmdCurrent,self._cmd_current_callback)
        self._cmd_off_sub = rospy.Subscriber('~cmd_off',CmdChannel,self._cmd_off_callback)
        self._cmd_all_off_sub = rospy.Subscriber('~cmd_all_off',Empty,self._cmd_all_off_callback)

        serial_port = rospy.get_param('~serial_port',None)
        if (serial_port is not None) and (len(serial_port) > 0):
            self._dev = MightexDevice(port=serial_port)
        else:
            self._dev = MightexDevice()
        rospy.loginfo('mightex_controller_node initialized!')
        self._setup = False
        self._setup_device()
        self._initialized = True

    def _setup_device(self):
        setup_attempts_max = 10
        setup_attempt = 0
        while (not self._setup) and (setup_attempt < setup_attempts_max):
            setup_attempt += 1
            try:
                self._channel_count = self._dev.get_channel_count()
                self._enabled = []
                for channel in range(self._channel_count):
                    self._enabled.append(False)
                if self._channel_count == 0:
                    rospy.loginfo('mightex_controller found no device channels!')
                else:
                    rospy.loginfo('mightex_controller found {0} device channels!'.format(self._channel_count))
                    self._setup = True
                current_max = rospy.get_param('~current_max')
                for channel in range(self._channel_count):
                    self._dev.set_normal_parameters(channel,current_max,1)
            except ReadError:
                pass

    def _cmd_current_callback(self,data):
        if self._initialized:
            # rospy.loginfo('mightex_controller channel: {0}, current {1}'.format(data.channel,data.current))
            if not self._setup:
                self._setup_device()
            channel = data.channel
            current = data.current
            if (channel >= 0) and (channel < self._channel_count):
                if current > 0:
                    self._dev.set_normal_current(channel,current)
                    if not self._enabled[channel]:
                        self._dev.set_mode_normal(channel)
                        self._enabled[channel] = True
                else:
                    self._dev.set_mode_disable(channel)
                    self._enabled[channel] = False

    def _cmd_off_callback(self,data):
        if self._initialized:
            # rospy.loginfo('mightex_controller cmd_off')
            if not self._setup:
                self._setup_device()
            channel = data.channel
            if (channel >= 0) and (channel < self._channel_count):
                self._dev.set_mode_disable(channel)
                self._enabled[channel] = False

    def _cmd_all_off_callback(self,data):
        if self._initialized:
            # rospy.loginfo('mightex_controller cmd_all_off')
            if not self._setup:
                self._setup_device()
            self.all_off()

    def all_off(self):
        if self._initialized:
            if not self._setup:
                self._setup_device()
            for channel in range(self._channel_count):
                self._dev.set_mode_disable(channel)
                self._enabled[channel] = False


if __name__ == '__main__':
    rospy.init_node('mightex_controller_node')
    mc = MightexController()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        mc.all_off()
