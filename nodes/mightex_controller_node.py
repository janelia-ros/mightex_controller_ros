#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import print_function,division

from mightex_device import MightexDevice

import rospy

from mightex_controller.msg import CmdCurrent,CmdChannel


class MightexController(object):
    def __init__(self,*args,**kwargs):
        rospy.loginfo('Initializing mightex_device_controller...')
        self._initialized = False

        self._cmd_current_sub = rospy.Subscriber('~cmd_current',CmdCurrent,self._cmd_current_callback)
        self._cmd_off_sub = rospy.Subscriber('~cmd_off',CmdChannel,self._cmd_off_callback)

        current_max = rospy.get_param('~current_max')
        self._dev = MightexDevice()
        self._channel_count = self._dev.get_channel_count()
        for channel in range(self._channel_count):
            channel += 1
            self._dev.set_normal_parameters(channel,current_max,1)
        rospy.loginfo('mightex_device_controller initialized!')
        self._initialized = True

    def _cmd_current_callback(self,data):
        if self._initialized:
            channel = data.channel
            current = data.current
            if (channel >= 1) and (channel <= self._channel_count):
                if current > 0:
                    self._dev.set_normal_current(channel,current)
                    self._dev.set_mode_normal(channel)
                else:
                    self._dev.set_mode_disable(channel)

    def _cmd_off_callback(self,data):
        if self._initialized:
            channel = data.channel
            if (channel >= 1) and (channel <= self._channel_count):
                self._dev.set_mode_disable(channel)


if __name__ == '__main__':
    try:
        rospy.init_node('mightex_controller_node')
        mdc = MightexController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
