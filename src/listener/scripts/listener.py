#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

import os, time
from machinekit import hal
import struct

# print ring properties
def print_ring(r):
    print "name=%s size=%d reader=%d writer=%d scratchpad=%d" % (r.name,r.size,r.reader,r.writer,r.scratchpad_size),
    print "use_rmutex=%d use_wmutex=%d type=%d in_halmem=%d" % (r.rmutex_mode, r.wmutex_mode,r.type,r.in_halmem)

# retrieve list of ring names
rings = hal.rings()
print "rings: ", rings

# Message details:
msg_fmt  = '6d'
msg_size = struct.calcsize(msg_fmt)

# Global variable for ring handle
w = 0


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print 'In callback'
    for pose in data.points:
        # print pose.positions[0]
        print ["{0:0.2e}".format(i) for i in pose.positions]

        if w.available > msg_size:
            msg = struct.pack(msg_fmt, pose.positions[0],
                                       pose.positions[1],
                                       pose.positions[2],
                                       pose.positions[3],
                                       pose.positions[4],
                                       pose.positions[5] )
            w.write(msg)


    # investigate scratchpad region if one is defined
    if w.scratchpad_size:
        print "scratchpad:%d = '%s'" % (w.scratchpad_size, w.scratchpad.tobytes())


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    if "jointpos" in rings:

        # attach to existing ring
        global w
        w = hal.Ring("jointpos")

        # see what we have
        print_ring(w)

    rospy.Subscriber("/joint_path_command", JointTrajectory, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print 'HelloThere'
    listener()
