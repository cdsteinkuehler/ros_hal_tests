#!/usr/bin/env python

# create or attach to a ring
# write a sequence of increasing doubles

import os,time,sys
from machinekit import hal
import struct
name = "jointpos"

try:
    # to create the ring
    r = hal.Ring(name, size=4096)
except RuntimeError:
    # oops, existed already, so just attach
    r = hal.Ring(name)


# Message details:
msg_fmt  = 'd'
msg_size = struct.calcsize(msg_fmt)

v = 0.0
while True:
    if r.available >= msg_size:
        r.write(struct.pack('d', v))
        v += 1.0
    time.sleep(1)
