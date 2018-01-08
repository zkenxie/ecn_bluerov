#!/usr/bin/env python

from std_msgs.msg import Float32
import rospy
from numpy import sin, pi


rospy.init_node('pid')

pub = rospy.Publisher('/gna', Float32, queue_size=1)

msg = Float32()
dt = 0.05
f = .5
t = 0


while not rospy.is_shutdown():
    
    msg.data = sin(2*pi*f*t)
    t += dt
    
    pub.publish(msg)
        
    rospy.sleep(dt)
