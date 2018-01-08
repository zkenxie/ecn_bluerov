#!/usr/bin/env python

import navio2.pwm
import navio2.util
import rospy
import numpy as np
from scipy.interpolate import interp1d
from sensor_msgs.msg import JointState
import time
from os.path import exists

# Thruster force mapping (in N)
map_force = np.array([-30.,0.,30.])
map_pwm = np.array([1.100,1.500,1.900])

class Listener:
    def __init__(self):
        self.thr = [0.,0.,0.,0.,0.,0.]
        self.sub = rospy.Subscriber('thruster_command', JointState, self.callback)
        
    def callback(self, msg):
        for i in xrange(6):
            try:
                # look for corresponding thruster name and copies in this
                idx = msg.name.index('thr{}'.format(i+1))
                print 'Found thr{}'.format(i+1)

                if msg.effort[idx] >= map_force[0] and msg.effort[idx] <= map_force[-1]:
                    self.thr[i] = msg.effort[idx]
            except:
                pass

# wait for pwm to be online
while not exists('/sys/class/pwm/pwmchip0'):
    time.sleep(1)
            
            
rospy.init_node('thruster_node')
      
      
navio2.util.check_apm()
PWM_OUTPUTs = [1, 3, 5, 7, 9, 11]
PWM_OUTPUTs = [v-1 for v in PWM_OUTPUTs]

T = 1./20

# enable all
for PWM_OUTPUT in PWM_OUTPUTs:
    with navio2.pwm.PWM(PWM_OUTPUT) as pwm:
        print("Arming pwm #{}".format(PWM_OUTPUT))    
        pwm.set_period(50)
        pwm.enable()
        pwm.set_duty_cycle(1.500)
    time.sleep(.1)


# applies subscribed forces
pwm_interp = interp1d(map_force, map_pwm, kind='linear')

listener = Listener()

while not rospy.is_shutdown():
    
    for idx,PWM_OUTPUT in enumerate(PWM_OUTPUTs):
        with navio2.pwm.PWM(PWM_OUTPUT) as pwm:
            v = pwm_interp(listener.thr[idx])
            if listener.thr[idx] != 0:
                print('Thr {}, {} N = {} PWM'.format(idx+1, listener.thr[idx],v))
            pwm.set_duty_cycle(v)
            
    rospy.sleep(T)
    
        
