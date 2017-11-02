#!/usr/bin/python

import sys
import time

import navio2.pwm
import navio2.util

navio2.util.check_apm()

PWM_OUTPUTs = [2,1,3,5,8,10]
#PWMs = [navio2.pwm.PWM(PWM_OUTPUT PWM_OUTPUT in PWM_OUTPUTs]

enabled = False

while True:
    for PWM_OUTPUT in PWM_OUTPUTs:
        with navio2.pwm.PWM(PWM_OUTPUT) as pwm:
            if not enabled:
                enabled = True
                pwm.set_period(50)
                pwm.enable()
            pwm.set_duty_cycle(1.500)
    time.sleep(1)
        
