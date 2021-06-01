#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8, Bool, Float32

import numpy as np

N = 20

ang_vel_readings = np.zeros(N, dtype=float)

def angular_velocity_callback(data) :

    global ang_vel_readings

    ang_vel_readings[1:] = ang_vel_readings[:-1]
    ang_vel_readings[0] = data.data

    pass

PWM_Publisher = rospy.Publisher('/U1/L/P', UInt8, queue_size=10)

rospy.init_node('Motor_PWM_Response', anonymous=True)

rospy.Subscriber('/U1/L/current_ang_vel', Float32, angular_velocity_callback)

pwm_arr = np.arange(0, 256, dtype=float)
ang_vel_array = np.zeros_like(pwm_arr, dtype=float)

rate = rospy.Rate(0.5)

for i, pwm in enumerate(pwm_arr) :

    PWM_Publisher.publish(int(pwm))

    rospy.loginfo('\tPWM : ' + str(pwm))

    while not rospy.is_shutdown() :

        rate.sleep()

        # print(ang_vel_readings)
        print(np.mean(ang_vel_readings), np.std(ang_vel_readings))
        
        if np.std(ang_vel_readings) <= 1.5 :

            ang_vel_array[i] = np.mean(ang_vel_readings)
            break

PWM_Publisher.publish(0)
rospy.loginfo('\tSaving Data')

data = np.column_stack((pwm_arr, ang_vel_array))
np.savetxt('Motor_PWM_Response.csv', data, delimiter=',', header='PWM,Angular Speed')
    




