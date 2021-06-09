#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8, Bool, Float32
import numpy as np

sampling_freq = 50 #Hz
time_period = 1.0 / float(sampling_freq)    # s

time_duration = 5  # s

N = time_duration * sampling_freq
M = 30  # No of experiments

time_array = np.zeros(N, dtype=float)
target_ang_vel_readings = np.zeros(N, dtype=float)
ang_vel_readings = np.zeros((M,N), dtype=float)

curr_ang_vel = 0
curr_target = 0
t0 = 0

def angular_velocity_callback(data) :

    global curr_ang_vel

    curr_ang_vel = data.data

    pass

def get_normalised(value) :

    if value >= 0 and value <= 30 :

        return value
    
    else : return np.nan

Target_Ang_Vel_Publisher = rospy.Publisher('/U1/L/target_ang_vel', Float32, queue_size=10)
PWM_Publisher = rospy.Publisher('/U1/L/P', UInt8, queue_size=10)

rospy.init_node('Motor_Step_Response', anonymous=True)

rate = rospy.Rate(sampling_freq)

Target_Ang_Vel_Publisher.publish(0)

rospy.Subscriber('/U1/L/current_ang_vel', Float32, angular_velocity_callback)

rospy.loginfo('\tInitialized node')

for j in range(M) :

    rospy.loginfo('\tPerforming experiment no : ' + str(j+1))
    
    i = 0

    t0 = rospy.get_time()
    
    time_array[i] = 0

    target_ang_vel_readings[i] = curr_target

    ang_vel_readings[j, i] = curr_ang_vel

    i += 1

    rate.sleep()

    while i < N :

        # rospy.loginfo('\tInside loop, Iteration # ' + str(i))

        if i * time_period < 2.0 :

            curr_target = 0
            Target_Ang_Vel_Publisher.publish(curr_target)

        else:

            curr_target = 20
            Target_Ang_Vel_Publisher.publish(curr_target)

        time_array[i] = rospy.get_time() - t0

        target_ang_vel_readings[i] = curr_target

        ang_vel_readings[j, i] = get_normalised(curr_ang_vel)

        i += 1

        rate.sleep()

    curr_target = 0
    Target_Ang_Vel_Publisher.publish(curr_target)

    rospy.sleep(5.0)

rospy.loginfo('\tSaving Data')

Target_Ang_Vel_Publisher.publish(0)

rospy.loginfo('\tMax Standard Deviation : ' + str(np.max(np.nanstd(ang_vel_readings, axis=0))))

data = np.column_stack((time_array, target_ang_vel_readings, np.nanmean(ang_vel_readings, axis=0)))
np.savetxt('Motor_Step_Response.csv', data, delimiter=',', header='Time (s), Target Angular Velocity, Angular Speed')