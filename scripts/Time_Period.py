import numpy as np
import rospy
from std_msgs.msg import UInt8, Bool, Float32, Int16MultiArray

N = 10 #No. of cycles

read_time_readings = np.zeros(N, dtype = float) 
vel_cal_time_readings = np.zeros(N, dtype = float) 
PWM_time_readings = np.zeros(N, dtype = float) 

encoder_index = 0
vel_cal_index = 0
PWM_index = 0

def encoder_callback(data):
    global encoder_index, read_time_readings
    now = rospy.get_rostime()
    if encoder_index<N:
        read_time_readings[encoder_index] = now.secs
        print('Encoder callback called')
    encoder_index+=1
    pass

def curr_ang_vel_callback(data):
    global vel_cal_index, encoder_index, vel_cal_time_readings
    if encoder_index!=0:
        if vel_cal_index<encoder_index and vel_cal_index<N:
            now = rospy.get_rostime()
            vel_cal_time_readings[vel_cal_index] = now.secs
            print('Current Ang Vel callback called')
            vel_cal_index+=1
    pass
    

def PWM_callback(data):
    global vel_cal_index, PWM_index, PWM_time_readings
    if vel_cal_index!=0:
        if PWM_index<vel_cal_index:
            now = rospy.get_rostime()
            PWM_time_readings[PWM_index] = now.secs
            print('PWM callback called')
            PWM_index+=1
    pass

rospy.init_node('Time_Period', anonymous=True)

rospy.Subscriber('/U1/L/current_ang_vel', Float32, curr_ang_vel_callback)
rospy.Subscriber('/U1/L/E', Int16MultiArray, encoder_callback)
rospy.Subscriber('/U1/L/P', UInt8, PWM_callback)


while PWM_index<N and not rospy.is_shutdown():
    pass

print('Read time mean: ', np.nanmean(read_time_readings[1:]-read_time_readings[:-1]))
print('Read time std dev: ', np.std(read_time_readings[1:]-read_time_readings[:-1]))
print('Velocity calculation time mean: ', np.nanmean(vel_cal_time_readings-read_time_readings))
print('Velocity calculation time std dev: ', np.std(vel_cal_time_readings-read_time_readings))
print('PWM time mean: ', np.nanmean(PWM_time_readings-vel_cal_time_readings))
print('PWM time std dev: ', np.std(PWM_time_readings-vel_cal_time_readings))