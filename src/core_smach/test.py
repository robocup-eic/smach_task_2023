import rospy
rospy.init_node('hi',anonymous=True)
rate = rospy.Rate(5)

timeout = 10 #seconds
timeout_count = 0
while not rospy.is_shutdown():
    if timeout_count < 10/(1/5):
        print('STILL GOT TIME')
    else:
        print('TIMES UP')
        break
    timeout_count+=1
    rate.sleep()