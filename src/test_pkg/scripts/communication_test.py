#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import rospy
from std_msgs.msg import String, Int32, Float32MultiArray, Bool


rospy.init_node('communication_test', anonymous=True)




def test_pub():
    send_params = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6] # float array
    send_steps = 5030 # int

    def check_params(data):
        # assert data == send_params, "topic parameters: {} received, {} expected".format(data, send_params)
        # print("topic params working properly")
        if data:
            print("topic parameters: OK")
        else:
            print("topic parameters: received data in remote is wrong")

    def check_steps(data):
        # assert data = send_steps, "topic start: {} received, {} expected".format(data, send_steps)
        # print("topic steps working properly")
        if data:
            print("topic start: OK")
        else:
            print("topic start: received data in remote is wrong")
        
    # testing multiple publisher
    pub1 = rospy.Publisher('parameters', Float32MultiArray, queue_size=100)
    pub2 = rospy.Publisher('start', Int32, queue_size=100)

    # receive echoback
    rospy.Subscriber('parameters_reply', Bool, check_params)
    rospy.Subscriber('start_reply', Bool, check_steps)
    
    pub1.publish(Float32MultiArray(data=send_params))
    pub2.publish(Int32(data=send_steps))

    print("waiting for echo back")
    rospy.spin() # wait for a mbed to echo back


    
def test_sub():
    expected_data = "finish running 5030 steps"
    def check_subscriber(data):
        assert data == expected_data, "topic end: received {}, expected {}".format(data, expected_data)
        print("string subscription OK")

    # testing subscriber
    rospy.Subscriber('end', )

    pub = rospy.Publisher('end', )



if __name__ == '__main__':
    if len(sys.argv) == 1 or sys.argv[1] == 'pub':
        test_pub()
    else:
        test_sub()

