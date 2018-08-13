#!/usr/bin/env python

import rospy

altitude0, altitude1 = 0

def callback(msg):
    altitude1 = altitude0
    altitude0 = msg

def main():
    threshold = float(rospy.get_param("~threshold")
    rospy.init_node('altitude_listener', anonymous=True)
    r = rospy.Rate(20)
    rospy.Subscriber("/laserscan", Float32, callback, 1)
    pub = rospy.Publisher("/altitude_tracker", queue_size=10)
    while not rospy.is_shutdown():
        if(abs(altitude1 - altitude0) < threshold)):
            pub.publish(altitude1); # There is no obstacle between drone and ground
        else:
            while not rospy.is_shutdown():
                if(abs(altitude1 - altitude0 < threshold):
                    pub.publish(0); # Obstacle found. Do not remove points under obstacle
                else:
                    break

if __name__ == '__main__':
    main()
