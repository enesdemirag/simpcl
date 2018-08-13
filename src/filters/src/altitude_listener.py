#!/usr/bin/env python

import rospy

altitude0, altitude1 = 0
threshold = 0.3 # 0.3 meter
def callback(msg):
    altitude1 = altitude0
    altitude0 = msg

def main():
    rospy.init_node('altitude_listener', anonymous=True)
    r = rospy.Rate(20)
    rospy.Subscriber("/laserscan", Float32, callback, 1)
    pub = rospy.Publisher("/altitude_tracker", queue_size=10)
    while(True):
        if((altitude1 - altitude0) < threshold):
            pub.publish(altitude1) # There is no obstacle between drone and ground
        else:
            pub.publish(0) # Obstacle found. Do not remove points under obstacle
        r.sleep()
        rospy.spin()

if __name__ == '__main__':
    main()
