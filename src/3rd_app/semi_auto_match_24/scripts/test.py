#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy


def main() :
    rospy.init_node('test')
    rate = rospy.Rate(30)
    counter = 0
    while not rospy.is_shutdown():
        print(counter)
        counter += 1
    rate.sleep()

if __name__ == "__main__" :
    try :
        main()
    except rospy.ROSInterruptException as e:
        rospy.INFO(e)