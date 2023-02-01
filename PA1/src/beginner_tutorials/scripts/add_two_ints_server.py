#!/usr/bin/env python

from __future__ import print_function

from beginner_tutorials.srv import AddTwoInts,MultiplyTwoFloats,AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    print("Returning [%s * %s = %s]"%(req.a, req.b, (req.a * req.b)))
    #return AddTwoIntsResponse(req.a + req.b)
    return float(req.a * req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', MultiplyTwoFloats, handle_add_two_ints)
    print("Ready to multiply two floats.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
