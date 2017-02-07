import sys, time
import numpy as np
import random
import Quaternion
import copy

import rospy
from math import sin, cos, pi
import visualization_msgs
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

def talker():
    dhDict = readRobotJson("robot_test.json")
    bot = MyRobot(dhDict)

    print( bot.numLinks )

    markerPub = rospy.Publisher("visualization_marker", Marker, queue_size = 10)
    markerArrPub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size = 10)
    rospy.init_node('basic_shapes')
    rate = rospy.Rate(30)
    markerArr = MarkerArray()
    while not rospy.is_shutdown():

        #change q randomly
        bot.q += (1.5*np.random.rand(bot.numLinks, 1) - np.random.rand(bot.numLinks, 1) ) * 0.01

        #markers
        #clear markerArray
        while len(markerArr.markers) > 0:
            markerArr.markers.pop(0)

        marker = getMarker(bot, -1)
        markerArr.markers.append(marker)
        for i in range(bot.numLinks):
            marker = getMarker( bot, i )
            markerArr.markers.append(marker)
        marker = getLineStrip(bot)

        while markerPub.get_num_connections() < 1 :
            if rospy.is_shutdown():
                return 0
            time.sleep(1)
        markerPub.publish(marker)
        markerArrPub.publish(markerArr)
        rate.sleep()

def getColor(i):
    colors = {}
    colors[-1] =(1.0, 0.0, 0.0)
    colors[0] = (0.8, 0.0, 0.2)
    colors[1] = (0.6, 0.0, 0.4)
    colors[2] = (0.4, 0.0, 0.6)
    colors[3] = (0.2, 0.0, 0.8)
    colors[4] = (0.0, 0.2, 1.0)
    colors[5] = (0.0, 0.4, 1.0)
    colors[6] = (0.0, 0.6, 1.0)
    colors[7] = (0.0, 0.8, 1.0)


    return colors[i]

def getMarker(bot, index):
    marker = Marker()
    marker.header.frame_id = "/base_link"
    marker.header.stamp = rospy.Time.now()

    ns = str(index)
    marker.ns = "basic_shapes{}".format(ns)
    marker.id = 0

    marker.type = Marker.CUBE
    marker.action = Marker.ADD


    posF = bot.getTranslation(0,index+1)
    rotF = bot.getRotationMatrix(0,index+1)

    marker.pose.position.x = posF[0]
    marker.pose.position.y = posF[1]
    marker.pose.position.z = posF[2]

    quat = Quaternion.Quat(rotF).q

    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]

    # scaling = bot.dhParams[index][2]/10.0
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02

    rgb = getColor(index)
    marker.color.r = rgb[0]
    marker.color.g = rgb[1]
    marker.color.b = rgb[2]
    marker.color.a = 1.0

    marker.lifetime = rospy.Duration()

    return marker

def getLineStrip(bot):
    marker = Marker()
    marker.header.frame_id = "/base_link"
    marker.header.stamp = rospy.Time.now()

    marker.ns = "linestrip1"
    marker.id = 0

    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.01

    rgb = [1.0,0.8,0]
    marker.color.r = rgb[0]
    marker.color.g = rgb[1]
    marker.color.b = rgb[2]
    marker.color.a = 1.0

    p = Point( 0, 0, 0)
    marker.points.append(p)

    for index in xrange(bot.numLinks):
        posF = bot.getTranslation(0,index+1) # scale for drawing sanity.
        p = Point(*posF)
        marker.points.append(p)

    marker.lifetime = rospy.Duration()

    return marker

class MyRobot:

    def __init__(self, DHparams):
        # list of 4-tuples (a, alpha, d, theta)
        if type(DHparams) == dict:
            self.setDHFromDict(DHparams)
        elif type(DHparams) == list:
            self.dhParams = DHparams
        else:
            throw("Invalid Format")

        print self.dhParams

        self.numLinks = len(self.dhParams)
        self.q = np.zeros((self.numLinks, 1))

    def setDHFromDict(self, DHdict):
        self.dhParams = []
        for joint in DHdict["Joints"]:
            self.dhParams.append(joint["DH_parameters"])

    def getTranslation(self, i,j):
        return self.getT(i,j)[:-1,-1]

    def getRotationMatrix(self, i,j):
        return self.getT(i,j)[:3,:3]

    # i is the index of the starting coordinate frame
    # j is the index of the ending coordinate frame
    def getT(self, i, j):
        T = self.constructA(i) # Transform from i to i+1
        for itter in range(i+1,j):
            T = np.dot( T, self.constructA(itter) )
        return T

    # returns the transform to the i+1th from the ith
    def constructA(self, i):
        a, alpha, d ,theta = self.dhParams[i]

        #to add prismatic joints the following line would need to be changed.
        theta += self.q[i]
        row1 = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta) ]
        row2 = [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta) ]
        row3 = [       0.0,             sin(alpha),             cos(alpha),            d ]
        row4 = [       0.0,                    0.0,                    0.0,          1.0 ]
        return np.asarray([row1, row2, row3, row4])

def readRobotJson(filename):
    f = open(filename)
    botSpecs = eval(f.read())
    f.close()
    return botSpecs

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
