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
    n = 10000
    dt = 1/100.0
    # circularPath
    Ve = [ [5.0*sin(2.0*pi*t/1000.0) , 5.0*cos(2.0*pi*t/1000.0), 0.0, 0.0, 0.0, 0.0] for t in range(n)]

    # print( bot.numLinks )

    markerPub = rospy.Publisher("visualization_marker", Marker, queue_size = 10)
    markerArrPub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size = 10)
    rospy.init_node('basic_shapes')
    rate = rospy.Rate(1000)
    q = np.random.rand(7,1)/5.0
    bot.q = q

    markerArr = MarkerArray()
    t = 0
    while not rospy.is_shutdown():
        #change q
        qDot = bot.getQdotFromVe(Ve, dt)
        print(np.round(Ve[t],3))
        bot.q = [ dt*qDot[t][i] + bot.q[i] for i in range(bot.numLinks) ]
        t = (t+1) % n

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
    marker.header.frame_id = "/my_frame"
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
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    rgb = getColor(index)
    marker.color.r = rgb[0]
    marker.color.g = rgb[1]
    marker.color.b = rgb[2]
    marker.color.a = 1.0

    marker.lifetime = rospy.Duration()

    return marker

def getLineStrip(bot):
    marker = Marker()
    marker.header.frame_id = "/my_frame"
    marker.header.stamp = rospy.Time.now()

    marker.ns = "linestrip1"
    marker.id = 0

    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.05

    rgb = [1.0,0.8,0]
    marker.color.r = rgb[0]
    marker.color.g = rgb[1]
    marker.color.b = rgb[2]
    marker.color.a = 1.0

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


    #CH3.2 pg111-112
    def jacobian(self, ):
        # This array starts with nothing, but Each Ji is appended to the end later on.
        J = np.ndarray([6,0]) # 6 for x,y,z,roll,pitch,yaw
        
        #position of the end effector in the global frame.
        pe = self.getTranslation(0, self.numLinks)

        #Loop over each joint i appending Jpi and Joi to J
        for i in range(self.numLinks):
            #Was pulling zi by translating 
            # zi = np.matmul(self.getT(0, i), z)[:-1] # zi axis in global frame
            zi = self.getT(0,i)[0:3,2]
            pi = self.getTranslation(0,i)
            dp = pe - pi

            # Assuming Revolute Joints
            Jpi = np.cross(zi, dp).reshape(3,-1)
            Joi = zi.reshape(3,-1)
            
            Ji = np.vstack([Jpi, Joi]) # combine Jpi and Joi into Ji
            J = np.hstack([J,Ji]) # concat Ji onto J
        # print(np.round(J, 3))
        return J

    #CH3.5 eq3.52
    def jacobInv(self, ):
        J = self.jacobian()
        JJT = np.matmul(J, J.T)
        try:
            JJTInv = np.linalg.inv(JJT)
        except:
            # print("Using SVD Inv")
            JJTInv = np.linalg.pinv(JJT)
        print("Jinv")
        print( np.round(np.matmul(J.T, JJTInv), 3))
        return np.matmul(J.T, JJTInv)


    #CH3.5 eq3.51
    def getQdotFromVe(self, Ve, dt):
        JInv = self.jacobInv()
        return [ np.matmul(JInv, p) for p in Ve ]

    # #CH3.5 eq3.48 Not part of assignment
    # def getQfromQdot(self, qDot, q0, dt):
    #     q = [q0]
    #     t = 0.0
    #     for i in range(1, len(qDot)):
    #         q.append( q[i-1] + (qDot[i-1]*dt) )
    #         t += dt
    #     return q

def readRobotJson(filename):
    f = open(filename)
    botSpecs = eval(f.read())
    f.close()
    return botSpecs


#check out velocity based control

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
