import numpy as np
import matplotlib.pyplot as plt
import sys
from math import *

def calculate_distance(a_point, b_point):
    distance = pow(a_point[0] - b_point[0], 2)
    distance += pow(a_point[1] - b_point[1], 2)
    return sqrt(distance)

'''Function written by proffessors at University of La Laguna '''
def calculate_transformation_matrix(d, th, a, al):
      return [[cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th)]
             ,[sin(th),  cos(th)*cos(al), -sin(al)*cos(th), a*sin(th)]
             ,[      0,          sin(al),          cos(al),         d]
             ,[      0,                0,                0,         1]
             ]

''' TODO: If I wanted to do this better how should i modify the literal values?'''
def direct_kinematics(robotic_arm):
    transformation_matrix = np.identity(4)

    for i in range(len(robotic_arm.links) - 1):
        transformation_matrix = np.dot(transformation_matrix, calculate_transformation_matrix(0,robotic_arm.joints[i].theta, robotic_arm.links[i], 0))
        temp = np.dot(transformation_matrix, [0,0,0,1])
        robotic_arm.joints[i + 1].x = temp[0]
        robotic_arm.joints[i + 1].y = temp[1]

    transformation_matrix = np.dot(transformation_matrix, calculate_transformation_matrix(0,robotic_arm.joints[i].theta, robotic_arm.links[i], 0))
    temp = np.dot(transformation_matrix, [0,0,0,1])
    robotic_arm.end_point[0] = temp[0]
    robotic_arm.end_point[1] = temp[1]

'''
This is the Joint class.
Every joint is conformed by a x,y coordinate in the 2-dimensional
space and an angle with regards to the origin.
'''
class Joint(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

'''
The PrismaticJoint class is a type of Joint.
It has a maximum shift value.
'''
class PrimasticJoint(Joint):
    def __init__(self, x, y, theta, max_shift):
        super().__init__(x, y, theta)
        self.max_shift = max_shift

'''
The RotationalJoint class is a type of Joint.
It has a maximum rotation angle.
'''
class RotationalJoint(Joint):
    def __init__(self, x, y, theta, max_angle):
        super().__init__(x,y,theta)
        self.max_angle = max_angle

'''
The Robotic arm class is composed by a series of joints connected
by links.
'''
class RoboticArm():
    def __init__(self):
        self.joints = []
        self.links = []
        self.end_point = []

    def add_joint(self,joint):
        self.joints.append(joint)
        self.calculate_links()

    def add_end_point(self,x,y):
        self.end_point.append(x)
        self.end_point.append(y)
        self.links.append(calculate_distance([self.joints[-1].x, self.joints[-1].y], self.end_point))

    # For each pair of joints the distance between them (joint)
    # is calculated.
    def calculate_links(self):
        self.links = [0] * (len(self.joints) - 1)
        for i in range(len(self.joints) - 1):
            self.links[i] = calculate_distance([self.joints[i].x, self.joints[i].y],
                                               [self.joints[i+1].x, self.joints[i+1].y])

    def show_robotic_arm(self):
        plt.figure(1)
        plt.xlim(-10,20)
        plt.ylim(-10,20)

        x_points = []
        for element in self.joints:
            x_points.append(element.x)
        x_points.append(self.end_point[0])

        y_points = []
        for element in self.joints:
            y_points.append(element.y)
        y_points.append(self.end_point[1])

        plt.plot(x_points, y_points, '-o', color="r")
        plt.plot(x_points[-1], y_points[-1], 's', color='b')
        plt.show()
        plt.clf()

    def show_joint_coordinates(self):
        print("Coordenadas de las articulaciones:")
        for i in range(len(self.joints)):
            print("(O" + str(i) + ") = [" + str(self.joints[i].x) + ", " + str(self.joints[i].y) + "]")
        print("(EP) = " + str(self.end_point))

'''
---------------------------------------------------------------------------------------------------------
                                                PROGRAM
---------------------------------------------------------------------------------------------------------
'''

if len(sys.argv) != 3:
  sys.exit("Usage: $python3 " + sys.argv[0] + " x y.\nWhere x and y are the target position of the robotic arm.")
target_position =[float(i) for i in sys.argv[1:]]

robotic_arm = RoboticArm()
robotic_arm.add_joint(RotationalJoint(0,0,0,0))
robotic_arm.add_joint(RotationalJoint(5,0,0,0))
robotic_arm.add_joint(RotationalJoint(10,0,0,0))
robotic_arm.add_end_point(15,0)

direct_kinematics(robotic_arm)
robotic_arm.show_joint_coordinates()
robotic_arm.show_robotic_arm()
