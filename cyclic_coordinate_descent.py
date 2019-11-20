from math import *

def calculate_distance(a_joint, b_joint):
    distance = pow(a_joint.x - b_joint.x, 2)
    distance += pow(a_joint.y - b_joint.y, 2)
    return sqrt(distance)


def show_joint_coordinates(joints):
    print("Coordenadas de las articulaciones:")
    for i in range(len(joints)):
        print("(O" + str(i) + ") = [" + str(joints[i].x) + ", " + str(joints[i].y) + "]")

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

    def add_joint(self,joint):
        self.joints.append(joint)

    # For each pair of joints the distance between them (joint)
    # is calculated.
    def calculate_joints(self):
        self.links = [0] * (len(self.joints) - 1)
        for i in range(len(self.joints) - 1):
            self.links[i] = calculate_distance(self.joints[i], self.joints[i+1])
