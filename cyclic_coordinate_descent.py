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

    transformation_matrix = np.dot(transformation_matrix, calculate_transformation_matrix(0,robotic_arm.joints[-1].theta, robotic_arm.links[-1], 0))
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
        self.theta = radians(theta)

'''
The PrismaticJoint class is a type of Joint.
It has a maximum shift value.
    x: Position in the x-axis
    y: Position in the y-axis
    theta: Angle of the joint with respect to the previous. In this case
           as the joint is prismatic it shouldn't spin by itself.
    max_shift: The maximum amount of displacement.
'''
class PrismaticJoint(Joint):
    def __init__(self, x, y, theta, max_shift):
        super().__init__(x, y, theta)
        self.current_shift = 0
        self.max_shift = max_shift

    def add_shift(self, shift_amount):
        if(shift_amount > 0):
            if(self.current_shift + shift_amount >= self.max_shift):
                offset = self.max_shift - self.current_shift
                self.current_shift = self.max_shift
                return offset
            else :
                self.current_shift += shift_amount
                return shift_amount
        else:
            return 0;

    def show_information(self):
        print("\t-Type: Prismatic Joint")
        print("\t-Position: " + "(" + str(round(self.x,3)) + ", " + str(round(self.y,3)) + ")")
        print("\t-Theta: " + str(round(self.theta,3)) + " rad | " + str(round(degrees(self.theta),3)) + " deg.")
        print("\t-Shift: " + str(self.current_shift) + "\n")

'''
The RotationalJoint class is a type of Joint.
It has a maximum rotation angle.
    x: Position in the x-axis
    y: Position in the y-axis
    theta: Angle of the joint with respect to the previous. In this case
           as the joint is prismatic it shouldn't spin by itself.
    min_angle: Minimum angle that the Joint can have.
    max_angle: Maximum angle that the Joint can have.
'''
class RotationalJoint(Joint):
    def __init__(self, x, y, theta, min_angle, max_angle):
        super().__init__(x,y,theta)
        self.min_angle = radians(min_angle)
        self.max_angle = radians(max_angle)

    def add_rotation(self, rotation_amount):
        if(self.theta + rotation_amount >= self.max_angle):
            offset = self.max_angle - self.theta
            self.theta = self.max_angle
            return offset
        elif(self.theta + rotation_amount <= self.min_angle):
            offset = self.min_angle - self.theta
            self.theta = self.min_angle
            return offset
        else:
            self.theta += rotation_amount
            return rotation_amount


    def show_information(self):
        print("\t-Type: Rotational Joint")
        print("\t-Position: " + "(" + str(round(self.x,3)) + ", " + str(round(self.y,3)) + ")")
        print("\t-Theta: " + str(round(self.theta,3)) + " rad | " + str(round(degrees(self.theta),3)) + " deg.\n")

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


    # TODO: Fix range of the plot and final point.
    def show_robotic_arm(self):
        plt.figure(1)

        min_x_value = self.joints[0].x;
        max_x_value = min_x_value

        min_y_value = self.joints[0].y;
        max_y_value = min_x_value


        x_points = []
        for element in self.joints:
            if(element.x < min_x_value):
                min_x_value = element.x
            if(element.x > max_x_value):
                max_x_value = element.x
            x_points.append(element.x)
        x_points.append(self.end_point[0])

        y_points = []
        for element in self.joints:
            if(element.y < min_y_value):
                min_y_value = element.y
            if(element.y > max_y_value):
                max_y_value = element.y
            y_points.append(element.y)
        y_points.append(self.end_point[1])

        plt.xlim(min(min_x_value, target_position[0]) - 10 , max(max_x_value, target_position[0]) + 10)
        plt.ylim(min(min_y_value, target_position[1]) - 10 , max(max_y_value, target_position[1]) + 10)

        plt.plot(x_points, y_points, '-o', color="r")
        plt.plot(x_points[-1], y_points[-1], 's', color='b')
        plt.plot(target_position[0], target_position[1], '-s', color="g")
        plt.show()
        plt.clf()

    def show_joint_coordinates(self):
        print("Coordenadas de las articulaciones:")
        for i in range(len(self.joints)):
            print("(O" + str(i) + ") = [" + str(self.joints[i].x) + ", " + str(self.joints[i].y) + "]")
        print("(EP) = " + str(self.end_point))

    def show_joint_information(self):
        for joint in self.joints:
            joint.show_information()


'''
---------------------------------------------------------------------------------------------------------
                                                PROGRAM
---------------------------------------------------------------------------------------------------------
'''

if len(sys.argv) < 3:
  sys.exit("Usage: $python3 " + sys.argv[0] + " x y --verbose.\nWhere x and y are the target position of the robotic arm.\nVerbose option allows the user to see step by step of the execution.")
target_position = [float(sys.argv[1]),float(sys.argv[2])]

verbose = False

if(len(sys.argv) > 3):
    if(sys.argv[3] == '--verbose' or sys.argv[3] == '-v'):
        verbose = True
    else:
        sys.exit("Usage: $python3 " + sys.argv[0] + " x y --verbose.\nWhere x and y are the target position of the robotic arm.\nVerbose option allows the user to see step by step of the execution.")


robotic_arm = RoboticArm()
# robotic_arm.add_joint(PrismaticJoint(0,0,0,10))
robotic_arm.add_joint(RotationalJoint(0,0,0,0,90))
robotic_arm.add_joint(RotationalJoint(5,0,0,0,90))
# robotic_arm.add_joint(RotationalJoint(10,0,0,0,90))
# robotic_arm.add_joint(PrismaticJoint(5,0,0,40))
robotic_arm.add_joint(PrismaticJoint(10,0,0,10))
# robotic_arm.add_joint(RotationalJoint(10,0,0,90))
robotic_arm.add_end_point(15,0)
robotic_arm.show_robotic_arm()
direct_kinematics(robotic_arm)

minimum_distance = .01
actual_distance = float("inf")
previous_distance = 0
iteration = 1

'''
TODO: Ask why they're dividing by 100f
'''
while(actual_distance > minimum_distance and abs(previous_distance - actual_distance) > minimum_distance / 100.):
    previous_distance = actual_distance

    link = len(robotic_arm.links) - 1
    for joint in reversed(robotic_arm.joints):

        if(isinstance(joint, RotationalJoint)):

            '''We calculate the vector from the current joint to the end effector.'''
            v_joint_to_effector = [0] * 2
            v_joint_to_effector[0] = robotic_arm.end_point[0] - joint.x
            v_joint_to_effector[1] = robotic_arm.end_point[1] - joint.y
            v_joint_to_effector_dist = sqrt(pow(v_joint_to_effector[0], 2) + pow(v_joint_to_effector[1], 2))

            '''We calculate the vector from the current joint to the target position'''
            v_joint_to_target = [0] * 2
            v_joint_to_target[0] = target_position[0] - joint.x
            v_joint_to_target[1] = target_position[1] - joint.y
            v_joint_to_target_dist = sqrt(pow(v_joint_to_target[0], 2) + pow(v_joint_to_target[1], 2))

            '''We divide each component of the vectors between its respective distance.
               The division is made in order to calculate the vectorial and escalar products.'''

            v_joint_to_effector = [component / v_joint_to_effector_dist for component in v_joint_to_effector]
            v_joint_to_target = [component / v_joint_to_target_dist for component in v_joint_to_target]

            '''The cross product provides the cosine used to obtain the angle(theta) that the
                joint needs to spin'''
            cos_theta = np.dot(v_joint_to_effector, v_joint_to_target)

            # The value is clamped between (-1,1) to avoid floating point number problems with precision
            cos_theta = max(min(cos_theta, 1), -1)
            theta = acos(cos_theta)

            '''The vectorial product provides a perpendicular vector z that specifies the direction
                of theta.'''
            direction_theta = np.cross(v_joint_to_effector, v_joint_to_target)

            if(direction_theta < 0.0):
                theta = -theta

            joint.theta += joint.add_rotation(theta)

        elif(isinstance(joint, PrismaticJoint)):

            v_effector_target = [0] * 2
            v_effector_target[0] = target_position[0] - robotic_arm.end_point[0]
            v_effector_target[1] = target_position[1] - robotic_arm.end_point[1]

            joint_to_x_axis_angle = 0
            for joint_ in robotic_arm.joints:
                joint_to_x_axis_angle += joint_.theta

            effector_to_target_distance = np.dot([cos(joint_to_x_axis_angle),sin(joint_to_x_axis_angle)], v_effector_target)

            joint.theta = 0
            shift = joint.add_shift(effector_to_target_distance)
            robotic_arm.links[link] += shift

        else:
            raise Exception("Not recognized type of joint.")

        direct_kinematics(robotic_arm)

        if(verbose == True):
            robotic_arm.show_robotic_arm()

        link -= 1

    actual_distance = calculate_distance(target_position, robotic_arm.end_point)
    iteration += 1

if actual_distance <= minimum_distance:
    print("\nSuccessfully converged.")
    print("Epsilon threshold: " + str(minimum_distance))
    print("Iterations to converge: " + str(iteration))
    print("---------------------------------------------------")
    print("Distance to the objective: " + str(actual_distance))
    print("Final joint values: \n")
    robotic_arm.show_joint_information()
else:
    print("\nCouldn't converge.")
    print("Epsilon threshold: " + str(minimum_distance))
    print("Iterations: " + str(iteration))
    print("---------------------------------------------------")
    print("Distance to the objective: " + str(actual_distance))
    print("Final joint values: \n")
    robotic_arm.show_joint_information()


robotic_arm.show_robotic_arm()
