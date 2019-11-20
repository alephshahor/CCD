import pytest
from cyclic_coordinate_descent import *

def test_joint():
    joint = Joint(1,1,0.5)
    assert joint.x == 1
    assert joint.y == 1
    assert joint.theta == 0.5
    assert isinstance(joint, Joint) == True
    assert isinstance(joint, PrimasticJoint) == False
    assert isinstance(joint, RotationalJoint) == False


def test_prismatic_joint():
    joint = PrimasticJoint(1,2,0.5,20)
    assert joint.x == 1
    assert joint.y == 2
    assert joint.theta == 0.5
    assert joint.max_shift == 20
    assert isinstance(joint, Joint) == True
    assert isinstance(joint, PrimasticJoint) == True
    assert isinstance(joint, RotationalJoint) == False

def test_rotational_joint():
    joint = RotationalJoint(1,2,0.5,45)
    assert joint.x == 1
    assert joint.y == 2
    assert joint.theta == 0.5
    assert joint.max_angle == 45
    assert isinstance(joint, Joint) == True
    assert isinstance(joint, PrimasticJoint) == False
    assert isinstance(joint, RotationalJoint) == True

def test_robotic_arm():
    robotic_arm = RoboticArm()
    robotic_arm.add_joint(RotationalJoint(0,0,0,360))
    robotic_arm.add_joint(RotationalJoint(5,0,0,360))
    robotic_arm.add_joint(RotationalJoint(10,0,0,360))
    robotic_arm.add_joint(RotationalJoint(15,0,0,360))
    robotic_arm.calculate_joints()
    assert robotic_arm.links == [5.,5.,5.]

def test_calculate_distance():
    joint_1 = Joint(0,0,0)
    joint_2 = Joint(5,5,0)
    joint_3 = Joint(0,5,0)
    assert round(calculate_distance(joint_1, joint_2),2) == 7.07 
    assert round(calculate_distance(joint_1, joint_3),2) == 5
