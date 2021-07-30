import random
import pybullet as p

dim_xy = 2
dim = 3
robotId = 0
joint_indices = None
set_random_start = False
set_known_start = True
start_point = None
joint_indices = [0, 1]

def gen_random_start_point() -> list:  # TODO: generate random start points
    '''
        Generate a random start state in Pybullet, this start point should not collide with the object
    '''
    random_start_point = [0] * dim_xy  # x, y, z

    random_x = 0.5
    random_y = 0.5
    while 0.35 < random_x < 0.65 and 0.35 < random_y < 0.65:
        for i in range(dim):
            random_x = random.uniform(-1.5, 1.5)
            random_y = random.uniform(-1.5, 1.5)

    random_start_point[0] = random_x
    random_start_point[1] = random_y

    return random_start_point


def set_random_start_point(robotId, joint_indices, random_start_point):
    '''
        Set a random start state in Pybullet
    '''
    for i in range(dim_xy):
        p.resetJointState(robotId, joint_indices[i], random_start_point[i])


def set_known_start_point(known_start_point):
    '''
        Set a known start state in Pybullet
    '''

    for i in range(dim_xy):
        p.resetJointState(robotId, joint_indices[i], known_start_point[i])


def set_start_points(start_point: list, robotId, joint_indices):
    '''
        Set the start state in Pybullet
    '''
    # ic(start_point)
    for i in range(dim_xy):
        p.resetJointState(robotId, joint_indices[i], start_point[i])

def set():
    if set_random_start:
        random_start_point = gen_random_start_point()
        set_random_start_point(random_start_point)
    elif set_known_start:
        set_known_start_point()
    else:
        set_start_points(start_point)