# coding: utf8

import numpy as np  # Numpy library
import pybullet_data

import pybullet as p  # PyBullet simulator

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import os
# from icecream import ic

def load_tiago():
    current_path = os.getcwd()
    print('current_path', current_path)

    # The model of Romeo is contained in the path PINOCCHIO_GIT_REPOSITORY/models/romeo
    model_path = current_path + "/models/tiago/"
    #print(model_path)
    mesh_dir = model_path
    urdf_filename = "tiago_single.urdf" #TODO: modify tiago_dual.urdf to eliminate internal collision
    urdf_model_path = model_path + urdf_filename
    #rint(urdf_model_path)

    robot = RobotWrapper.BuildFromURDF(urdf_model_path, [mesh_dir])

    return robot

def configure_simulation(dt, enableGUI, fixed_base=True):
    global jointTorques

    robot = load_tiago()# TODO: load use my own urdf
    # model = robot.model
    # #print(model)
    # data = robot.data
    # #print(data)
    ''':  
Nb joints = 25 (nq=34,nv=24)
  Joint 0 universe: parent=0
  Joint 1 ignore_caster_back_left_1_joint: parent=0  #12
  Joint 2 ignore_caster_back_left_2_joint: parent=1  #13
  Joint 3 ignore_caster_back_right_1_joint: parent=0  #14
  Joint 4 ignore_caster_back_right_2_joint: parent=3  #15
  Joint 5 ignore_caster_front_left_1_joint: parent=0  #16
  Joint 6 ignore_caster_front_left_2_joint: parent=5  #17
  Joint 7 ignore_caster_front_right_1_joint: parent=0  #18
  Joint 8 ignore_caster_front_right_2_joint: parent=7  #19
  Joint 9 ignore_suspension_left_joint: parent=0        #10
  # Joint 10 wheel_left_joint: parent=9                   #11  
  Joint 11 ignore_suspension_right_joint: parent=0          #8
  # Joint 12 wheel_right_joint: parent=11                   #9
  Joint 13 torso_lift_joint: parent=0                       #21
  # Joint 14 arm_1_joint: parent=13     # 31
  # Joint 15 arm_2_joint: parent=14     # 32
  # Joint 16 arm_3_joint: parent=15     # 33
  # Joint 17 arm_4_joint: parent=16     # 34
  # Joint 18 arm_5_joint: parent=17     # 35
  # Joint 19 arm_6_joint: parent=18     # 36
  # Joint 20 arm_7_joint: parent=19     # 37
  # Joint 21 gripper_left_finger_joint: parent=20  # 41
  # Joint 22 gripper_right_finger_joint: parent=20      #42
  Joint 23 head_1_joint: parent=13      # 22
  Joint 24 head_2_joint: parent=23      #23
  '''

    # Start the client for PyBullet
    if enableGUI:
        physicsClient = p.connect(p.GUI)
    else:
        physicsClient = p.connect(p.DIRECT)  # noqa
    # p.GUI for graphical version
    # p.DIRECT for non-graphical version

    # Set gravity (disabled by default)
    p.setGravity(0, 0, -9.81)

    # Load horizontal plane for PyBullet
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    print('pybullet_data.getDataPath(): ', pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")

    # Load the robot for PyBullet
    cur_path = os.getcwd()
    urdf_path = "/cep/robots/tiago/tiago_single.urdf"
    load_path = cur_path + urdf_path
    robotStartPos = [0., 0., 0.]
    robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF(load_path, robotStartPos, robotStartOrientation)

    # Set time step of the simulation
    # dt = 0.001
    p.setTimeStep(dt)
    # realTimeSimulation = True # If True then we will sleep in the main loop to have a frequency of 1/dt

    # Disable default motor control for revolute joints

    Num_Joints = p.getNumJoints(robotId)
    # print('p.getNumJoints(robotId)', p.getNumJoints(robotId))
    # for i in range(Num_Joints):
    #     print('p.getJointState(robotId, {})'.format(i), p.getJointState(robotId, i))
    for i in range(Num_Joints):
        print('p.getJointInfo(robotId, {})'.format(i), p.getJointInfo(robotId, i))

    revoluteJointIndices = [
                            8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
                            21, 22, 23,
                            31, 32, 33, 34, 35, 36, 37,
                            41, 42]
    #revoluteJointIndices = [31, 32, 33, 34, 35, 36, 37]
    '''
    base_antenna_left_joint -> 5
    base_antenna_right_joint -> 6
    wheel_right_jont -> 9, JOINT_PLANAR
    wheel_left_joint -> 11, JOINT_PLANAR
    torso_lift_joint -> 21 JOINT_PRISMATIC
    arm_1_joint -> 31, JOINT_REVOLUTE
    arm_2_joint -> 32, JOINT_REVOLUTE
    arm_3_joint -> 33, JOINT_REVOLUTE
    arm_4_joint -> 34, JOINT_REVOLUTE
    arm_5_joint -> 35, JOINT_REVOLUTE
    arm_6_joint -> 36, JOINT_REVOLUTE
    arm_7_joint -> 37, JOINT_REVOLUTE
    gripper_right_finger_joint -> 41, JOINT_PRISMATIC
    gripper_left_finger_joint -> 42, JOINT_PRISMATIC
    ''' # TODO: add joints which control the direction of the base

    # p.setJointMotorControlArray(robotId,
    #                             jointIndices=revoluteJointIndices,
    #                             controlMode=p.VELOCITY_CONTROL,
    #                             targetVelocities=[0.0 for m in revoluteJointIndices],
    #                             forces=[0.0 for m in revoluteJointIndices])

    # Enable torque control for revolute joints
    jointTorques = [0.0 for m in revoluteJointIndices]
    p.setJointMotorControlArray(robotId, revoluteJointIndices, controlMode=p.TORQUE_CONTROL, forces=jointTorques)

    #getPosVelJoints(robotId, revoluteJointIndices)

    # Compute one step of simulation for initialization
    #p.stepSimulation()

    return robotId, robot, revoluteJointIndices, planeId


# Function to get the position/velocity of all joints
def getPosVelJoints(robotId, revoluteJointIndices):

    linkStates = p.getLinkStates(robotId, revoluteJointIndices, computeLinkVelocity=True,  computeForwardKinematics=True)
    # print('linkStates: ', linkStates)
    # print('np.shape(linkStates): ', np.shape(linkStates)) # (7, 8)

    pos_link_world = np.vstack((np.array([[linkStates[i_joint][0] for i_joint in range(len(linkStates))]]))) # (7, 3)
    orn_link_world = np.vstack((np.array([[linkStates[i_joint][1] for i_joint in range(len(linkStates))]]))) # (7, 4)
    # print('pos_link_world: ', pos_link_world)
    # print('np.shape(pos_link_world):', np.shape(pos_link_world))
    # print('orn_link_world: ', orn_link_world)
    # print('np.shape(orn_link_world): ', np.shape(orn_link_world))

    frame_pos_world = np.vstack((np.array([[linkStates[i_joint][4] for i_joint in range(len(linkStates))]])))
    linear_vel_world = np.vstack((np.array([[linkStates[i_joint][6] for i_joint in range(len(linkStates))]])))

    jointStates = p.getJointStates(robotId, revoluteJointIndices)  # State of all joints (position, velocity, reaction forces, appliedJointMotortoruqe)
    # print('np.shape(jointStates):', np.shape(jointStates))
    # print('len(jointStates): ', len(jointStates))
    # print('jointStates[0]: ', jointStates[0])
    # print('jointStates[0][0]: ', jointStates[0][0])
    #baseState = p.getBasePositionAndOrientation(robotId)  # Position and orientation of the free flying base (Position, orientation)
    #baseVel = p.getBaseVelocity(robotId)  # Velocity of the free flying base  (linear velocity, angular velocity)

    # Reshaping data into q and qdot
    q = np.vstack((np.array([[jointStates[i_joint][0] for i_joint in range(len(jointStates))]]).transpose()))
    # q = np.vstack((np.array([baseState[0]]).transpose(), np.array([baseState[1]]).transpose(),
    #                np.array([[jointStates[i_joint][0] for i_joint in range(len(jointStates))]]).transpose()))
    #print('q: ', q) # ([:3] -> base position,
    #                 # [3:7] -> base orientation in Quatenion,
    #                 # [7:9] -> wheel right and left,
    #                 # [9:16] -> position of 7 joints,
    #                 # [16:] -> position of gripper right finger and left finger
    #print('np.shape(q): ', np.shape(q)) # (18, 1), baseState[1] is orientation in Quatenion

    qdot = np.vstack((np.array([[jointStates[i_joint][1] for i_joint in range(len(jointStates))]]).transpose()))
    #qdot = np.vstack((np.array([baseVel[0]]).transpose(), np.array([baseVel[1]]).transpose(),
    #                   np.array([[jointStates[i_joint][1] for i_joint in range(len(jointStates))]]).transpose()))
    #print('qot: ', qdot) # ([:3] -> base linear velocity
    #                     # [3:6] -> base angular velocity,
    #                     # [6:8] -> velocity of wheel right and left,
    #                     # [8:15] -> velocity of 7 joints,
    #                     # [15:] -> velocity of gripper right finger and left finger
    #print('np.shape(qdot): ', np.shape(qdot)) # (17, 1)

    return q, qdot, pos_link_world, orn_link_world, frame_pos_world, linear_vel_world


def Debug_test(dt, enableGUI):
    global jointTorques

    # Start the client for PyBullet
    if enableGUI:
        physicsClient = p.connect(p.GUI)
    else:
        physicsClient = p.connect(p.DIRECT)  # noqa
    # p.GUI for graphical version
    # p.DIRECT for non-graphical version

    # Set gravity (disabled by default)
    p.setGravity(0, 0, -9.81)

    # Load horizontal plane for PyBullet
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # print('pybullet_data.getDataPath(): ', pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    # Load the robot for PyBullet
    cur_path = os.getcwd()
    urdf_path = "/tiago_dual_holoBase0712.urdf"
    load_path = cur_path + urdf_path
    robotStartPos = [0., 0., 0.]
    robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF(load_path)

    # Set time step of the simulation
    # dt = 0.001
    #p.setTimeStep(dt)
    # realTimeSimulation = True # If True then we will sleep in the main loop to have a frequency of 1/dt

    # Disable default motor control for revolute joints

    Num_Joints = p.getNumJoints(robotId)
    # print('p.getNumJoints(robotId)', p.getNumJoints(robotId))
    # for i in range(Num_Joints):
    #     print('p.getJointState(robotId, {})'.format(i), p.getJointState(robotId, i))
    for i in range(Num_Joints):
        print('p.getJointInfo(robotId, {})'.format(i), p.getJointInfo(robotId, i))

    revoluteJointIndices = [
        8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
        21, 22, 23,
        31, 32, 33, 34, 35, 36, 37,
        41, 42]

    ignore_caster_back_left_1_joint = p.addUserDebugParameter('ignore_caster_back_left_1_joint', -10, 10, 0)  #16
    ignore_caster_back_left_2_joint = p.addUserDebugParameter('ignore_caster_back_left_2_joint', -10, 10, 0)  #17
    ignore_caster_back_right_1_joint = p.addUserDebugParameter('ignore_caster_back_right_1_joint', -10, 10, 0) #18
    ignore_caster_back_right_2_joint = p.addUserDebugParameter('ignore_caster_back_right_2_joint', -10, 10, 0) #19
    ignore_caster_front_left_1_joint = p.addUserDebugParameter('ignore_caster_front_left_1_joint', -10, 10, 0) #12
    ignore_caster_front_left_2_joint = p.addUserDebugParameter('ignore_caster_front_left_2_joint', -10, 10, 0) #13
    ignore_caster_front_right_1_joint = p.addUserDebugParameter('ignore_caster_front_right_1_joint', -10, 10, 0) #14
    ignore_caster_front_right_2_joint = p.addUserDebugParameter('ignore_caster_front_right_2_joint', -10, 10, 0) #15

    ignore_suspension_left_joint = p.addUserDebugParameter('ignore_suspension_left_joint', -10, 10, 0) #10
    ignore_suspension_right_joint = p.addUserDebugParameter('ignore_suspension_right_joint', -10, 10, 0) #8

    wheel_left_joint = p.addUserDebugParameter('wheel_left_joint', -100, 100, 0) #11
    wheel_right_joint = p.addUserDebugParameter('wheel_right_joint', -100, 100, 0) #9

    torso_lift_joint = p.addUserDebugParameter('torso_lift_joint', -10, 10, 0) #21

    arm1 = p.addUserDebugParameter('arm_1_joint', -10, 10, 0) #31
    arm2 = p.addUserDebugParameter('arm_2_joint', -10, 10, 0) #32
    arm3 = p.addUserDebugParameter('arm_3_joint', -10, 10, 0) #33
    arm4 = p.addUserDebugParameter('arm_4_joint', -10, 10, 0) #34
    arm5 = p.addUserDebugParameter('arm_5_joint', -10, 10, 0) #35
    arm6 = p.addUserDebugParameter('arm_6_joint', -10, 10, 0) #36
    arm7 = p.addUserDebugParameter('arm_7_joint', -10, 10, 0) #37

    gripper_right_finger_joint = p.addUserDebugParameter('gripper_right_finger_joint', -10, 10, 0) #41
    gripper_left_finger_joint = p.addUserDebugParameter('gripper_left_finger_joint', -10, 10, 0) #42

    head_1_joint = p.addUserDebugParameter('head_1_joint', -10, 10, 0) #22
    head_2_joint = p.addUserDebugParameter('head_2_joint', -10, 10, 0) #23

    while True:

        ibl1 = p.readUserDebugParameter(ignore_caster_back_left_1_joint)
        ibl2 = p.readUserDebugParameter(ignore_caster_back_left_2_joint)
        ibr1 = p.readUserDebugParameter(ignore_caster_back_right_1_joint)
        ibr2 = p.readUserDebugParameter(ignore_caster_back_right_2_joint)
        ifl1 = p.readUserDebugParameter(ignore_caster_front_left_1_joint)
        ifl2 = p.readUserDebugParameter(ignore_caster_front_left_2_joint)
        ifr1 = p.readUserDebugParameter(ignore_caster_front_right_1_joint)
        ifr2 = p.readUserDebugParameter(ignore_caster_front_right_2_joint)

        isl = p.readUserDebugParameter(ignore_suspension_left_joint)
        isr = p.readUserDebugParameter(ignore_suspension_right_joint)

        wl = p.readUserDebugParameter(wheel_left_joint)
        wr = p.readUserDebugParameter(wheel_right_joint)

        tl = p.readUserDebugParameter(torso_lift_joint)

        a1 = p.readUserDebugParameter(arm1)
        a2 = p.readUserDebugParameter(arm2)
        a3 = p.readUserDebugParameter(arm3)
        a4 = p.readUserDebugParameter(arm4)
        a5 = p.readUserDebugParameter(arm5)
        a6 = p.readUserDebugParameter(arm6)
        a7 = p.readUserDebugParameter(arm7)

        right_finger = p.readUserDebugParameter(gripper_right_finger_joint)
        left_finger = p.readUserDebugParameter(gripper_left_finger_joint)

        h1 = p.readUserDebugParameter(head_1_joint)
        h2 = p.readUserDebugParameter(head_2_joint)
        #
        # SetPositions = [-10, 0., -10, 0.,
        #                 0., 0., 0., 0.,
        #                 0., 0., 0., 0.,
        #                 0., 0., 0.,
        #                 0., 0., 0., 0., 0., 0., 0.,
        #                 0., 0.,
        #                  ]

        footprint_link_state = p.getLinkState(robotId, 0)
        fp_link_pos = footprint_link_state[0]
        fp_link_orn = footprint_link_state[1]
        print('#######')
        print(fp_link_pos)
        print(fp_link_orn)
        p.addUserDebugLine(fp_link_pos,
                           (fp_link_pos[0]+1, fp_link_pos[1], fp_link_pos[2]),
                           [1, 0, 0],
                           5.)

        p.setJointMotorControlArray(robotId,
                                    revoluteJointIndices,
                                    p.POSITION_CONTROL,
                                    # targetPositions = SetPositions)
                                    targetPositions = [isr, wr, isl, wr,
                                                       ifr1, ifr2, ifr1, ifl2,
                                                       ibr1, ibr2, ibl1, ibl2,
                                                       tl, h1, h2,
                                                       a1, a2, a3, a4, a5, a6, a7,
                                                       right_finger, left_finger])

    # Enable torque control for revolute joints
    #     jointTorques = [0.0 for m in revoluteJointIndices]

        #getPosVelJoints(robotId, revoluteJointIndices)

        # print out link world position
        # print('link world position: ',
        #       p.getLinkState(robotId, 37, computeLinkVelocity=1, computeForwardKinematics=1)[0])
        # print out world link frame position
        # print('world link frame position: ',
        #       p.getLinkState(robotId, 37, computeLinkVelocity=1, computeForwardKinematics=1)[4])
        # for i in range(Num_Joints):
        #     print('p.getJointState(robotId, {})'.format(i), p.getJointState(robotId, i))
        # print('world link frame orientation : ',
        #       p.getLinkState(robotId, 37, computeLinkVelocity=1, computeForwardKinematics=1)[5])

    # Compute one step of simulation for initialization
        p.stepSimulation()


Debug_test(0.001, enableGUI=True)
