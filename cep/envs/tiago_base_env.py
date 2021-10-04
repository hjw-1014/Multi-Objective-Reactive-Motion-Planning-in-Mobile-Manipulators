import math

import pybullet as p
import numpy as np
import os

import pybullet_data
import torch
from gym import error, spaces

def solve_euler(q, dq, dt):
    return q+dq*dt

class TiagoOnlyBase(): # TODO: 10.02

    """
    Tiago with base Simple Environment.
    Environment to evaluate the quality of prior composition for obstacle avoidance + Goto Target
    State dim 14; 2 q / 2 dq;
    Action dim (2,2); 2 q_des/ 2 dq_des

    """

    ######################## Tiago with one parallel gripper hand ########################

    def __init__(self, reward_type=0, time_step=1./240., Target_pose=[1.5, 1.2, 0.8]):
        p.connect(p.GUI_SERVER)
        self.Base_X_ID = 0
        self.Base_Y_ID = 1
        self.Target_pos = Target_pose
        self.Base_position = [1.2, 1.0, 0.0]
        self.name_base_link = "world"
        self.JOINT_ID = [0, 1]
        self.link_names = ['X', 'Y']
        self.delta = time_step

        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setTimeStep(time_step)
        p.setGravity(0, 0, 0)  # TODO: SET -9.81???

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        basename = os.path.dirname(os.path.abspath(__file__ + '/../../'))
        self.robot_file = os.path.join(basename, "robots/tiago/tiago_only_base.urdf")

        self.robot = p.loadURDF(self.robot_file, flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

        self.ballId = p.loadURDF('sphere_1cm.urdf', [1.5, 1.2, 0.8], p.getQuaternionFromEuler([0., 0., 0.]))
        joint_num = p.getNumJoints(self.robot)
        #print("joint_num ", joint_num)

        # TODO: add 08.18
        p.addUserDebugLine([0., 0., 0.], [1.2, 1.0, 0.], lineColorRGB=[1., 0., 0.])

        self.Box = p.loadURDF('cube_small.urdf', np.array([0.5, 0.5, 0.15]),  # TODO: Put an object in target postion
                   p.getQuaternionFromEuler([0, 0, 0]), globalScaling=6,
                   useFixedBase=True)
        ##############################################################

        ############## Robot Initial State #################
        self.q_home = [-0.15, -0.15]
        self.qlimits = [[-3.0, 3.0],
                        [-3.0, 3.0]]

        ## Observation-Action Space ##
        self.action_space = spaces.Box(-10, 10, shape=(2,), dtype='float32')
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(18,), dtype='float32')  # TODO: Shape???

        self.max_obj_dist_init = 0.01

        self.reward_type = reward_type

        ## VARIABLES ##
        self.DYNAMICS_ON = 0

        ## Threshold ##
        self.break_threshold = 0.08
        self.success_threshold = 0.02

    @property
    def dt(self):
        return 1 / self.fq * self.n_substeps

    def reset(self, q0=None):
        # Initialize Pybullet Environment
        #print('====RESET=====')
        q_init = torch.zeros((1, 2))
        if q0 is None:
            for i, q_i in enumerate(self.q_home):
                q_i = q_i + np.random.rand(1) * 0.3  # Reset initial joint positions
                #print('## q_i ##', q_i)
                p.resetJointState(self.robot, self.JOINT_ID[i], q_i)
                q_init[0][i] = q_i[0]
            p.stepSimulation()
        else:
            for i, q_i in enumerate(q0):
                #print('## else: q_i ##', q_i)
                p.resetJointState(self.robot, self.JOINT_ID[i], q_i)
            p.stepSimulation()
        #q = np.array([[p.getJointState(self.robot, 0)[0], p.getJointState(self.robot, 1)[0]]])
        q = q_init
        print("### Start point: ", q)
        # Fixed the dimensions for the returned state
        return np.concatenate((q, np.zeros_like(q)), 1)  # (1, 4)

    def step(self, action, state):
        #print('===== start step =====')
        a_p = action[0]  # Position
        a_v = action[1]  # Velocity


        pos = state[0][:2]
        v = state[0][2:]

        new_v = action * self.delta + v
        new_q = new_v * self.delta + pos

        for i, q_i in enumerate(new_q):
            #print('======= Set joint motor contorl ========')
            if self.DYNAMICS_ON:  # Use setJointMotorControl or resetJointState to control q
                #print('1step')
                p.setJointMotorControl2(self.robot, self.JOINT_ID[i], p.POSITION_CONTROL, targetPosition=q_i,
                                        #targetVelocity=a_v[i],
                                        force=p.getJointInfo(self.robot, self.JOINT_ID[i])[10])
            else:
                #print('q_i', q_i)
                #print('2step')
                p.resetJointState(self.robot, self.JOINT_ID[i], q_i)

        #self.q = np.array([[p.getJointState(self.robot, 0)[0], p.getJointState(self.robot, 1)[0]]])
        self.q = new_q.reshape(1, 2)

        if self.DYNAMICS_ON:
            self.dq = np.array(
                [[p.getJointState(self.robot, 0)[0], p.getJointState(self.robot, 1)[0]]])
        else:
            #self.dq = a_v[None, :]
            self.dq = new_v.reshape(1, 2)

        obs = np.concatenate((self.q, self.dq), 1)

        d = self._compute_base_distance()

        #r = self._compute_reward()

        done = self._check_done()

        success = self._check_success()

        col = self._check_base_collision()

        #q_vals = self.getPosVelJoints()
        #print('joint positions: ', self.getPosVelJoints())

        return obs, d, done, success, col

    # AAL: make sure the initial configuration is not in self-collision
    def check_self_collision(self):
        closest_points = p.getClosestPoints(self.robot, self.robot, self.max_obj_dist_init)

        n_links = p.getNumJoints(self.robot) - 1
        if len(closest_points) > 3 * n_links - 2:  # TODO: How to compute???
            print('WARNING: invalid initialization: in self-collision, please regenerate an initial configuration!')
            return True
        else:
            return False

    def check_endPosition(self):

        end_pos = p.getLinkState(self.robot, self.EE_ID)[0]

        return end_pos

    def _compute_base_distance(self):

        cur_x = np.array(p.getLinkState(self.robot, self.Base_X_ID)[0])
        cur_y = np.array(p.getLinkState(self.robot, self.Base_Y_ID)[0])
        #print('current position: ', cur_pos)
        dist = math.hypot(cur_x[0] - self.Base_position[0], cur_y[1] - self.Base_position[1])

        return dist

    def _compute_reward(self):

        cur_pos = np.array(p.getLinkState(self.robot, self.EE_ID)[0])
        #print('current position: ', cur_pos)
        rwd = np.sqrt(np.sum(np.power(np.abs(cur_pos - self.Base_position), 2)))

        return rwd

    def _check_done(self):
        if self._compute_base_distance() <= self.max_obj_dist_init and not self._check_base_collision():
            return True
        return False

    def _check_success(self):
        success = False
        if self._compute_base_distance() <= self.max_obj_dist_init:
            success = True
        return success

    def getPosVelJoints(self):  # Function to get the position/velocity of all joints from pybullet


        jointStates = p.getJointStates(self.robot, self.JOINT_ID)  # State of all joints (position, velocity, reaction forces, appliedJointMotortoruqe)
        joint_pos = np.vstack((np.array([[jointStates[i_joint][0] for i_joint in range(len(jointStates))]]).transpose()))
        link_pos = p.getLinkState(self.robot, self.EE_ID)
        return joint_pos, link_pos

    def _check_base_collision(self):

        closest_points = p.getClosestPoints(bodyA=self.robot, bodyB=self.Box, distance=self.max_obj_dist_init)

        if len(closest_points) > 0:
            print('WARNING: Robot collides with Box!!!')
            return True
        return False
