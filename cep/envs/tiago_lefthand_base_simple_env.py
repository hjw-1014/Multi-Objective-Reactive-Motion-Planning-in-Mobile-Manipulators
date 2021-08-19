import pybullet as p
import numpy as np
import os

import pybullet_data
from gym import error, spaces

def solve_euler(q, dq, dt):
    return q+dq*dt

class Tiago_LeftParallelHand_Base(): # TODO: 08.06

    """
    Tiago with one parallel gripper and base Simple Environment.
    Environment to evaluate the quality of prior composition for obstacle avoidance + Goto Target
    State dim 14; 10 q / 10 dq;
    Action dim (10,2); 10 q_des/ 10 dq_des

    """

    ######################## Tiago with one parallel gripper hand ########################

    def __init__(self, reward_type=0, time_step=1./240., Target_pose=[1.5, 1.2, 0.8]):
        self.EE_link = "arm_7_link"
        self.EE_ID = 43
        self.Target_pos = Target_pose
        self.name_base_link = "world"
        self.JOINT_ID = [0, 1, 2, 34, 35, 36, 37, 38, 39, 40]
        self.link_names = ['X', 'Y', 'R', "arm_1_link", "arm_2_link", "arm_3_link", "arm_4_link", "arm_5_link", "arm_6_link", self.EE_link]

        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setTimeStep(time_step)
        p.setGravity(0, 0, 0)  # TODO: SET -9.81???

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        basename = os.path.dirname(os.path.abspath(__file__ + '/../../'))
        self.robot_file = os.path.join(basename, "robots/tiago/tiago_single_with_holoBaseCXY.urdf")

        self.robot = p.loadURDF(self.robot_file, flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

        self.ballId = p.loadURDF('sphere_1cm.urdf', [1.5, 1.2, 0.8], p.getQuaternionFromEuler([0., 0., 0.]))
        joint_num = p.getNumJoints(self.robot)
        #print("joint_num ", joint_num)

        # TODO: add 08.18
        p.addUserDebugLine([0., 0., 0.], [1.2, 1.0, 0.], lineColorRGB=[1., 0., 0.])

        p.loadURDF('cube_small.urdf', np.array([0.5, 0.5, 0.15]),  # TODO: Put an object in target postion
                   p.getQuaternionFromEuler([0, 0, 0]), globalScaling=6,
                   useFixedBase=True)
        ##############################################################

        ############## Robot Initial State #################
        self.q_home = [-0.1, -0.1, 0., 0.5, 0., 0., 0., 0., 0., 0.]
        self.qlimits = [[-5.0, 5.0],
                        [-5.0, 5.0],
                        [-3.1416,  3.1416],
                        [0.0, 2.74889357189],
                        [-1.57079632679, 1.0908307825],
                        [-3.53429173529, 1.57079632679],
                        [-0.392699081699, 2.35619449019],
                        [-2.09439510239, 2.09439510239],
                        [-1.57079632679, 1.57079632679],
                        [-2.09439510239, 2.09439510239]]

        ## Observation-Action Space ##
        self.action_space = spaces.Box(-10, 10, shape=(10,), dtype='float32')
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(18,), dtype='float32')  # TODO: Shape???

        self.max_obj_dist_init = 0.01

        self.reward_type = reward_type

        ## VARIABLES ##
        self.DYNAMICS_ON = False

        ## Threshold ##
        self.break_threshold = 0.08
        self.success_threshold = 0.02

    @property
    def dt(self):
        return 1 / self.fq * self.n_substeps

    def reset(self, q0=None):
        # Initialize Pybullet Environment
        #print('====RESET=====')
        if q0 is None:
            for i, q_i in enumerate(self.q_home):
                q_i = q_i + np.random.randn() * 0.1  # Reset initial joint positions
                #print('## q_i ##', q_i)
                p.resetJointState(self.robot, self.JOINT_ID[i], q_i)
            p.stepSimulation()
        else:
            for i, q_i in enumerate(q0):
                #print('## else: q_i ##', q_i)
                p.resetJointState(self.robot, self.JOINT_ID[i], q_i)
            p.stepSimulation()

        q = np.array(
            [[p.getJointState(self.robot, 0)[0], p.getJointState(self.robot, 1)[0], p.getJointState(self.robot, 2)[0],
              p.getJointState(self.robot, 34)[0], p.getJointState(self.robot, 35)[0], p.getJointState(self.robot, 36)[0],
              p.getJointState(self.robot, 37)[0], p.getJointState(self.robot, 38)[0],
              p.getJointState(self.robot, 39)[0], p.getJointState(self.robot, 40)[0]]])

        # Fixed the dimensions for the returned state
        return np.concatenate((q, np.zeros_like(q)), 1)  # (7, 2)

    def step(self, action):
        #print('===== start step =====')
        a_p = action[0]  # Position
        a_v = action[1]  # Velocity

        for i, q_i in enumerate(a_p):
            #print('======= Set joint motor contorl ========')
            if self.DYNAMICS_ON:  # Use setJointMotorControl or resetJointState to control q
                #print('1step')
                p.setJointMotorControl2(self.robot, self.JOINT_ID[i], p.POSITION_CONTROL, targetPosition=q_i,
                                        targetVelocity=a_v[i],
                                        force=p.getJointInfo(self.robot, self.JOINT_ID[i])[10])
            else:
                #print('q_i', q_i)
                #print('2step')
                p.resetJointState(self.robot, self.JOINT_ID[i], q_i)

        self.q = np.array(
            [[p.getJointState(self.robot, 0)[0], p.getJointState(self.robot, 1)[0], p.getJointState(self.robot, 2)[0],
              p.getJointState(self.robot, 34)[0], p.getJointState(self.robot, 35)[0], p.getJointState(self.robot, 36)[0],
              p.getJointState(self.robot, 37)[0], p.getJointState(self.robot, 38)[0],
              p.getJointState(self.robot, 39)[0], p.getJointState(self.robot, 40)[0]]])

        if self.DYNAMICS_ON:
            self.dq = np.array(
                [[p.getJointState(self.robot, 0)[0], p.getJointState(self.robot, 1)[0], p.getJointState(self.robot, 2)[0],
                  p.getJointState(self.robot, 34)[0], p.getJointState(self.robot, 35)[0], p.getJointState(self.robot, 36)[0],
                  p.getJointState(self.robot, 37)[0], p.getJointState(self.robot, 38)[0],
                  p.getJointState(self.robot, 39)[0], p.getJointState(self.robot, 40)[0]]])
        else:
            self.dq = a_v[None, :]

        obs = np.concatenate((self.q, self.dq), 1)

        r = self._compute_reward()

        done = self._check_done(r)

        success = self._check_success(r)

        q_vals = self.getPosVelJoints()
        #print('joint positions: ', self.getPosVelJoints())

        return obs, r, done, success, q_vals

    # AAL: make sure the initial configuration is not in self-collision
    def check_collision(self):
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

    def _compute_reward(self):

        cur_pos = np.array(p.getLinkState(self.robot, self.EE_ID)[0])
        #print('current position: ', cur_pos)
        dist = np.sqrt(np.sum(np.power(np.abs(cur_pos - self.Target_pos), 2)))

        return dist

    def _check_done(self, r):
        return False

    def _check_success(self, r):
        success = False
        return success

    def getPosVelJoints(self):  # Function to get the position/velocity of all joints from pybullet


        jointStates = p.getJointStates(self.robot, self.JOINT_ID)  # State of all joints (position, velocity, reaction forces, appliedJointMotortoruqe)
        joint_pos = np.vstack((np.array([[jointStates[i_joint][0] for i_joint in range(len(jointStates))]]).transpose()))
        link_pos = p.getLinkState(self.robot, self.EE_ID)
        return joint_pos, link_pos