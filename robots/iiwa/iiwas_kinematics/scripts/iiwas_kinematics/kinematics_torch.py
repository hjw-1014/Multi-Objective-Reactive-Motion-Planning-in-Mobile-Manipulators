import numpy as np
import torch

# import matplotlib.pyplot as plt

IIWA_JOINT_MIN_LIMITS = torch.deg2rad(torch.tensor([-170., -120., -170., -120., -170., -120., -175.]))
IIWA_JOINT_MAX_LIMITS = torch.deg2rad(torch.tensor([170., 120., 170., 120., 170., 120., 175.]))


class Rotation:
    def __init__(self, quat, normalize=True):
        self._quat = quat if isinstance(quat, torch.Tensor) else torch.tensor(quat).double()
        if normalize:
            self._quat /= torch.norm(quat)

    @classmethod
    def from_quat(cls, quat):
        return cls(quat, normalize=True)

    @classmethod
    def from_matrix(cls, matrix):
        trace = torch.trace(matrix)
        quat = torch.zeros(4).double()
        if trace > 1e-5:
            s = torch.sqrt(trace + 1) * 2
            quat[0] = s / 4
            quat[1] = (matrix[2, 1] - matrix[1, 2]) / s
            quat[2] = (matrix[0, 2] - matrix[2, 0]) / s
            quat[3] = (matrix[1, 0] - matrix[0, 1]) / s
        elif (matrix[0, 0] > matrix[1, 1]) and (matrix[0, 0] > matrix[2, 2]):
            s = torch.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            quat[0] = (matrix[2, 1] - matrix[1, 2]) / s
            quat[1] = 0.25 * s
            quat[2] = (matrix[0, 1] + matrix[1, 0]) / s
            quat[3] = (matrix[0, 2] + matrix[2, 0]) / s
        elif (matrix[1, 1] > matrix[2, 2]):
            s = torch.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            quat[0] = (matrix[0, 2] - matrix[2, 0]) / s
            quat[1] = (matrix[0, 1] + matrix[1, 0]) / s
            quat[2] = 0.25 * s
            quat[3] = (matrix[1, 2] + matrix[2, 1]) / s
        else:
            s = torch.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            quat[0] = (matrix[1, 0] - matrix[0, 1]) / s
            quat[1] = (matrix[0, 2] + matrix[2, 0]) / s
            quat[2] = (matrix[1, 2] + matrix[2, 1]) / s
            quat[3] = 0.25 * s
        return cls(quat, normalize=False)

    @classmethod
    def from_rotvec(cls, rotvec):
        quat = torch.zeros(4).double()
        norm = torch.norm(rotvec)
        small_angle = (norm <= 1e-3)

        if small_angle:
            scale = (0.5 - norm ** 2 / 48 + norm ** 4 / 3840)
        else:
            scale = (torch.sin(norm / 2) / norm)

        quat[0] = torch.cos(norm / 2)
        quat[1:] = scale * rotvec
        return cls(quat, normalize=True)

    @classmethod
    def from_euler_yzy(cls, yzy):
        a = yzy[0]
        b = yzy[1]
        c = yzy[2]
        rot_mat = torch.tensor(
            [[-torch.sin(a) * torch.sin(c) + torch.cos(a) * torch.cos(b) * torch.cos(c),
              -torch.sin(b) * torch.cos(c),
              torch.sin(a) * torch.cos(b) * torch.cos(c) + torch.sin(c) * torch.cos(a)],
             [torch.sin(b) * torch.cos(a),
              torch.cos(b),
              torch.sin(a) * torch.sin(b)],
             [-torch.sin(a) * torch.cos(c) - torch.sin(c) * torch.cos(a) * torch.cos(b),
              torch.sin(b) * torch.sin(c),
              -torch.sin(a) * torch.sin(c) * torch.cos(b) + torch.cos(a) * torch.cos(c)]])
        return cls.from_matrix(rot_mat)

    def as_quat(self):
        return self._quat

    def as_matrix(self):
        quat = self._quat
        R = torch.eye(3).double()
        R[0, 0] = 1 - 2 * (quat[2] ** 2 + quat[3] ** 2)
        R[0, 1] = 2 * (quat[1] * quat[2] - quat[3] * quat[0])
        R[0, 2] = 2 * (quat[1] * quat[3] + quat[2] * quat[0])

        R[1, 0] = 2 * (quat[1] * quat[2] + quat[3] * quat[0])
        R[1, 1] = 1 - 2 * (quat[1] ** 2 + quat[3] ** 2)
        R[1, 2] = 2 * (quat[2] * quat[3] - quat[1] * quat[0])

        R[2, 0] = 2 * (quat[1] * quat[3] - quat[2] * quat[0])
        R[2, 1] = 2 * (quat[2] * quat[3] + quat[1] * quat[0])
        R[2, 2] = 1 - 2 * (quat[1] ** 2 + quat[2] ** 2)
        return R

    def as_rotvec(self):
        angle = 2 * np.arctan2(np.linalg.norm(self._quat[1:]), self._quat[0])

        small_angle = angle <= 1e-3

        if small_angle:
            scale = (2 + angle[small_angle] ** 2 / 12 + 7 * angle[small_angle] ** 4 / 2880)
        else:
            scale = (angle / np.sin(angle / 2))

        rotvec = scale * self._quat[1:]
        return rotvec

    def as_euler_yzy(self):
        rot_mat = self.as_matrix()
        yzy = torch.zeros(3)
        if torch.isclose(torch.abs(rot_mat[1, 1]), torch.ones(1).double()):
            yzy[0] = torch.atan2(rot_mat[0, 2], rot_mat[0, 0])
            yzy[1] = torch.acos(rot_mat[1, 1])
            yzy[2] = 0
        else:
            yzy[0] = torch.atan2(rot_mat[1, 2], rot_mat[1, 0])
            yzy[1] = torch.atan2(torch.sqrt(rot_mat[1, 2] ** 2 + rot_mat[1, 0] ** 2), rot_mat[1, 1])
            yzy[2] = torch.atan2(rot_mat[2, 1], -rot_mat[0, 1])
        return yzy


class KinematicsTorch:
    def __init__(self, tcp_pos=None, tcp_quat=None):
        self.d_bs = torch.tensor(0.36)
        self.d_se = torch.tensor(0.42)
        self.d_ew = torch.tensor(0.4)
        self.d_wf = torch.tensor(0.151)

        # DH_paramters a_i, alpha_i, d_i
        self.dh_a = torch.tensor([0., 0., 0., 0., 0., 0., 0.])
        self.dh_alpha = torch.tensor([-np.pi / 2, np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0.])
        self.dh_d = torch.tensor([self.d_bs, 0., self.d_se, 0., self.d_ew, 0., self.d_wf])

        self.joint_limits = torch.stack((IIWA_JOINT_MIN_LIMITS, IIWA_JOINT_MAX_LIMITS), dim=0).T

        self.joint_vel_limits = torch.deg2rad(torch.tensor([[-85., 85.],
                                                            [-85., 85.],
                                                            [-100., 100.],
                                                            [-75., 75.],
                                                            [-130., 130.],
                                                            [-135., 135.],
                                                            [-135., 135.]]))

        self.singularity_eps = 0.1

        self.T_ee = torch.eye(4).double()
        if tcp_pos is not None:
            self.T_ee[:3, 3] = tcp_pos
        if tcp_quat is not None:
            self.T_ee[:3, :3] = Rotation.from_quat(tcp_quat).as_matrix()

    def _transform_i(self, index, theta_i):
        alpha_i = self.dh_alpha[index]
        a_i = self.dh_a[index]
        d_i = self.dh_d[index]
        T_i = torch.eye(4).double()
        T_i[0, 0] = torch.cos(theta_i)
        T_i[0, 1] = -torch.sin(theta_i) * torch.cos(alpha_i)
        T_i[0, 2] = torch.sin(theta_i) * torch.sin(alpha_i)
        T_i[0, 3] = a_i * torch.cos(theta_i)
        T_i[1, 0] = torch.sin(theta_i)
        T_i[1, 1] = torch.cos(theta_i) * torch.cos(alpha_i)
        T_i[1, 2] = -torch.cos(theta_i) * torch.sin(alpha_i)
        T_i[1, 3] = a_i * torch.sin(theta_i)
        T_i[2, 0] = 0.
        T_i[2, 1] = torch.sin(alpha_i)
        T_i[2, 2] = torch.cos(alpha_i)
        T_i[2, 3] = d_i
        return T_i

    def _transform(self, q):
        T = torch.eye(4).double()
        for i, theta_i in enumerate(q):
            T_i = self._transform_i(i, theta_i)
            T = T @ T_i
        T = T @ self.T_ee
        return T

    def forward_kinematics(self, q):
        T = self._transform(q)
        position = T[:-1, 3]
        rot_mat = T[:-1, :-1]
        return torch.cat([position, Rotation.from_matrix(rot_mat).as_quat()], dim=0)

    def get_jacobian(self, q):
        q.requires_grad_(True)
        y = self.forward_kinematics(q)
        jac = torch.zeros(y.shape[0], q.shape[0]).double()
        for i in range(q.shape[0]):
            jac[i] = torch.autograd.grad(y[i], q, create_graph=True, retain_graph=True)[0]
        return jac

    def get_redundancy(self, q):
        pose = self.forward_kinematics(q)
        gc_4 = torch.sign(q[3])
        p = pose[:3]
        rot_mat = Rotation.from_quat(pose[3:]).as_matrix()

        rot_0_e = rot_mat @ self.T_ee[:3, :3].T
        p_0_e = p - rot_0_e @ self.T_ee[:3, 3]

        p_0_2 = torch.tensor([0., 0., self.d_bs]).double()
        p_6_7 = torch.tensor([0., 0., self.d_wf]).double()

        # get q4 and p2_6 Shoulder-Elbow-Wrist (SEW) Plane
        p_2_6 = p_0_e - p_0_2 - rot_0_e @ p_6_7

        cosq_4_v = (torch.norm(p_2_6, dim=0) ** 2 - self.d_se ** 2 - self.d_ew ** 2) / (2 * self.d_se * self.d_ew)
        q_4_v = gc_4 * torch.acos(cosq_4_v)

        R_0_1_z = torch.tensor([0., 0., 1.]).double()
        if torch.norm(torch.cross(p_2_6, R_0_1_z)) < 1e-3:
            q_1_v = 0
        else:
            q_1_v = torch.atan2(p_2_6[1], p_2_6[0])

        phi = torch.acos(
            (self.d_se ** 2 + torch.norm(p_2_6) ** 2 - self.d_ew ** 2) / (2 * self.d_se * torch.norm(p_2_6)))

        q_2_v = torch.atan2(torch.sqrt(p_2_6[0] ** 2 + p_2_6[1] ** 2), p_2_6[2]) + gc_4 * phi

        q_3_v = torch.tensor(0.).double()

        p_0_2_v = (self._transform_i(0, q_1_v) @ self._transform_i(1, q_2_v))[:3, 3]
        p_0_4_v = (self._transform_i(0, q_1_v) @ self._transform_i(1, q_2_v) @
                   self._transform_i(2, q_3_v) @ self._transform_i(3, q_4_v))[:3, 3]

        p_0_4 = (self._transform_i(0, q[0]) @ self._transform_i(1, q[1]) @
                 self._transform_i(2, q[2]) @ self._transform_i(3, q[3]))[:3, 3]

        vec_2_4_v = (p_0_4_v - p_0_2_v) / torch.norm(p_0_4_v - p_0_2_v)
        vec_2_6_v = (p_2_6) / torch.norm(p_2_6)
        vec_2_4 = (p_0_4 - p_0_2) / torch.norm(p_0_4 - p_0_2)
        vec_2_6 = (p_2_6) / torch.norm(p_2_6)

        vec_sew_v = torch.cross(vec_2_4_v, vec_2_6_v)
        vec_sew = torch.cross(vec_2_4, vec_2_6)
        sign_psi = torch.sign(torch.cross(vec_sew_v, vec_sew).dot(p_2_6))
        psi = sign_psi * torch.acos(vec_sew_v.dot(vec_sew) / (torch.norm(vec_sew) * torch.norm(vec_sew_v)))
        return psi

    def _get_auxiliary_parameter(self, p, rot_mat, gc):
        p0_2 = torch.tensor([0., 0., self.d_bs]).double()
        p6_7 = torch.tensor([0., 0., self.d_wf]).double()
        R0_7 = rot_mat

        self.gc = np.array([gc[0], gc[0], gc[0], gc[1], gc[2], gc[2], gc[2]])

        # get q4 and p2_6 Shoulder-Elbow-Wrist (SEW) Plane
        p2_6 = p - p0_2 - rot_mat @ p6_7
        cosq4_v = (torch.norm(p2_6, dim=0) ** 2 - self.d_se ** 2 - self.d_ew ** 2) / (2 * self.d_se * self.d_ew)
        if abs(cosq4_v) > 1 + 1e-9:
            self.q4_v = None
            self.a = torch.zeros((7, 2))
            self.b = torch.zeros((7, 2))
            self.c = torch.zeros((7, 2))
            return False
        else:
            cosq4_v = torch.clamp(cosq4_v, -1., 1.)
            self.q4_v = gc[1] * torch.acos(cosq4_v)
        p2_6_norm = torch.norm(p2_6)

        q1_v = torch.atan2(p2_6[1], p2_6[0])
        phi = torch.acos((self.d_se ** 2 + p2_6_norm ** 2 - self.d_ew ** 2) / (2 * self.d_se * p2_6_norm))
        q2_v = torch.atan2(torch.sqrt(p2_6[0] ** 2 + p2_6[1] ** 2), p2_6[2]) + gc[1] * phi
        q3_v = torch.tensor(0.0)
        T_0_3_v = self._transform_i(0, q1_v) @ self._transform_i(1, q2_v) @ self._transform_i(2, q3_v)
        R_0_3_v = T_0_3_v[:3, :3]

        # cross product matrixq_true
        p2_6_hat = p2_6 / p2_6_norm
        cpm = torch.tensor([[0., -p2_6_hat[2], p2_6_hat[1]],
                            [p2_6_hat[2], 0., -p2_6_hat[0]],
                            [-p2_6_hat[1], p2_6_hat[0], 0.]])
        A_s = cpm @ R_0_3_v
        B_s = - (cpm @ cpm) @ R_0_3_v
        C_s = torch.ger(p2_6_hat, p2_6_hat) @ R_0_3_v

        T_3_4 = self._transform_i(3, self.q4_v)
        R_3_4 = T_3_4[:3, :3]

        A_w = R_3_4.T @ A_s.T @ R0_7
        B_w = R_3_4.T @ B_s.T @ R0_7
        C_w = R_3_4.T @ C_s.T @ R0_7

        # x[3, :]=0 For joint 4 is zero
        # x[1 5, 1]=0 a_d, b_d, c_d =0
        self.a = torch.zeros((7, 2))
        self.b = torch.zeros((7, 2))
        self.c = torch.zeros((7, 2))

        self.a[0, 0] = A_s[1, 1]
        self.b[0, 0] = B_s[1, 1]
        self.c[0, 0] = C_s[1, 1]
        self.a[0, 1] = A_s[0, 1]
        self.b[0, 1] = B_s[0, 1]
        self.c[0, 1] = C_s[0, 1]

        self.a[1, 0] = A_s[2, 1]
        self.b[1, 0] = B_s[2, 1]
        self.c[1, 0] = C_s[2, 1]

        self.a[2, 0] = -A_s[2, 2]
        self.b[2, 0] = -B_s[2, 2]
        self.c[2, 0] = -C_s[2, 2]
        self.a[2, 1] = -A_s[2, 0]
        self.b[2, 1] = -B_s[2, 0]
        self.c[2, 1] = -C_s[2, 0]

        self.a[4, 0] = A_w[1, 2]
        self.a[4, 1] = A_w[0, 2]
        self.b[4, 0] = B_w[1, 2]
        self.b[4, 1] = B_w[0, 2]
        self.c[4, 0] = C_w[1, 2]
        self.c[4, 1] = C_w[0, 2]

        self.a[5, 0] = A_w[2, 2]
        self.b[5, 0] = B_w[2, 2]
        self.c[5, 0] = C_w[2, 2]

        self.a[6, 0] = A_w[2, 1]
        self.a[6, 1] = -A_w[2, 0]
        self.b[6, 0] = B_w[2, 1]
        self.b[6, 1] = -B_w[2, 0]
        self.c[6, 0] = C_w[2, 1]
        self.c[6, 1] = -C_w[2, 0]
        return True

    def _transform_tcp2ee(self, pose):
        # R_0_tcp = R_0_e @ R_e_tcp -> R_0_e = R_0_tcp @ R_e_tcp.T
        rot_0_e = Rotation.from_quat(pose[3:]).as_matrix() @ self.T_ee[:3, :3].T
        # p_0_tcp = p_0_e + R_0_e * p_e_tcp
        p_0_e = pose[:3] - rot_0_e @ self.T_ee[:3, 3]
        return p_0_e, rot_0_e

    def inverse_kinematics(self, pose, psi, gc):
        q = torch.zeros((7,))
        # get auxiliary parameter a, b, c: [7, 2],

        p_0_e, rot_0_e = self._transform_tcp2ee(pose)

        ret = self._get_auxiliary_parameter(p_0_e, rot_0_e, gc)

        q[:] = np.nan
        if not ret:
            return False, q

        cos_q2 = self.a[1, 0] * torch.sin(psi) + self.b[1, 0] * torch.cos(psi) + self.c[1, 0]
        if torch.abs(cos_q2) > 1 + 1e-9:
            return False, q
        else:
            cos_q2 = torch.clamp(cos_q2, -1, 1)

        q[0] = torch.atan2(self.gc[0] * (self.a[0, 0] * torch.sin(psi) + self.b[0, 0] * torch.cos(psi) + self.c[0, 0]),
                           self.gc[0] * (self.a[0, 1] * torch.sin(psi) + self.b[0, 1] * torch.cos(psi) + self.c[0, 1]))
        q[1] = self.gc[1] * torch.acos(cos_q2)
        q[2] = torch.atan2(self.gc[2] * (self.a[2, 0] * torch.sin(psi) + self.b[2, 0] * torch.cos(psi) + self.c[2, 0]),
                           self.gc[2] * (self.a[2, 1] * torch.sin(psi) + self.b[2, 1] * torch.cos(psi) + self.c[2, 1]))

        q[3] = self.q4_v

        cos_q6 = self.a[5, 0] * torch.sin(psi) + self.b[5, 0] * torch.cos(psi) + self.c[5, 0]
        if torch.abs(cos_q6) > 1 + 1e-9:
            return False, q
        else:
            cos_q6 = torch.clamp(cos_q6, -1, 1)

        q[4] = torch.atan2(self.gc[4] * (self.a[4, 0] * torch.sin(psi) + self.b[4, 0] * torch.cos(psi) + self.c[4, 0]),
                           self.gc[4] * (self.a[4, 1] * torch.sin(psi) + self.b[4, 1] * torch.cos(psi) + self.c[4, 1]))
        q[5] = self.gc[5] * torch.acos(cos_q6)
        q[6] = torch.atan2(self.gc[6] * (self.a[6, 0] * torch.sin(psi) + self.b[6, 0] * torch.cos(psi) + self.c[6, 0]),
                           self.gc[6] * (self.a[6, 1] * torch.sin(psi) + self.b[6, 1] * torch.cos(psi) + self.c[6, 1]))
        return True, q

    def _get_psi_i(self, theta_i, joint_id):
        if joint_id in [0, 2, 4, 6]:
            a_p = self.gc[joint_id] * ((self.c[joint_id, 1] - self.b[joint_id, 1]) *
                                       torch.tan(theta_i) + (self.b[joint_id, 0] - self.c[joint_id, 0]))
            b_p = 2 * self.gc[joint_id] * (self.a[joint_id, 1] * torch.tan(theta_i) - self.a[joint_id, 0])
            c_p = self.gc[joint_id] * ((self.c[joint_id, 1] + self.b[joint_id, 1]) *
                                       torch.tan(theta_i) - (self.b[joint_id, 0] + self.c[joint_id, 0]))
            square_p = b_p ** 2 - 4 * a_p * c_p
            psi_ = []
            if square_p < 0:
                return psi_
            else:
                psi_1 = 2 * torch.atan((-b_p - torch.sqrt(square_p)) / (2 * a_p))
                theta_1 = torch.atan2(self.gc[joint_id] * (self.a[joint_id, 0] * torch.sin(psi_1) +
                                                           self.b[joint_id, 0] * torch.cos(psi_1) +
                                                           self.c[joint_id, 0]),
                                      self.gc[joint_id] * (self.a[joint_id, 1] * torch.sin(psi_1) +
                                                           self.b[joint_id, 1] * torch.cos(psi_1) +
                                                           self.c[joint_id, 1]))


                psi_2 = 2 * torch.atan((-b_p + torch.sqrt(square_p)) / (2 * a_p))
                theta_2 = torch.atan2(self.gc[joint_id] * (self.a[joint_id, 0] * torch.sin(psi_2) +
                                                           self.b[joint_id, 0] * torch.cos(psi_2) +
                                                           self.c[joint_id, 0]),
                                      self.gc[joint_id] * (self.a[joint_id, 1] * torch.sin(psi_2) +
                                                           self.b[joint_id, 1] * torch.cos(psi_2) +
                                                           self.c[joint_id, 1]))

                if torch.isclose(theta_i, theta_1) and torch.isclose(theta_i, theta_2):
                    psi_.append(psi_1)
                    psi_.append(psi_2)

                return psi_

        elif joint_id in [1, 5]:
            square_p = self.a[joint_id, 0] ** 2 + self.b[joint_id, 0] ** 2 - \
                       (self.c[joint_id, 0] - torch.cos(theta_i)) ** 2
            psi_ = []
            if square_p < 0:
                return psi_
            else:
                psi_1 = 2 * torch.atan((self.a[joint_id, 0] - torch.sqrt(square_p)) /
                                       (torch.cos(theta_i) + self.b[joint_id, 0] - self.c[joint_id, 0]))
                theta_1 = torch.acos(self.a[joint_id, 0] * torch.sin(psi_1) +
                                     self.b[joint_id, 0] * torch.cos(psi_1) + self.c[joint_id, 0])

                psi_2 = 2 * torch.atan((self.a[joint_id, 0] + torch.sqrt(square_p)) /
                                       (torch.cos(theta_i) + self.b[joint_id, 0] - self.c[joint_id, 0]))
                theta_2 = torch.acos(self.a[joint_id, 0] * torch.sin(psi_2) +
                                     self.b[joint_id, 0] * torch.cos(psi_2) +
                                     self.c[joint_id, 0])

                if torch.isclose(theta_i, theta_1) and torch.isclose(theta_i, theta_2):
                    psi_.append(psi_1)
                    psi_.append(psi_2)
                return psi_

        else:
            raise NotImplementedError()

    def _check_stationary(self, joint_id):
        # check whether contains stationary point
        if joint_id in [0, 2, 4, 6]:
            self.a_t = self.c[joint_id, 0] * self.b[joint_id, 1] - self.b[joint_id, 0] * self.c[joint_id, 1]
            self.b_t = self.a[joint_id, 0] * self.c[joint_id, 1] - self.c[joint_id, 0] * self.a[joint_id, 1]
            self.c_t = self.a[joint_id, 0] * self.b[joint_id, 1] - self.b[joint_id, 0] * self.a[joint_id, 1]

            square_t = self.a_t ** 2 + self.b_t ** 2 - self.c_t ** 2
            if torch.abs(square_t) > 0 + 1e-3:
                return 0
            else:
                return 1

    def _get_interval_i(self, joint_id):
        u_bound = []
        l_bound = []
        psi_singularity = []
        theta_l = self.joint_limits[joint_id, 0]
        theta_u = self.joint_limits[joint_id, 1]
        # Check for pivot joints
        if joint_id in [0, 2, 4, 6]:
            # check whether stationary point
            has_stationary = self._check_stationary(joint_id)
            if has_stationary:
                if torch.abs(self.b_t - self.c_t) < 1e-6:
                    psi_singularity.append(np.pi - self.singularity_eps)
                    psi_singularity.append(-np.pi + self.singularity_eps)
                    u_bound.append(np.pi - self.singularity_eps)
                    l_bound.append(-np.pi + self.singularity_eps)
                else:
                    psi_singularity.append(2 * torch.atan(self.a_t / (self.b_t - self.c_t)) - self.singularity_eps)
                    psi_singularity.append(2 * torch.atan(self.a_t / (self.b_t - self.c_t)) + self.singularity_eps)
                    u_bound.append(2 * torch.atan(self.a_t / (self.b_t - self.c_t)) - self.singularity_eps)
                    l_bound.append(2 * torch.atan(self.a_t / (self.b_t - self.c_t)) + self.singularity_eps)
            for theta_i in [theta_l, theta_u]:
                psi_i = self._get_psi_i(theta_i, joint_id)
                for psi_tmp in psi_i:
                    dtheta = self.a_t * torch.sin(psi_tmp) + self.b_t * torch.cos(psi_tmp) + self.c_t
                    if theta_i * dtheta > 0:
                        u_bound.append(psi_tmp)
                    elif theta_i * dtheta < 0:
                        l_bound.append(psi_tmp)
                    else:
                        pass

        elif joint_id in [1, 5]:
            # psi_test = torch.linspace(-np.pi, np.pi, 1000)
            # theta_test = self.gc[joint_id] * torch.acos(self.a[joint_id, 0] * torch.sin(psi_test) + self.b[joint_id, 0] * torch.cos(psi_test) + self.c[joint_id, 0])

            # plt.figure(1)
            # plt.clf()
            # plt.plot(psi_test, theta_test, c='b')
            # plt.plot(psi_test, torch.ones_like(psi_test) * theta_l, c='r')
            # plt.plot(psi_test, torch.ones_like(psi_test) * theta_u, c='r')
            # plt.plot(psi_test, torch.zeros_like(psi_test), linestyle = '--',  c='r', )
            # plt.xlim(-np.pi, np.pi)
            # plt.ylim(-np.pi, np.pi)
            # plt.draw()
            # plt.pause(0.1)

            # symmetry at theta=0, only check upper part
            theta_i = theta_u
            psi_i = self._get_psi_i(theta_i, joint_id)

            for psi_tmp in psi_i:
                dtheta = -self.a[joint_id, 0] * torch.cos(psi_tmp) + self.b[joint_id, 0] * torch.sin(psi_tmp)
                if dtheta > 0:
                    u_bound.append(psi_tmp)
                elif dtheta < 0:
                    l_bound.append(psi_tmp)
                else:
                    pass

        l_bound.append(torch.tensor(-np.pi))
        u_bound.append(torch.tensor(np.pi))

        if not (len(l_bound) == len(u_bound)):
            raise ValueError("Number of upper and lower bound are not the same")
        else:
            num_bound = len(l_bound)

        l_bound_torch = torch.cat((torch.tensor(l_bound).unsqueeze(1), torch.zeros((num_bound, 1))), dim=1)
        u_bound_torch = torch.cat((torch.tensor(u_bound).unsqueeze(1), torch.ones((num_bound, 1))), dim=1)

        bound = torch.cat((l_bound_torch, u_bound_torch), dim=0)
        bound = bound[bound[:, 0].sort(dim=0).indices]

        index = 0
        interval_ = []
        while index < bound.shape[0]:
            if bound[index, 1] == 0:
                iter_low = bound[index, 0]
                index += 1
                if bound[index, 1] == 1:
                    iter_high = bound[index, 0]
                    # interval.extend([[iter_low, 0.], [iter_high, 1.]])
                    interval_.append([iter_low, iter_high])
                    index += 1
                else:
                    continue
            else:
                index += 1
        interval_ = torch.tensor(interval_)
        return interval_, psi_singularity

    def _get_intersection_interval(self, interval_1, interval_2):
        intersections = []
        for int_1 in interval_1:
            for int_2 in interval_2:
                low = torch.tensor((int_1[0], int_2[0])).max()
                high = torch.tensor((int_1[1], int_2[1])).min()
                if high >= low:
                    intersections.append([low, high])
        return torch.tensor(intersections)

    def get_feasible_interval(self, pose, gc):
        p, rot = self._transform_tcp2ee(pose)
        if not self._get_auxiliary_parameter(p, rot, gc):
            return None
        interval_prev = torch.tensor([[-np.pi, np.pi]])
        for i in range(7):
            interval_i, singularity = self._get_interval_i(i)
            interval_prev = self._get_intersection_interval(interval_prev, interval_i)

        return interval_prev


if __name__ == "__main__":
    tol = 1e-4
    import quaternion

    for i in range(100):
        quat_test = np.random.rand(4)
        quat_test = quat_test / np.linalg.norm(quat_test)
        rot_true = quaternion.from_float_array(quat_test)
        rot_torch = Rotation.from_quat(torch.tensor(quat_test))
        if np.linalg.norm(rot_torch.as_matrix().detach().numpy() - quaternion.as_rotation_matrix(rot_true)) > 1e-8:
            print("Rotation to Matrix error")

        if np.linalg.norm(rot_torch.from_matrix(rot_torch.as_matrix()).as_quat().detach().numpy() - quat_test) > 1e-8:
            print("Matrix to Quaternion Error")

        if np.linalg.norm(rot_torch.as_rotvec().detach().numpy() - quaternion.as_rotation_vector(rot_true)) > 1e-8:
            print("Quaternion to Rotation Vector Error")

        if np.linalg.norm(rot_torch.from_rotvec(rot_torch.as_rotvec()).as_quat().detach().numpy() - quat_test) > 1e-8:
            print("Rotation Vector to Quaternion Error")

    for i in range(10):
        yzy_test = torch.rand(3) * torch.tensor([2 * np.pi, np.pi, 2 * np.pi]) - torch.tensor([np.pi, 0, np.pi])
        kine_test = KinematicsTorch(tcp_pos=torch.rand(3),
                                    tcp_quat=Rotation.from_euler_yzy(yzy_test).as_quat())
        q_test = torch.deg2rad(torch.rand((7,)) * 360 - 180).requires_grad_()
        pose_test = kine_test.forward_kinematics(q_test)

        psi_test = torch.rand(1)
        gc_test = torch.tensor([1, 1, 1])
        res, q_inv = kine_test.inverse_kinematics(pose_test, psi_test, gc_test)
        if res:
            pose_inv = kine_test.forward_kinematics(q_inv)
            position_error = torch.norm(pose_test[:3] - pose_inv[:3])
            if position_error > tol:
                print("Position Error:", position_error.item())
            rotation_error = torch.norm(pose_test[3:] - pose_inv[3:])
            if rotation_error > tol:
                print("Rotation Error", rotation_error.item())

        # interval_test = kine_test.get_feasible_interval(pose_test.requires_grad_(True), gc_test)
        # print("Interval:", interval_test)

        jac = kine_test.get_jacobian(q_test)
        dq_test = torch.ones(7).double() * 1e-3
        pose_dq_test = kine_test.forward_kinematics(dq_test + q_test) - pose_test
        pose_jac_test = jac @ dq_test
        pose_jac_error = torch.norm(pose_dq_test - pose_jac_test)
        if pose_jac_error > tol:
            print("Jacobian Error:", pose_jac_error)

    # Test for calculating redundancy
    for i in range(10):
        # yzy_test = torch.rand(3) * torch.tensor([2 * np.pi, np.pi, 2 * np.pi]) - torch.tensor([np.pi, 0, np.pi])
        # kine_test = KinematicsTorch(tcp_pos=torch.rand(3),
        #                             tcp_quat=Rotation.from_euler_yzy(yzy_test).as_quat())
        # q_test = torch.rand(7) * (kine_test.joint_limits[:, 1]) * 2 + kine_test.joint_limits[:, 0]

        yzy_test = torch.rand(3) * torch.tensor([2 * np.pi, np.pi, 2 * np.pi]) - torch.tensor([np.pi, 0, np.pi])
        kine_test = KinematicsTorch(tcp_pos=torch.tensor([0.1, 0.2, 0.5]),
                                tcp_quat=torch.tensor([1., 0.5, 0.1, 0.6])/torch.norm(torch.tensor([1., 0.5, 0.1, 0.6])))
        q_test = torch.tensor([-0.1, 0.2, -0.3, -0.4, -0.5, 0.6, -0.7])

        pose_test = kine_test.forward_kinematics(q_test)
        gc = torch.sign(q_test[[1, 3, 5]])
        psi_test = kine_test.get_redundancy(q_test)
        res, q_test_out = kine_test.inverse_kinematics(pose_test, psi_test, gc)

        q_error = torch.norm(q_test - q_test_out)
        if q_error > tol:
            print("psi is not correct")

    print("Finish!")
