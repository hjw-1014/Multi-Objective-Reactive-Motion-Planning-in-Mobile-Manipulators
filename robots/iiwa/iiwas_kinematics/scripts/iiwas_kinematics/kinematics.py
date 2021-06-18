import numpy as np
import quaternion

# import matplotlib.pyplot as plt

IIWA_JOINT_MIN_LIMITS = np.deg2rad(np.array([-170., -120., -170., -120., -170., -120., -175.])).tolist()
IIWA_JOINT_MAX_LIMITS = np.deg2rad(np.array([170., 120., 170., 120., 170., 120., 175.])).tolist()


class Kinematics:
    def __init__(self, tcp_pos=None, tcp_quat=None):
        self.d_bs = 0.36
        self.d_se = 0.42
        self.d_ew = 0.4
        self.d_wf = 0.151

        # DH_paramters a_i, alpha_i, d_i
        self.dh_a = np.array([0., 0., 0., 0., 0., 0., 0.])
        self.dh_alpha = np.array([-np.pi / 2, np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0.])
        self.dh_d = np.array([self.d_bs, 0., self.d_se, 0., self.d_ew, 0., self.d_wf])

        self.joint_limits = np.vstack([IIWA_JOINT_MIN_LIMITS, IIWA_JOINT_MAX_LIMITS]).T

        self.joint_vel_limits = np.array([[-85., 85.],
                                          [-85., 85.],
                                          [-100., 100.],
                                          [-75., 75.],
                                          [-130., 130.],
                                          [-135., 135.],
                                          [-135., 135.]]) / 180. * np.pi

        self.singularity_eps = 0.1

        self.T_ee = np.eye(4)
        if tcp_pos is not None:
            self.T_ee[:3, 3] = tcp_pos
        if tcp_quat is not None:
            self.T_ee[:3, :3] = quaternion.as_rotation_matrix(quaternion.from_float_array(tcp_quat))

    def _transform_i(self, index, theta_i):
        alpha_i = self.dh_alpha[index]
        a_i = self.dh_a[index]
        d_i = self.dh_d[index]
        T_i = np.array([[np.cos(theta_i), -np.sin(theta_i) * np.cos(alpha_i),
                         np.sin(theta_i) * np.sin(alpha_i), a_i * np.cos(theta_i)],
                        [np.sin(theta_i), np.cos(theta_i) * np.cos(alpha_i),
                         -np.cos(theta_i) * np.sin(alpha_i), a_i * np.sin(theta_i)],
                        [0., np.sin(alpha_i), np.cos(alpha_i), d_i],
                        [0., 0., 0., 1.]])
        return T_i

    def _transform(self, q):
        T = np.eye(4)
        for i, theta_i in enumerate(q):
            T_i = self._transform_i(i, theta_i)
            T = T @ T_i
        T = T @ self.T_ee
        return T

    def forward_kinematics(self, q):
        T = self._transform(q)
        position = T[:-1, 3]
        rotation = quaternion.from_rotation_matrix(T[:-1, :-1])
        return np.concatenate((position, rotation.components))

    def _get_auxiliary_parameter(self, p, rot, gc):
        p0_2 = np.array([0., 0., self.d_bs])
        p6_7 = np.array([0., 0., self.d_wf])
        R0_7 = rot

        # get q4 and p2_6 Shoulder-Elbow-Wrist (SEW) Plane
        p2_6 = p - p0_2 - np.dot(rot, p6_7)
        cosq4_v = (np.linalg.norm(p2_6) ** 2 - self.d_se ** 2 - self.d_ew ** 2) / (2 * self.d_se * self.d_ew)
        if abs(cosq4_v) > 1 + 1e-9:
            self.q4_v = None
            a = np.zeros((7, 2))
            b = np.zeros((7, 2))
            c = np.zeros((7, 2))
            return a, b, c, None
        else:
            cosq4_v = np.clip(cosq4_v, -1., 1.)
            self.q4_v = gc[1] * np.arccos(cosq4_v)
        p2_6_norm = np.linalg.norm(p2_6)

        q1_v = np.arctan2(p2_6[1], p2_6[0])
        phi = np.arccos((self.d_se ** 2 + p2_6_norm ** 2 - self.d_ew ** 2) / (2 * self.d_se * p2_6_norm))
        q2_v = np.arctan2(np.sqrt(p2_6[0] ** 2 + p2_6[1] ** 2), p2_6[2]) + gc[1] * phi
        q3_v = 0.0
        T_0_3_v = self._transform_i(0, q1_v) @ self._transform_i(1, q2_v) @ self._transform_i(2, q3_v)
        R_0_3_v = T_0_3_v[:3, :3]

        # cross product matrixq_true
        p2_6_hat = p2_6 / p2_6_norm
        cpm = np.array([[0., -p2_6_hat[2], p2_6_hat[1]],
                        [p2_6_hat[2], 0., -p2_6_hat[0]],
                        [-p2_6_hat[1], p2_6_hat[0], 0.]])
        A_s = cpm @ R_0_3_v
        B_s = - (cpm @ cpm) @ R_0_3_v
        C_s = np.outer(p2_6_hat, p2_6_hat) @ R_0_3_v

        T_3_4 = self._transform_i(3, self.q4_v)
        R_3_4 = T_3_4[:3, :3]

        A_w = R_3_4.T @ A_s.T @ R0_7
        B_w = R_3_4.T @ B_s.T @ R0_7
        C_w = R_3_4.T @ C_s.T @ R0_7

        # x[3, :]=0 For joint 4 is zero
        # x[1 5, 1]=0 a_d, b_d, c_d =0
        self.a = np.zeros((7, 2))
        self.b = np.zeros((7, 2))
        self.c = np.zeros((7, 2))

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

        self.gc = np.array([gc[0], gc[0], gc[0], gc[1], gc[2], gc[2], gc[2]])

    def inverse_kinematics(self, pose, psi, gc):
        q = np.zeros((7,))

        rot_0_e = quaternion.as_rotation_matrix(quaternion.from_float_array(pose[3:]))
        p_0_e = pose[:3] - rot_0_e @ self.T_ee[:3, 3]

        # get auxiliary parameter a, b, c: [7, 2],
        self._get_auxiliary_parameter(p_0_e, rot_0_e, gc)

        q[:] = np.nan
        if self.q4_v is None:
            return False, q

        cosq2 = self.a[1, 0] * np.sin(psi) + self.b[1, 0] * np.cos(psi) + self.c[1, 0]
        if abs(cosq2) > 1 + 1e-9:
            return False, q
        else:
            cosq2 = np.clip(cosq2, -1, 1)

        q[0] = np.arctan2(self.gc[0] * (self.a[0, 0] * np.sin(psi) + self.b[0, 0] * np.cos(psi) + self.c[0, 0]),
                          self.gc[0] * (self.a[0, 1] * np.sin(psi) + self.b[0, 1] * np.cos(psi) + self.c[0, 1]))
        q[1] = self.gc[1] * np.arccos(cosq2)
        q[2] = np.arctan2(self.gc[2] * (self.a[2, 0] * np.sin(psi) + self.b[2, 0] * np.cos(psi) + self.c[2, 0]),
                          self.gc[2] * (self.a[2, 1] * np.sin(psi) + self.b[2, 1] * np.cos(psi) + self.c[2, 1]))

        q[3] = self.q4_v

        cosq6 = self.a[5, 0] * np.sin(psi) + self.b[5, 0] * np.cos(psi) + self.c[5, 0]
        if abs(cosq6) > 1 + 1e-9:
            return False, q
        else:
            cosq6 = np.clip(cosq6, -1, 1)

        q[4] = np.arctan2(self.gc[4] * (self.a[4, 0] * np.sin(psi) + self.b[4, 0] * np.cos(psi) + self.c[4, 0]),
                          self.gc[4] * (self.a[4, 1] * np.sin(psi) + self.b[4, 1] * np.cos(psi) + self.c[4, 1]))
        q[5] = self.gc[5] * np.arccos(cosq6)
        q[6] = np.arctan2(self.gc[6] * (self.a[6, 0] * np.sin(psi) + self.b[6, 0] * np.cos(psi) + self.c[6, 0]),
                          self.gc[6] * (self.a[6, 1] * np.sin(psi) + self.b[6, 1] * np.cos(psi) + self.c[6, 1]))
        return True, q

    def _get_psi_i(self, theta_i, joint_id):
        if joint_id in [0, 2, 4, 6]:
            a_p = self.gc[joint_id] * ((self.c[joint_id, 1] - self.b[joint_id, 1]) *
                                       np.tan(theta_i) + (self.b[joint_id, 0] - self.c[joint_id, 0]))
            b_p = 2 * self.gc[joint_id] * (self.a[joint_id, 1] * np.tan(theta_i) - self.a[joint_id, 0])
            c_p = self.gc[joint_id] * ((self.c[joint_id, 1] + self.b[joint_id, 1]) *
                                       np.tan(theta_i) - (self.b[joint_id, 0] + self.c[joint_id, 0]))
            square_p = b_p ** 2 - 4 * a_p * c_p
            psi = []
            if square_p < 0:
                return psi

            else:
                psi_1 = 2 * np.arctan((-b_p - np.sqrt(square_p)) / (2 * a_p))
                theta_1 = np.arctan2(self.gc[joint_id] * (
                        self.a[joint_id, 0] * np.sin(psi_1) + self.b[joint_id, 0] * np.cos(psi_1) + self.c[
                    joint_id, 0]),
                                     self.gc[joint_id] * (
                                             self.a[joint_id, 1] * np.sin(psi_1) + self.b[joint_id, 1] * np.cos(
                                         psi_1) + self.c[joint_id, 1]))
                if np.isclose(theta_i, theta_1):
                    psi.append(psi_1)

                psi_2 = 2 * np.arctan((-b_p + np.sqrt(square_p)) / (2 * a_p))
                theta_2 = np.arctan2(self.gc[joint_id] * (
                        self.a[joint_id, 0] * np.sin(psi_2) + self.b[joint_id, 0] * np.cos(psi_2) + self.c[
                    joint_id, 0]),
                                     self.gc[joint_id] * (
                                             self.a[joint_id, 1] * np.sin(psi_2) + self.b[joint_id, 1] * np.cos(
                                         psi_2) + self.c[joint_id, 1]))
                if np.isclose(theta_i, theta_2):
                    psi.append(psi_2)
                return psi

        elif joint_id in [1, 5]:
            square_p = self.a[joint_id, 0] ** 2 + self.b[joint_id, 0] ** 2 - \
                       (self.c[joint_id, 0] - np.cos(theta_i)) ** 2
            psi = []
            if square_p < 0:
                return psi
            else:
                psi_1 = 2 * np.arctan((self.a[joint_id, 0] - np.sqrt(square_p)) /
                                      (np.cos(theta_i) + self.b[joint_id, 0] - self.c[joint_id, 0]))
                theta_1 = np.arccos(
                    self.a[joint_id, 0] * np.sin(psi_1) + self.b[joint_id, 0] * np.cos(psi_1) + self.c[joint_id, 0])
                if np.isclose(theta_i, theta_1):
                    psi.append(psi_1)

                psi_2 = 2 * np.arctan((self.a[joint_id, 0] + np.sqrt(square_p)) /
                                      (np.cos(theta_i) + self.b[joint_id, 0] - self.c[joint_id, 0]))
                theta_2 = np.arccos(
                    self.a[joint_id, 0] * np.sin(psi_2) + self.b[joint_id, 0] * np.cos(psi_2) + self.c[joint_id, 0])
                if np.isclose(theta_i, theta_2):
                    psi.append(psi_2)
                return psi

        else:
            raise NotImplementedError()

    def _check_stationary(self, joint_id):
        # check whether contains stationary point
        # self.a_t = self.gc[joint_id] * (
        #             self.c[joint_id, 0] * self.b[joint_id, 1] - self.b[joint_id, 0] * self.c[joint_id, 1])
        # self.b_t = self.gc[joint_id] * (
        #             self.a[joint_id, 0] * self.c[joint_id, 1] - self.c[joint_id, 0] * self.a[joint_id, 1])
        # self.c_t = self.gc[joint_id] * (
        #             self.a[joint_id, 0] * self.b[joint_id, 1] - self.b[joint_id, 0] * self.a[joint_id, 1])
        if joint_id in [0, 2, 4, 6]:
            self.a_t = self.c[joint_id, 0] * self.b[joint_id, 1] - self.b[joint_id, 0] * self.c[joint_id, 1]
            self.b_t = self.a[joint_id, 0] * self.c[joint_id, 1] - self.c[joint_id, 0] * self.a[joint_id, 1]
            self.c_t = self.a[joint_id, 0] * self.b[joint_id, 1] - self.b[joint_id, 0] * self.a[joint_id, 1]

            square_t = self.a_t ** 2 + self.b_t ** 2 - self.c_t ** 2
            if abs(square_t) > 0 + 1e-3:
                return 0
            else:
                return 1

    def get_interval_i(self, joint_id):
        u_bound = []
        l_bound = []
        psi_singularity = []
        theta_l = self.joint_limits[joint_id, 0]
        theta_u = self.joint_limits[joint_id, 1]
        # Check for pivot joints
        if joint_id in [0, 2, 4, 6]:
            # psi_test = np.linspace(-np.pi, np.pi, 1000)
            # theta_test = np.arctan2(self.gc[joint_id] * (self.a[joint_id, 0] * np.sin(psi_test) + self.b[joint_id, 0] * np.cos(psi_test) + self.c[joint_id, 0]),
            #                         self.gc[joint_id] * (self.a[joint_id, 1] * np.sin(psi_test) + self.b[joint_id, 1] * np.cos(psi_test) + self.c[joint_id, 1]))
            # plt.figure(1)
            # plt.clf()
            # plt.plot(psi_test, theta_test, c='b')
            # plt.plot(psi_test, np.ones_like(psi_test)* theta_l, c='r')
            # plt.plot(psi_test, np.ones_like(psi_test)* theta_u, c='r')
            # plt.xlim(-np.pi, np.pi)
            # plt.ylim(-np.pi, np.pi)
            # plt.draw()
            # plt.pause(0.1)

            # check whether stationary point
            has_stationary = self._check_stationary(joint_id)
            if has_stationary:
                if abs(self.b_t - self.c_t) < 1e-6:
                    psi_singularity.append(np.pi - self.singularity_eps)
                    psi_singularity.append(-np.pi + self.singularity_eps)
                    u_bound.append(np.pi - self.singularity_eps)
                    l_bound.append(-np.pi + self.singularity_eps)
                else:
                    psi_singularity.append(2 * np.arctan(self.a_t / (self.b_t - self.c_t)) - self.singularity_eps)
                    psi_singularity.append(2 * np.arctan(self.a_t / (self.b_t - self.c_t)) + self.singularity_eps)
                    u_bound.append(2 * np.arctan(self.a_t / (self.b_t - self.c_t)) - self.singularity_eps)
                    l_bound.append(2 * np.arctan(self.a_t / (self.b_t - self.c_t)) + self.singularity_eps)
            for theta_i in [theta_l, theta_u]:
                psi_i = self._get_psi_i(theta_i, joint_id)
                for psi_tmp in psi_i:
                    dtheta = self.a_t * np.sin(psi_tmp) + self.b_t * np.cos(psi_tmp) + self.c_t
                    if theta_i * dtheta > 0:
                        u_bound.append(psi_tmp)
                    else:
                        l_bound.append(psi_tmp)

        elif joint_id in [1, 5]:
            # psi_test = np.linspace(-np.pi, np.pi, 1000)
            # theta_test = self.gc[joint_id] * np.arccos(self.a[joint_id, 0] * np.sin(psi_test) + self.b[joint_id, 0] * np.cos(psi_test) + self.c[joint_id, 0])

            # plt.figure(1)
            # plt.clf()
            # plt.plot(psi_test, theta_test, c='b')
            # plt.plot(psi_test, np.ones_like(psi_test) * theta_l, c='r')
            # plt.plot(psi_test, np.ones_like(psi_test) * theta_u, c='r')
            # plt.plot(psi_test, np.zeros_like(psi_test), linestyle = '--',  c='r', )
            # plt.xlim(-np.pi, np.pi)
            # plt.ylim(-np.pi, np.pi)
            # plt.draw()
            # plt.pause(0.1)

            # symmetry at theta=0, only check upper part
            theta_i = theta_u
            psi_i = self._get_psi_i(theta_i, joint_id)

            for psi_tmp in psi_i:
                dtheta = -self.a[joint_id, 0] * np.cos(psi_tmp) + self.b[joint_id, 0] * np.sin(psi_tmp)
                if dtheta > 0:
                    u_bound.append(psi_tmp)
                else:
                    l_bound.append(psi_tmp)

        l_bound.append(-np.pi)
        u_bound.append(np.pi)

        if len(l_bound) is not len(u_bound):
            raise ValueError("Number of upper and lower bound are not the same")
        else:
            num_bound = len(l_bound)

        l_bound_np = np.concatenate((np.array(l_bound)[:, np.newaxis], np.zeros((num_bound, 1))), axis=1)
        u_bound_np = np.concatenate((np.array(u_bound)[:, np.newaxis], np.ones((num_bound, 1))), axis=1)

        bound = np.vstack((l_bound_np, u_bound_np))
        bound = bound[bound[:, 0].argsort()]

        index = 0
        interval = []
        while index < bound.shape[0]:
            if bound[index, 1] == 0:
                iter_low = bound[index, 0]
                index += 1
                if bound[index, 1] == 1:
                    iter_high = bound[index, 0]
                    # interval.extend([[iter_low, 0.], [iter_high, 1.]])
                    interval.append([iter_low, iter_high])
                    index += 1
                else:
                    continue
            else:
                index += 1
        interval = np.array(interval)
        return interval, psi_singularity

    def _get_intersection_interval(self, interval_1, interval_2):
        intersections = []
        for int_1 in interval_1:
            for int_2 in interval_2:
                low = np.maximum(int_1[0], int_2[0])
                high = np.minimum(int_1[1], int_2[1])
                if high >= low:
                    intersections.append([low, high])
        return np.array(intersections)

    def get_feasible_interval(self, p, rot, gc):
        self._get_auxiliary_parameter(p, rot, gc)
        interval_prev = np.array([[-np.pi, np.pi]])
        # plt.figure(1)
        # color = ['tab:blue', 'tab:orange', 'tab:green','tab:purple', 'tab:pink', 'tab:grey', 'tab:olive']
        for i in range(7):
            interval_i, singularity = self.get_interval_i(i)
            interval_prev = self._get_intersection_interval(interval_prev, interval_i)

            # if len(singularity) > 0:
            #     plt.scatter(singularity, [i+1, i+1], color='tab:red', marker='x', s=150, zorder=2)
            # for ii in interval_i:
            #     plt.plot(ii, [i+1, i+1], color=color[i], lw = 10, zorder=1)

        # print(interval_prev)

        # for ii in interval_prev:
        #     plt.plot(ii, [0, 0], color='tab:red', lw=10, label='Total')
        # plt.ylabel("Joint Index")
        # plt.xlabel("Psi")
        # plt.show()

        return interval_prev
