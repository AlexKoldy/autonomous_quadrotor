import scipy.optimize
import numpy as np
from scipy.spatial.transform import Rotation


class SE3Control(object):
    """ """

    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass = quad_params["mass"]  # kg
        self.Ixx = quad_params["Ixx"]  # kg*m^2
        self.Iyy = quad_params["Iyy"]  # kg*m^2
        self.Izz = quad_params["Izz"]  # kg*m^2
        self.arm_length = quad_params["arm_length"]  # meters
        self.rotor_speed_min = quad_params["rotor_speed_min"]  # rad/s
        self.rotor_speed_max = quad_params["rotor_speed_max"]  # rad/s
        self.k_thrust = quad_params["k_thrust"]  # N/(rad/s)**2
        self.k_drag = quad_params["k_drag"]  # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz]))  # kg*m^2
        self.g = 9.81  # m/s^2

        # STUDENT CODE HERE
        # self.k_M = (1.5 * 10**-9) / ((2 * np.pi / 60) ** 2)  # [Nm/(rad/s)^2]
        # self.k_F = (6.11 * 10**-8) / ((2 * np.pi / 60) ** 2)  # [N/(rad/s)^2]
        self.gamma = self.k_drag / self.k_thrust
        # self.gamma = self.k_M / self.k_F

        self.K_p = np.array(
            [
                [6, 0, 0],
                [0, 6, 0],
                [0, 0, 10],
            ]
        )  # 4.375, 2500
        self.K_d = np.array(
            [
                [4.5, 0, 0],
                [0, 4.5, 0],
                [0, 0, 4.4],
            ]
        )  # 4.00001, 700

        # 1000, 100
        # for psi: 1000, 300
        self.k_p_phi = 1250
        self.k_d_phi = 130
        self.k_p_theta = 1250
        self.k_d_theta = 130
        self.k_p_psi = 1250
        self.k_d_psi = 130

    def update(self, t, state, flat_output):
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        r = state["x"]
        r_dot = state["v"]
        # rotation = Rotation.from_quat(state["q"]).as_euler("zxy")
        # psi, phi, theta = rotation
        rotation = Rotation.from_quat(state["q"]).as_matrix()

        psi_dot = state["w"]

        r_traj = flat_output["x"]
        r_dot_traj = flat_output["x_dot"]
        r_ddot_traj = flat_output["x_ddot"]

        psi_traj = flat_output["yaw"]
        psi_dot_traj = flat_output["yaw_dot"]

        r_ddot_des = (
            r_ddot_traj - self.K_d @ (r_dot - r_dot_traj) - self.K_p @ (r - r_traj)
        ).flatten()

        e_psi_dot = psi_dot - psi_dot_traj
        # print(e_psi_dot.shape)

        m = self.mass
        g = self.g
        F_g = np.array([0, 0, m * g])

        F_des = self.mass * r_ddot_des + F_g
        print(F_des)
        b_3 = rotation @ np.array([0, 0, 1])
        u_1 = np.array([b_3 @ F_des])

        b_3_des = F_des / np.linalg.norm(F_des)
        a_psi = np.array([np.cos(psi_traj), np.sin(psi_traj), 0])
        b_2_cross = np.cross(b_3_des, a_psi)
        b_2_des = b_2_cross / np.linalg.norm(b_2_cross)
        rotation_des = np.hstack(
            (
                np.reshape(np.cross(b_2_des, b_3_des), (3, 1)),
                np.reshape(b_2_des, (3, 1)),
                np.reshape(b_3_des, (3, 1)),
            )
        )

        R = 0.5 * (rotation_des.T @ rotation - rotation.T @ rotation_des)
        e_R = np.array([R[2, 1], R[0, 2], R[1, 0]])

        # u_2 = np.array([[self.Ixx, 0, 0], [0, self.Iyy, 0], [0, 0, self.Izz],]) @ (
        #     -np.array(
        #         [[self.k_p_phi, 0, 0], [0, self.k_p_theta, 0], [0, 0, self.k_p_psi]]
        #     )
        #     @ e_R
        #     - np.array(
        #         [[self.k_d_phi, 0, 0], [0, self.k_d_theta, 0], [0, 0, self.k_d_psi]]
        #     )
        #     @ e_psi_dot
        # )

        u_2 = self.inertia @ (
            -np.array(
                [[self.k_p_phi, 0, 0], [0, self.k_p_theta, 0], [0, 0, self.k_p_psi]]
            )
            @ e_R
            - np.array(
                [[self.k_d_phi, 0, 0], [0, self.k_d_theta, 0], [0, 0, self.k_d_psi]]
            )
            @ e_psi_dot
        )

        u = np.vstack((np.reshape(u_1, (1, 1)), np.reshape(u_2, (3, 1))))
        F = np.linalg.lstsq(
            (
                np.array(
                    [
                        [1, 1, 1, 1],
                        [0, self.arm_length, 0, -self.arm_length],
                        [-self.arm_length, 0, self.arm_length, 0],
                        [self.gamma, -self.gamma, self.gamma, -self.gamma],
                    ]
                )
            ),
            u,
            rcond=None,
        )[0]

        cmd_motor_speeds = (
            np.sign(F).flatten()
            * np.sqrt(np.abs(F.flatten()) / self.k_thrust).flatten()
        )

        control_input = {
            "cmd_motor_speeds": cmd_motor_speeds,
            "cmd_thrust": u_1,
            "cmd_moment": u_2,
            "cmd_q": cmd_q,
        }
        return control_input

    def update1(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE
        r = state["x"]
        r_dot = state["v"]
        rotation = Rotation.from_quat(state["q"]).as_euler("zxy")
        psi, phi, theta = rotation
        phi_dot, theta_dot, psi_dot = state["w"]

        r_traj = flat_output["x"]
        r_dot_traj = flat_output["x_dot"]
        r_ddot_traj = flat_output["x_ddot"]

        psi_traj = flat_output["yaw"]
        psi_dot_traj = flat_output["yaw_dot"]

        r_ddot_des = (
            r_ddot_traj - self.K_d @ (r_dot - r_dot_traj) - self.K_p @ (r - r_traj)
        )
        # print(f"position error: {np.linalg.norm(r - r_traj)} | velocity: {np.linalg.norm(r_dot)}")
        e = r - r_traj
        # print(
        #     f"x-error: {e[0]} | y-error: {e[1]} | z-error: {e[2]} | error norm: {np.linalg.norm(e)}"
        # )

        theta_des = (
            r_ddot_des[1] * np.sin(psi_traj) + r_ddot_des[0] * np.cos(psi_traj)
        ) / (self.g * np.sin(psi_traj) ** 2 + self.g * np.cos(psi_traj) ** 2)

        phi_des = (
            r_ddot_des[0] * np.sin(psi_traj) - r_ddot_des[1] * np.cos(psi_traj)
        ) / (self.g * np.sin(psi_traj) ** 2 + self.g * np.cos(psi_traj) ** 2)

        while theta_des > np.pi:
            theta_des -= 2 * np.pi

        while theta_des < -np.pi:
            theta_des += 2 * np.pi

        while phi_des > np.pi:
            phi_des -= 2 * np.pi

        while phi_des < -np.pi:
            phi_des += 2 * np.pi

        p_des = 0
        q_des = 0

        u_1 = self.mass * (r_ddot_des[2] + self.g)

        u_2 = np.array(
            [
                [self.Ixx, 0, 0],
                [0, self.Iyy, 0],
                [0, 0, self.Izz],
            ]
        ) @ np.array(
            [
                [-self.k_p_phi * (phi - phi_des) - self.k_d_phi * (phi_dot - p_des)],
                [
                    -self.k_p_theta * (theta - theta_des)
                    - self.k_d_theta * (theta_dot - q_des)
                ],
                [
                    -self.k_p_psi * (psi - psi_traj)
                    - self.k_d_psi * (psi_dot - psi_dot_traj)
                ],
            ]
        )

        u = np.vstack((u_1, u_2))
        F = np.linalg.lstsq(
            (
                np.array(
                    [
                        [1, 1, 1, 1],
                        [0, self.arm_length, 0, -self.arm_length],
                        [-self.arm_length, 0, self.arm_length, 0],
                        [self.gamma, -self.gamma, self.gamma, -self.gamma],
                    ]
                )
            ),
            u,
            rcond=None,
        )[0]

        # print(f"Error: {r_traj[2] - r[2]} | Thrust: {u_1}")

        cmd_motor_speeds = (
            np.sign(F).flatten()
            * np.sqrt(np.abs(F.flatten()) / self.k_thrust).flatten()
        )

        # cmd_thrust = r_ddot_des[0] #(theta) * 180 / np.pi
        # cmd_moment = r_ddot_des[1] * np.ones((3,)) #((theta_des) * 180 / np.pi) * np.ones((3,))
        # cmd_moment[2] = r_ddot_des[2]#(theta - theta_des)  * 180 / np.pi
        cmd_thrust = phi_des
        # cmd_moment = (-self.K_d @ (r_dot - r_dot_traj))[0] * np.ones((3, ))
        cmd_moment[0] = phi
        cmd_moment[1] = phi_des
        cmd_moment[2] = theta_des * 180 / np.pi

        control_input = {
            "cmd_motor_speeds": cmd_motor_speeds,
            "cmd_thrust": cmd_thrust,
            "cmd_moment": cmd_moment,
            "cmd_q": cmd_q,
        }
        return control_input
