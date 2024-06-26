import numpy as np
from scipy import io
from typing import List, Tuple
import math


class UnscentedKalmanFilter:
    def __init__(
        self, Q: np.ndarray, R: np.ndarray, include_extra_sigma_point: bool = True
    ) -> None:
        # Covariances
        self.Q = Q
        self.R = R

        # Parameter to include the mean when generating sigma points
        self.include_extra_sigma_point = include_extra_sigma_point

        # For storing and usage of propagated sigma points
        self.propagated_sigma_points = {
            "dynamics": None,
            "measurement": None,
        }

    def f(
        self, x: Tuple[Quaternion, np.ndarray], dt: float
    ) -> Tuple[Quaternion, np.ndarray]:
        """"""
        dq = Quaternion()
        dq.from_axis_angle(x[1] * dt)
        return (
            x[0] * dq,
            x[1],
        )

    def g(self, x: Tuple[Quaternion, np.ndarray]) -> Tuple[Quaternion, np.ndarray]:
        """"""
        return (x[0].inv() * Quaternion(0, [0, 0, 1]) * x[0], x[1])

    def generate_sigma_points(
        self,
        mu: Tuple[Quaternion, np.ndarray],
        Sigma: np.ndarray,
    ) -> List[Tuple[Quaternion, np.ndarray]]:
        # Get number of columns in Sigma
        n = Sigma.shape[1] / 2

        # Find square root of the covariance
        S_q = np.linalg.cholesky(Sigma[0:3, 0:3])
        S_omega = np.linalg.cholesky(Sigma[3:6, 3:6])

        # Generate 2n sigma points by multiplying each column
        # by +-n^(1/2). If the 'self.include_extra_sigma_point'
        # is true, add the mean as an additional sigma point
        sigma_points = []
        if self.include_extra_sigma_point:
            sigma_points.append(mu)

        for col_q, col_omega in zip(
            (np.hstack((np.sqrt(2 * n) * S_q, -np.sqrt(2 * n) * S_q))).T,
            (np.hstack((np.sqrt(2 * n) * S_omega, -np.sqrt(2 * n) * S_omega))).T,
        ):
            # Convert each element associated with orientation to a
            # 'vector quaternion'
            s = Quaternion()
            s.from_axis_angle(col_q)

            # 'Add' the orientation portion of the column to the orientation
            # portion of the mean (via quaternion multiplication)
            sigma_point_q = mu[0] * s
            sigma_point_omega = mu[1] + col_omega
            sigma_points.append((sigma_point_q, sigma_point_omega))

        return sigma_points

    def calc_quaternion_mean_and_covariance(
        self,
        quaternions: List[Quaternion],
        mu_q: Quaternion,
        threshold: float = 0.001,
        max_iterations: int = 1000,
    ) -> Tuple[Quaternion, np.ndarray]:
        """"""
        # Set up error vector matrix
        n = len(quaternions)
        E = np.zeros((3, n))

        # Set up gradient descent variables and run gradient descent
        q_bar = mu_q
        error = np.inf

        iteration = 0
        while error > threshold and iteration < max_iterations:
            # Set up quaternion covariance matrix
            Sigma_q = np.zeros((3, 3))

            for i, q in enumerate(quaternions):
                # Compute relative rotation between given quaternion
                # and the quaternion mean, 'e' is the error vector here
                e = q * q_bar.inv()
                e.normalize()

                # Fill in error vector matrix
                E[:, i] = e.axis_angle()

                # Calculate the covariance from this axis-angle representation of the
                # error vector and add it to Sigma_q
                Sigma_q += 1 / n * np.outer(E[:, i], E[:, i])

            # Calculate the standard mean of error vectors and convert
            # to quaternion space
            e_bar = 1 / n * np.sum(E, axis=1)
            q_e = Quaternion()
            q_e.from_axis_angle(e_bar)

            # Calculate new quaternion mean
            q_bar = q_e * q_bar
            q_bar.normalize()

            # Update norm of error
            error = np.linalg.norm(e_bar)

            # Increase iteration variable
            iteration += 1

        self.E = E

        return (q_bar, Sigma_q)

    def calc_kalman_gain(
        self, Sigma_xy: np.ndarray, Sigma_yy: np.ndarray
    ) -> np.ndarray:
        """"""
        return Sigma_xy @ np.linalg.inv(Sigma_yy)

    def calc_innovation(self, y: np.ndarray, y_hat: np.ndarray) -> np.ndarray:
        """"""
        return y - y_hat

    def predict(
        self,
        mu_k_given_k: Tuple[Quaternion, np.ndarray],
        Sigma_k_given_k: np.ndarray,
        dt: float,
    ) -> Tuple[Tuple[Quaternion, np.ndarray], np.ndarray]:
        """"""
        # Generate sigma points. Remember to add covariance 'R' multiplied
        # by 'dt' to Sigma_{k|k} before calculation to account for varying
        # timesteps
        sigma_points = self.generate_sigma_points(
            mu_k_given_k, Sigma_k_given_k + self.R * dt
        )

        # Propogate dynamics for each sigma point
        self.propagated_sigma_points["dynamics"] = []
        for sigma_point in sigma_points:
            # tmp_sp = (sigma_point[0], np.array([-0.00676288, 0.00676288, 0.0084536]))
            # q_propagated, omega_propagated = self.f(tmp_sp, dt)
            # print(q_propagated.q)
            q_propagated, omega_propagated = self.f(sigma_point, dt)
            self.propagated_sigma_points["dynamics"].append(
                (q_propagated, omega_propagated)
            )

        # Calculate mu_{k+1|k} and Sigma_{k+1|k}
        n = len(self.propagated_sigma_points["dynamics"])
        mu_q, Sigma_q = self.calc_quaternion_mean_and_covariance(
            [
                propagated_sigma_point[0]
                for propagated_sigma_point in self.propagated_sigma_points["dynamics"]
            ],
            mu_k_given_k[0],
        )
        mu_omega = np.mean(
            np.array(
                [
                    propagated_sigma_point[1]
                    for propagated_sigma_point in self.propagated_sigma_points[
                        "dynamics"
                    ]
                ]
            ),
            axis=0,
        )
        mu_kp1_given_k = (mu_q, mu_omega)
        Sigma_kp1_given_k = np.zeros((Sigma_k_given_k.shape))
        Sigma_kp1_given_k[0:3, 0:3] = Sigma_q
        for propagated_sigma_point in self.propagated_sigma_points["dynamics"]:
            e = np.array(propagated_sigma_point[1]) - mu_kp1_given_k[1]
            Sigma_kp1_given_k[3:6, 3:6] += 1 / n * np.outer(e, e)

        return (mu_kp1_given_k, Sigma_kp1_given_k)

    def update(
        self,
        mu_kp1_given_k: Tuple[Quaternion, np.ndarray],
        Sigma_kp1_given_k: np.ndarray,
        y_kp1: np.ndarray,
    ) -> Tuple[Tuple[Quaternion, np.ndarray], np.ndarray]:
        """"""
        # Propagate measurement for each dynamically-propagated sigma point
        self.propagated_sigma_points["measurement"] = []
        for sigma_point in self.propagated_sigma_points["dynamics"]:
            q_propagated, omega_propagated = self.g(sigma_point)

            # Convert the propagated quaternion (which is really gravity
            # expressed in the body frame) to Euclidean space
            self.propagated_sigma_points["measurement"].append(
                np.hstack((q_propagated.vec(), omega_propagated))
            )

        # Calculate mean
        y_hat = np.mean(
            np.array(
                [
                    sigma_point
                    for sigma_point in self.propagated_sigma_points["measurement"]
                ],
            ),
            axis=0,
        )
        y_hat[0:3] = y_hat[0:3] / np.linalg.norm(y_hat[0:3])

        # Calculate Sigma_{yy} and Sigma_{xy}
        n = len(self.propagated_sigma_points["measurement"])
        Sigma_yy = np.zeros((Sigma_kp1_given_k.shape))
        Sigma_xy = np.zeros((Sigma_kp1_given_k.shape))
        for i, (
            dynamics_propagated_sigma_point,
            measurement_propagated_sigma_point,
        ) in enumerate(
            zip(
                self.propagated_sigma_points["dynamics"],
                self.propagated_sigma_points["measurement"],
            )
        ):
            # Add to Sigma_{yy}
            Sigma_yy[0:3, 0:3] += (
                1
                / n
                * np.outer(
                    measurement_propagated_sigma_point[0:3] - y_hat[0:3],
                    measurement_propagated_sigma_point[0:3] - y_hat[0:3],
                )
            )
            Sigma_yy[3:6, 3:6] += (
                1
                / n
                * np.outer(
                    measurement_propagated_sigma_point[3:6] - y_hat[3:6],
                    measurement_propagated_sigma_point[3:6] - y_hat[3:6],
                )
            )

            # Add to Sigma_{xy}
            Sigma_xy[0:3, 0:3] += (
                1
                / 7
                * np.outer(
                    self.E[:, i], measurement_propagated_sigma_point[0:3] - y_hat[0:3]
                )
            )
            Sigma_xy[3:6, 3:6] += (
                1
                / 7
                * np.outer(
                    dynamics_propagated_sigma_point[1] - mu_kp1_given_k[1],
                    measurement_propagated_sigma_point[3:6] - y_hat[3:6],
                )
            )
        Sigma_yy += self.Q

        # Calculate the Kalman gain matrix
        K = self.calc_kalman_gain(Sigma_xy, Sigma_yy)

        # Calculate the innovation using the (k+1)th observation and estimated
        # measurement
        y_kp1[0:3] / np.linalg.norm(y_kp1[0:3])

        innovation = self.calc_innovation(y_kp1, y_hat)

        # Calculate mu_{k+1|k+1}. Move K @ innovation back into quaternion space
        dq = Quaternion()
        dq.from_axis_angle((K @ innovation)[0:3])
        mu_kp1_given_kp1 = (
            dq * mu_kp1_given_k[0],
            mu_kp1_given_k[1] + (K @ innovation)[3:6],
        )

        # Calculate Sigma_{k+1|k+1}
        Sigma_kp1_given_kp1 = Sigma_kp1_given_k - K @ Sigma_yy @ K.T

        return (mu_kp1_given_kp1, Sigma_kp1_given_kp1)

    def step(
        self,
        mu_k_given_k: Tuple[Quaternion, np.ndarray],
        Sigma_k_given_k: np.ndarray,
        dt: float,
        y_kp1: np.ndarray,
    ) -> Tuple[Tuple[Quaternion, np.ndarray], np.ndarray]:
        # Propogate via dynamics and check output covariance
        mu_kp1_given_k, Sigma_kp1_given_k = self.predict(
            mu_k_given_k, Sigma_k_given_k, dt
        )

        # Update via measurements
        mu_kp1_given_kp1, Sigma_kp1_given_kp1 = self.update(
            mu_kp1_given_k, Sigma_kp1_given_k, y_kp1
        )

        return (mu_kp1_given_kp1, Sigma_kp1_given_kp1)

    def check_covariance(self, Sigma: np.ndarray) -> None:
        assert np.all(np.linalg.eigvals(Sigma) >= 0)
