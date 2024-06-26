from scipy import optimize
from scipy.interpolate import PPoly

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import time
from math import factorial

from .graph_search import graph_search

# from graph_search import graph_search


class WorldTraj(object):
    """ """

    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.25, 0.25, 0.25])
        self.margin = 0.5
        self.goal = goal

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        start_time = time.time()
        self.path, _ = graph_search(
            world, self.resolution, self.margin, start, goal, astar=True
        )
        print(f"Path generation time [s]: {time.time() - start_time}")

        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        p_0 = self.path[0, :]
        p_f = self.path[-1, :]
        path = self.path[1:-1, :]
        path[0, :] = start
        path[-1, :] = goal
        sparse_path = self.ramer_douglas_peucker(path, 0.1)
        path = sparse_path

        self.points = path
        points = self.points
        N, _ = points.shape
        self.v = 2.5

        self.r_dot = np.zeros((N - 1, 3))
        self.t = np.zeros((N,))
        for i in range(N - 1):
            d_i = np.linalg.norm(points[i + 1, :3] - points[i, :3])
            l_hat_i = (points[i + 1, :3] - points[i, :3]) / d_i
            self.r_dot[i, :] = self.v * l_hat_i
            self.t[i + 1] = self.t[i] + d_i / self.v

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x = np.zeros((3,))
        x_dot = np.zeros((3,))
        x_ddot = np.zeros((3,))
        x_dddot = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0
        if t == float("inf"):
            x = self.goal
            flat_output = {
                "x": x,
                "x_dot": x_dot,
                "x_ddot": x_ddot,
                "x_dddot": x_dddot,
                "x_ddddot": x_ddddot,
                "yaw": yaw,
                "yaw_dot": yaw_dot,
            }
            return flat_output

        if self.points.shape[0] == 1:
            x = self.points[:]
            x_dot = np.zeros((3,))
        else:
            i = self.find_segment(t)
            if i == self.points.shape[0]:
                x_dot = np.zeros((3,))
                x = self.points[-1, :]
            else:
                x_dot = self.r_dot[i, :]
                x = self.points[i, :] + x_dot * (t - self.t[i])

        flat_output = {
            "x": x,
            "x_dot": x_dot,
            "x_ddot": x_ddot,
            "x_dddot": x_dddot,
            "x_ddddot": x_ddddot,
            "yaw": yaw,
            "yaw_dot": yaw_dot,
        }
        return flat_output

    def ramer_douglas_peucker(self, path: np.ndarray, epsilon: float) -> np.ndarray:
        """
        TODO: cite in README
        https://medium.com/@indemfeld/the-ramer-douglas-peucker-algorithm-d542807093e7
        """
        i_start = 0
        i_end = path.shape[0] - 1

        max_dist = 0
        i_max = 0

        for i in range(i_start + 1, i_end):
            dist = self.calc_point_to_line_distance(
                path[i_start, :], path[i_end, :], path[i, :]
            )
            if dist > max_dist:
                max_dist = dist
                i_max = i

        if max_dist > epsilon:
            left = self.ramer_douglas_peucker(path[i_start : i_max + 1, :], epsilon)
            right = self.ramer_douglas_peucker(path[i_max:, :], epsilon)

            res = np.vstack((left[:-1], right))
            return res
        else:
            res = np.vstack((path[i_start, :], path[i_end, :]))
            return res

    def calc_point_to_line_distance(
        self, p_start: np.ndarray, p_end: np.ndarray, p_path: np.ndarray
    ) -> float:
        """
        TODO: cite in README
        https://stackoverflow.com/questions/56463412/distance-from-a-point-to-a-line-segment-in-3d-python
        """
        l = (p_end - p_start) / np.linalg.norm(p_end - p_start)

        h = np.maximum.reduce([(p_start - p_path) @ l, (p_path - p_end) @ l, 0])

        c = np.cross(p_path - p_start, l)

        return np.hypot(h, np.linalg.norm(c))

    def find_segment(self, t: float) -> int:
        """ """
        if t == 0:
            return 0

        for i in range(self.t.shape[0] - 1):
            if self.t[i] < t and t <= self.t[i + 1]:
                return i

        return self.t.shape[0]
