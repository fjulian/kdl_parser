import unittest

import sys
from os import path

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

from highlevel_planning.tools.fitting import *
import highlevel_planning.tools.util as util


class TestModelFitting(unittest.TestCase):
    def setUp(self):
        pass

    def test_line_fitting(self):
        line_start = np.array([0.5, 1.6, -0.2])
        line_direction = np.array([1.5, 5.2, 0.1])
        line_direction /= np.linalg.norm(line_direction)
        points_line = []
        for i in range(50):
            new_point = (
                line_start
                + np.random.uniform(-5.0, 5.0) * line_direction
                + np.random.normal(0.0, 0.01, 3)
            )
            points_line.append(new_point)
        points_line = np.array(points_line)

        estimated_line_start, estimated_line_direction = fit_line_3d(points_line, False)
        estimated_line_start2, estimated_line_direction2 = alt_fit_line_3d(
            points_line, False
        )

        d = np.linspace(-2, 2, 5)
        temp1 = np.hstack([estimated_line_direction.reshape(-1, 1)] * len(d))
        temp2 = np.hstack([estimated_line_start.reshape(-1, 1)] * len(d))
        estimated_points = temp2 + np.multiply(d.reshape(1, -1), temp1)

        temp1 = np.hstack([estimated_line_direction2.reshape(-1, 1)] * len(d))
        temp2 = np.hstack([estimated_line_start2.reshape(-1, 1)] * len(d))
        estimated_points2 = temp2 + np.multiply(d.reshape(1, -1), temp1)

        temp1 = np.hstack([line_direction.reshape(-1, 1)] * len(d))
        temp2 = np.hstack([line_start.reshape(-1, 1)] * len(d))
        true_points = temp2 + np.multiply(d.reshape(1, -1), temp1)

        fig = plt.figure()
        ax = Axes3D(fig)

        ax.scatter(points_line[:, 0], points_line[:, 1], points_line[:, 2])
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")

        ax.plot(true_points[0, :], true_points[1, :], true_points[2, :], label="true")
        ax.plot(
            estimated_points[0, :],
            estimated_points[1, :],
            estimated_points[2, :],
            label="est",
        )
        ax.plot(
            estimated_points2[0, :],
            estimated_points2[1, :],
            estimated_points2[2, :],
            label="est2",
        )
        ax.legend()

        # plt.show()

        self.assertTrue(True)


class TestTransforms(unittest.TestCase):
    def test_inverse_trafo(self):
        pos = np.array([0.5, 0.2, 0.8])
        orient = R.from_quat([0.001, 0.416, -0.139, -0.899])
        T1 = util.homogenous_trafo(pos, orient)

        T2_man = util.invert_hom_trafo(T1)

        orient_inv = orient.inv()
        T2_auto = util.homogenous_trafo(-orient_inv.apply(pos), orient_inv)

        self.assertTrue(np.all(T2_man - T2_auto < 1e-12))


if __name__ == "__main__":
    unittest.main()
