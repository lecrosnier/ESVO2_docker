import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import imu_cam_calib_core as c  # noqa: E402


def angle_deg(R):
    return np.degrees(np.linalg.norm(c.so3_log(R)))


class CalibCoreTest(unittest.TestCase):
    def test_exp_log_roundtrip(self):
        v = np.array([0.3, -0.2, 0.1])
        np.testing.assert_allclose(c.so3_log(c.so3_exp(v)), v, atol=1e-12)
        np.testing.assert_allclose(c.so3_exp(np.zeros(3)), np.eye(3), atol=1e-12)

    def test_kabsch_recovers_rotation(self):
        rng = np.random.default_rng(0)
        R = c.so3_exp(np.array([0.5, 0.1, -0.7]))
        B = rng.normal(size=(50, 3))
        A = B @ R.T
        self.assertLess(angle_deg(c.kabsch(A, B).T @ R), 1e-9)

    def test_ransac_rotation_sign_convention_and_outliers(self):
        rng = np.random.default_rng(1)
        R_c = c.so3_exp(np.array([0.0, 0.02, 0.0]))  # camera k+1 orientation in camera k frame
        p = rng.normal(size=(200, 3)); p[:, 2] = np.abs(p[:, 2]) + 3.0
        f_prev = p / np.linalg.norm(p, axis=1, keepdims=True)
        q = p @ R_c              # R_c^T p (row-vector form)
        f_next = q / np.linalg.norm(q, axis=1, keepdims=True)
        f_next[:40] = rng.normal(size=(40, 3)); f_next[:40] /= np.linalg.norm(f_next[:40], axis=1, keepdims=True)
        R, inl = c.fit_rotation_ransac(f_prev, f_next, thresh_rad=0.002, iters=200, rng=np.random.default_rng(2))
        self.assertLess(angle_deg(R.T @ R_c), 0.01)
        self.assertGreaterEqual(inl.sum(), 160)
        self.assertFalse(inl[:40].any())

    def test_time_offset(self):
        t = np.arange(0.0, 20.0, 0.005)
        s = np.abs(np.sin(1.3 * t)) + 0.5 * np.abs(np.sin(3.1 * t + 0.4))
        t_d = 0.023
        t_cam = np.arange(0.2, 19.8, 0.01)
        s_cam = np.interp(t_cam - t_d, t, s)   # camera at time tc sees what IMU logged at tc - t_d
        est, corr, sharp = c.estimate_time_offset(t_cam, s_cam, t, s, max_offset=0.1, step=0.001)
        self.assertAlmostEqual(est, t_d, delta=0.0015)
        self.assertGreater(corr, 0.99)
        self.assertGreater(sharp, 0.0)

    def test_time_offset_sharpness_peak_minus_far_max(self):
        # Sharpness is defined as (correlation at the peak) - (best correlation found
        # more than 50 ms away from the peak). Build a signal that decorrelates fast
        # away from its true alignment (irregularly spaced narrow pulses, so there is
        # no periodicity to create a competing correlation peak within +/-100 ms):
        # sharpness should come out clearly large.
        rng = np.random.default_rng(42)
        t = np.arange(0.0, 20.0, 0.002)
        base_centers = np.arange(0.5, 19.5, 0.37)
        centers = base_centers + rng.normal(scale=0.05, size=len(base_centers))
        s = np.zeros_like(t)
        for ctr in centers:
            s += np.exp(-0.5 * ((t - ctr) / 0.006) ** 2)
        t_d = 0.023
        t_cam = np.arange(0.6, 19.4, 0.004)
        s_cam = np.interp(t_cam - t_d, t, s)
        est, corr, sharp = c.estimate_time_offset(t_cam, s_cam, t, s, max_offset=0.1, step=0.001)
        self.assertAlmostEqual(est, t_d, delta=0.0015)
        self.assertGreater(corr, 0.99)
        # Measured value ~1.06; a wrong far-set statistic (e.g. mean instead of max)
        # or a wrong cutoff cannot fake this bound down, so it pins the "far max" half.
        self.assertGreater(sharp, 0.5)

    def test_time_offset_sharpness_respects_50ms_cutoff(self):
        # Build a signal with a strong secondary correlation peak ~70 ms from the true
        # peak (inside the search range but outside the 50 ms "near" band). With the
        # correct 50 ms cutoff that secondary peak falls in the far set, so it caps the
        # sharpness at a small value. An implementation using a wider cutoff (e.g. 100 ms)
        # would wrongly treat 70 ms as "near" and exclude it, reporting a large sharpness
        # instead -- so this assertion is what catches the cutoff being wrong.
        period = 0.07  # secondary correlation peak at +/- 70 ms, within max_offset=0.1
        t = np.arange(0.0, 20.0, 0.002)
        s = np.zeros_like(t)
        n_periods = int(20.0 / period)
        for i in range(n_periods):
            ctr = i * period + 0.01
            s += np.exp(-0.5 * ((t - ctr) / 0.01) ** 2)
        s *= 1.0 + 0.05 * np.sin(0.3 * t)  # break exact periodicity slightly
        t_d = 0.023
        t_cam = np.arange(0.6, 19.4, 0.004)
        s_cam = np.interp(t_cam - t_d, t, s)
        est, corr, sharp = c.estimate_time_offset(t_cam, s_cam, t, s, max_offset=0.1, step=0.001)
        self.assertAlmostEqual(est, t_d, delta=0.0015)
        self.assertGreater(corr, 0.99)
        # Measured value ~5e-7 with the correct 50 ms cutoff; a wider cutoff (e.g. 100 ms)
        # excludes the secondary peak from the far set and measures sharp ~1.0 instead,
        # so this bound pins the "50 ms" half of the definition.
        self.assertLess(sharp, 0.2)

    def test_time_offset_raises_on_insufficient_overlap(self):
        # Camera and IMU spans barely overlap: after trimming max_offset off each end of
        # the IMU span, almost no camera samples remain. Silently returning a sentinel
        # offset here would look like a real answer, so this must raise instead.
        t = np.arange(0.0, 20.0, 0.005)
        s = np.abs(np.sin(1.3 * t)) + 0.5 * np.abs(np.sin(3.1 * t + 0.4))
        t_cam = np.arange(19.85, 19.95, 0.01)  # entirely inside the trimmed-out [hi, +inf) region
        s_cam = np.interp(t_cam, t, s)
        with self.assertRaises(ValueError):
            c.estimate_time_offset(t_cam, s_cam, t, s, max_offset=0.1, step=0.001)

    def test_fit_R_b_c_with_outliers(self):
        rng = np.random.default_rng(3)
        R_b_c = c.so3_exp(np.array([1.2, -0.4, 0.3]))
        w_cam = rng.normal(size=(500, 3))
        w_imu = w_cam @ R_b_c.T + rng.normal(scale=0.01, size=(500, 3))
        w_imu[:50] = rng.normal(scale=3.0, size=(50, 3))
        R, inl, rms = c.fit_R_b_c(w_imu, w_cam)
        self.assertLess(angle_deg(R.T @ R_b_c), 0.2)
        self.assertLess(rms, 0.05)
        self.assertLess(inl[:50].mean(), 0.2)

    def test_signed_permutation_distance(self):
        P = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])
        self.assertAlmostEqual(c.signed_permutation_distance_deg(P), 0.0, places=6)
        tilted = P @ c.so3_exp(np.array([np.radians(3.0), 0, 0]))
        self.assertAlmostEqual(c.signed_permutation_distance_deg(tilted), 3.0, delta=0.05)


if __name__ == '__main__':
    unittest.main()
