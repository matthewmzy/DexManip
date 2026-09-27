"""Synthetic recovery and refusal tests; no hardware/network access or outputs."""

import importlib.util
import json
from pathlib import Path
import unittest

import numpy as np


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "si_analysis.py"
SPEC = importlib.util.spec_from_file_location("si_analysis", MODULE_PATH)
analysis = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analysis)


def synthetic_servo(t, u, tau=0.09, delay_samples=2, gain=0.87, bias=0.035):
    """Independent direct discrete simulation for an integer sample delay."""
    q = np.empty_like(t)
    q[0] = gain * u[0] + bias
    for k in range(1, len(t)):
        alpha = np.exp(-(t[k] - t[k - 1]) / tau)
        q[k] = alpha * q[k - 1] + (1 - alpha) * (gain * u[max(0, k - 1 - delay_samples)] + bias)
    return q


class ServoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.t = np.arange(0, 12, 0.02)
        cls.u = 0.2 + 0.15 * np.sin(2 * np.pi * 0.8 * cls.t) + 0.07 * np.sin(2 * np.pi * 2.3 * cls.t)
        cls.q = synthetic_servo(cls.t, cls.u)
        cls.fit = analysis.fit_servo(cls.t, cls.u, cls.q)

    def test_known_parameter_recovery_and_json(self):
        fitted = self.fit
        self.assertTrue(fitted["accepted"], fitted["warnings"])
        self.assertAlmostEqual(fitted["tau_s"], 0.09, delta=0.002)
        self.assertAlmostEqual(fitted["delay_s"], 0.04, delta=0.002)
        self.assertAlmostEqual(fitted["gain"], 0.87, delta=0.01)
        self.assertAlmostEqual(fitted["bias_rad"], 0.035, delta=0.002)
        self.assertLess(fitted["holdout_rmse_rad"], 1e-5)
        self.assertEqual(fitted["split_index"], 420)
        json.dumps(fitted, allow_nan=False)

    def test_holdout_does_not_teacher_force_or_fit_future(self):
        changed = self.q.copy()
        changed[420:] += 0.2
        fitted = analysis.fit_servo(self.t, self.u, changed)
        np.testing.assert_allclose(fitted["prediction_rad"], self.fit["prediction_rad"], atol=1e-8)
        self.assertAlmostEqual(fitted["holdout_rmse_rad"], 0.2, delta=1e-6)
        self.assertFalse(fitted["accepted"])

    def test_constant_and_low_excitation_refused(self):
        for command in (np.zeros_like(self.u), self.u * 1e-4):
            with self.assertRaisesRegex(ValueError, "excitation"):
                analysis.fit_servo(self.t, command, self.q)

    def test_invalid_inputs_refused(self):
        repeated = self.t.copy()
        repeated[20] = repeated[19]
        backwards = self.t[::-1]
        jitter = self.t.copy()
        jitter[30] += 0.009
        nonfinite = self.q.copy()
        nonfinite[10] = np.nan
        cases = [(repeated, self.u, self.q), (backwards, self.u, self.q),
                 (jitter, self.u, self.q), (self.t, self.u, nonfinite),
                 (self.t, self.u[:-1], self.q), (self.t, self.u[:, None], self.q)]
        for args in cases:
            with self.subTest(args=str([np.shape(x) for x in args])):
                with self.assertRaises(ValueError):
                    analysis.fit_servo(*args)

    def test_fractional_delay_exact_event_integration(self):
        # Generate independent high-rate ZOH data and sample every 20 ms.
        fine_t = np.arange(0, 10, 0.001)
        coarse_t = fine_t[::20]
        u = 0.2 * np.sin(coarse_t * 7) + 0.08 * np.sin(coarse_t * 17)
        fine_u = np.repeat(u, 20)
        fine_q = synthetic_servo(fine_t, fine_u, delay_samples=37)
        fitted = analysis.fit_servo(coarse_t, u, fine_q[::20])
        self.assertAlmostEqual(fitted["delay_s"], 0.037, delta=0.001)
        self.assertAlmostEqual(fitted["tau_s"], 0.09, delta=0.001)
        self.assertLess(fitted["holdout_rmse_rad"], 1e-5)


class GeometryTests(unittest.TestCase):
    def test_rigid_transform_recovery(self):
        source = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0.2, 0.3, 0.7]])
        angle = 0.6
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        translation = np.array([0.1, -0.3, 0.4])
        fit = analysis.fit_rigid_transform(source, source @ rotation.T + translation)
        np.testing.assert_allclose(fit["rotation"], rotation, atol=1e-12)
        np.testing.assert_allclose(fit["translation_m"], translation, atol=1e-12)
        self.assertTrue(fit["accepted"])

    def test_reflection_does_not_return_improper_rotation(self):
        source = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
        target = source * [-1, 1, 1]
        fit = analysis.fit_rigid_transform(source, target)
        self.assertAlmostEqual(fit["rotation_determinant"], 1.0, places=12)
        self.assertGreater(fit["rmse_m"], 0.1)
        self.assertFalse(fit["accepted"])

    def test_collinear_and_invalid_geometry_refused(self):
        for points in (np.zeros((3, 3)), np.arange(9).reshape(3, 3), np.zeros((2, 3)), np.full((4, 3), np.nan)):
            with self.assertRaises(ValueError):
                analysis.fit_rigid_transform(points, points)


class ForceTests(unittest.TestCase):
    @staticmethod
    def hinge_signals():
        t = np.linspace(0, 12, 1000)
        velocity = np.cos(t * 2) + 0.3 * np.cos(t * 7)
        acceleration = -2 * np.sin(t * 2) - 2.1 * np.sin(t * 7)
        return velocity, acceleration

    def test_hinge_recovery(self):
        velocity, acceleration = self.hinge_signals()
        torque = 0.004 * acceleration + 0.03 * velocity + 0.08 * np.sign(velocity) + 0.01
        fit = analysis.fit_hinge(velocity, acceleration, torque)
        self.assertTrue(fit["accepted"], fit["warnings"])
        self.assertAlmostEqual(fit["inertia_kg_m2"], 0.004, places=10)
        self.assertAlmostEqual(fit["viscous_Nm_s_per_rad"], 0.03, places=10)
        self.assertAlmostEqual(fit["coulomb_Nm"], 0.08, places=10)
        json.dumps(fit, allow_nan=False)

    def test_hinge_independence_and_physics(self):
        velocity, acceleration = self.hinge_signals()
        for bad_dq, bad_ddq in ((abs(velocity), acceleration), (velocity, velocity), (velocity, np.zeros_like(velocity))):
            with self.assertRaises(ValueError):
                analysis.fit_hinge(bad_dq, bad_ddq, velocity)
        fit = analysis.fit_hinge(velocity, acceleration, -0.004 * acceleration + 0.03 * velocity + 0.08 * np.sign(velocity))
        self.assertFalse(fit["accepted"])

    def test_contact_recovery_and_invalid_forces(self):
        fit = analysis.fit_contact([1, 2, 3, 4], [0.6, 1.2, 1.8, 2.4])
        self.assertAlmostEqual(fit["static_friction_mu"], 0.6)
        self.assertTrue(fit["accepted"])
        for normal, tangent in (([0, 1, 2], [1, 2, 3]), ([1, 2, 3], [1, -2, 3]), ([1, 2, 3], [1, np.inf, 3]), ([1, 2], [1, 2])):
            with self.assertRaises(ValueError):
                analysis.fit_contact(normal, tangent)


if __name__ == "__main__":
    unittest.main()
