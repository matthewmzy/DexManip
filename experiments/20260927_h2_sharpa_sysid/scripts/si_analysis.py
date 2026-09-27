"""Offline SI fits. No robot/network access and no files are written here.

Angles are radians, times seconds, torques N m, and forces N. Servo fits describe
the *closed-loop command-to-position* behavior, including host/transport timing;
they cannot identify motor Kp/Kd, inertia, or friction from positions alone.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares, minimize_scalar


def _vectors(min_samples: int = 3, **arrays) -> list[np.ndarray]:
    result = []
    length = None
    for name, value in arrays.items():
        a = np.asarray(value, dtype=float)
        if a.ndim != 1 or not np.all(np.isfinite(a)):
            raise ValueError(f"{name} must be a finite one-dimensional array")
        if len(a) < min_samples:
            raise ValueError(f"{name} needs at least {min_samples} samples")
        if length is not None and len(a) != length:
            raise ValueError("arrays must have equal lengths")
        length = len(a)
        result.append(a)
    return result


def _rmse(residual: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(residual))))


def _command_events(t: np.ndarray, u: np.ndarray, delay: float):
    """Merge output timestamps with delayed zero-order-hold command events."""
    shifted = t + delay
    events = np.unique(np.concatenate((t, shifted[(shifted > t[0]) & (shifted < t[-1])])))
    midpoint = (events[:-1] + events[1:]) * 0.5
    command_index = np.maximum(np.searchsorted(shifted, midpoint, side="right") - 1, 0)
    return np.diff(events), u[command_index], np.searchsorted(events, t)


def _unit_response(events, tau: float) -> np.ndarray:
    """Exact response to held commands with zero initial state and unit gain."""
    dt, command, sample_index = events
    alpha = np.exp(-dt / tau)
    forcing = -np.expm1(-dt / tau) * command
    state = np.empty(len(dt) + 1)
    state[0] = 0.0
    for i in range(len(dt)):
        state[i + 1] = alpha[i] * state[i] + forcing[i]
    return state[sample_index]


def fit_servo(t, u, q, max_delay_s: float = 0.15) -> dict:
    """Fit qdot=(gain*u(t-delay)+bias-q)/tau using a 70/30 time split.

    ``u[k]`` is held on [t[k], t[k+1]); before the recording it is assumed to
    equal u[0]. q[0] supplies the only measured initial state. Fitting uses the
    first 70%; predictions continue through the holdout without resetting to a
    measured position. Delay uses a grid followed by bounded local refinement.

    ``accepted`` is a screening result for this recording, not certification of
    a simulator or controller. Poor excitation/invalid timestamps raise
    ValueError; valid but poorly fitted recordings return accepted=False.
    """
    t, u, q = _vectors(min_samples=80, t=t, u=u, q=q)
    t = t - t[0]
    dt = np.diff(t)
    if np.any(dt <= 0):
        raise ValueError("sample timestamps must be strictly increasing; duplicates are invalid")
    median_dt = float(np.median(dt))
    jitter = float(np.max(np.abs(dt - median_dt)) / median_dt)
    if jitter > 0.10:
        raise ValueError("sample time jitter exceeds 10%; inspect gaps before explicit resampling")
    if not np.isfinite(max_delay_s) or max_delay_s < 0:
        raise ValueError("max_delay_s must be finite and nonnegative")
    split = int(0.7 * len(t))
    if max_delay_s > t[split - 1] / 5:
        raise ValueError("training duration must be at least five times max_delay_s")
    for name, values in (("command", u[:split]), ("position", q[:split])):
        if np.ptp(values) < 0.002 or np.std(values) < 0.0002:
            raise ValueError(f"insufficient {name} excitation: span <0.002 rad or std <0.0002 rad")

    train_t, train_u, train_q = t[:split], u[:split], q[:split]
    tau_bounds = (median_dt / 20, max(2.0, float(train_t[-1]) / 2))
    gain_bounds = (0.01, 5.0)
    bias_limit = max(1.0, 3 * float(np.ptp(train_u)), 2 * float(np.max(np.abs(train_q))))
    delay_grid = np.linspace(0, max_delay_s, min(61, max(2, int(np.ceil(max_delay_s / median_dt)) + 1)))
    if max_delay_s == 0:
        delay_grid = np.array([0.0])

    def linear_fit(tau, events):
        homogeneous = np.exp(-train_t / tau)
        response = _unit_response(events, tau)
        design = np.column_stack((response, 1 - homogeneous))
        target = train_q - homogeneous * q[0]
        coefficients = np.linalg.lstsq(design, target, rcond=None)[0]
        # Exact bounded 2-column least squares: inspect unconstrained and edges.
        candidates = []
        if gain_bounds[0] <= coefficients[0] <= gain_bounds[1] and abs(coefficients[1]) <= bias_limit:
            candidates.append(coefficients)
        for gain in gain_bounds:
            bias = np.dot(design[:, 1], target - gain * design[:, 0]) / np.dot(design[:, 1], design[:, 1])
            candidates.append(np.array([gain, np.clip(bias, -bias_limit, bias_limit)]))
        for bias in (-bias_limit, bias_limit):
            denominator = np.dot(design[:, 0], design[:, 0])
            gain = np.dot(design[:, 0], target - bias * design[:, 1]) / max(denominator, 1e-30)
            candidates.append(np.array([np.clip(gain, *gain_bounds), bias]))
        fitted = min(candidates, key=lambda c: np.sum(np.square(design @ c - target)))
        residual = design @ fitted - target
        return float(np.mean(residual**2)), fitted

    grid_results = []
    for delay in delay_grid:
        events = _command_events(train_t, train_u, float(delay))
        result = minimize_scalar(
            lambda log_tau: linear_fit(np.exp(log_tau), events)[0],
            bounds=np.log(tau_bounds), method="bounded", options={"xatol": 1e-6},
        )
        tau = float(np.exp(result.x))
        loss, coefficients = linear_fit(tau, events)
        grid_results.append((loss, tau, float(delay), *coefficients))
    best = min(grid_results)

    def prediction(parameters, timestamps, command):
        tau, gain, bias = parameters[:3]
        delay = parameters[3] if len(parameters) == 4 else 0.0
        homogeneous = np.exp(-timestamps / tau)
        response = _unit_response(_command_events(timestamps, command, delay), tau)
        return homogeneous * q[0] + gain * response + bias * (1 - homogeneous)

    initial = [best[1], best[3], best[4]]
    lower = [tau_bounds[0], gain_bounds[0], -bias_limit]
    upper = [tau_bounds[1], gain_bounds[1], bias_limit]
    if max_delay_s > 0:
        grid_step = float(delay_grid[1] - delay_grid[0])
        initial.append(best[2])
        lower.append(max(0.0, best[2] - grid_step))
        upper.append(min(max_delay_s, best[2] + grid_step))
    refined = least_squares(
        lambda p: prediction(p, train_t, train_u) - train_q,
        initial, bounds=(lower, upper), x_scale="jac", max_nfev=150,
        ftol=1e-10, xtol=1e-10, gtol=1e-10,
    )
    tau, gain, bias = map(float, refined.x[:3])
    delay = float(refined.x[3]) if max_delay_s > 0 else 0.0
    predicted = prediction(refined.x, t, u)
    train_rmse = _rmse(predicted[:split] - train_q)
    holdout_rmse = _rmse(predicted[split:] - q[split:])
    train_scale = float(np.std(train_q))
    holdout_scale = float(np.std(q[split:]))
    jac_column_norm = np.linalg.norm(refined.jac, axis=0)
    normalized_jacobian = refined.jac / np.maximum(jac_column_norm, 1e-30)
    jac_singular = np.linalg.svd(normalized_jacobian, compute_uv=False)
    identifiability_condition = float(jac_singular[0] / max(jac_singular[-1], 1e-30))
    warnings = []
    if np.any(jac_column_norm < 1e-10) or identifiability_condition > 1e4:
        warnings.append("weak parameter separation: tau/delay/gain are not independently identified by this excitation")
    if np.std(u[split:]) < 0.0002 or np.ptp(u[split:]) < 0.002:
        warnings.append("holdout command excitation is insufficient; repeat with independent excitation")
    if abs(float(np.corrcoef(train_t, train_u)[0, 1])) > 0.97:
        warnings.append("command is nearly a linear ramp; use multiple frequencies and reversals")
    transitions = int(np.count_nonzero(np.abs(np.diff(train_u)) > max(1e-6, np.ptp(train_u) * 1e-4)))
    if transitions < 4:
        warnings.append("fewer than four command transitions; repeat excitation at multiple rates")
    if tau <= tau_bounds[0] * 1.05 or tau >= tau_bounds[1] * 0.95:
        warnings.append("time constant reaches fitting bounds or sampling resolution")
    if gain <= gain_bounds[0] * 1.05 or gain >= gain_bounds[1] * 0.99 or abs(bias) >= bias_limit * 0.99:
        warnings.append("gain or bias reaches fitting bounds; check joint units, signs, and model")
    if max_delay_s > 0 and delay >= max_delay_s - median_dt * 0.1:
        warnings.append("delay reaches search limit; longer records and a larger delay range are needed")
    train_threshold = max(0.003, 0.20 * train_scale)
    holdout_threshold = max(0.003, 0.25 * holdout_scale)
    residual_ok = train_rmse <= train_threshold and holdout_rmse <= holdout_threshold
    if not residual_ok:
        warnings.append("free-run residual exceeds screening thresholds; first-order model is inadequate")
    if not refined.success:
        warnings.append("optimizer did not converge")
    return {
        "model": "closed_loop_first_order_zoh_with_effective_delay",
        "tau_s": tau, "delay_s": delay, "gain": gain, "bias_rad": bias,
        "train_rmse_rad": train_rmse, "holdout_rmse_rad": holdout_rmse,
        "train_nrmse_std": train_rmse / max(train_scale, 1e-12),
        "holdout_nrmse_std": holdout_rmse / max(holdout_scale, 1e-12),
        "train_threshold_rad": train_threshold, "holdout_threshold_rad": holdout_threshold,
        "sample_period_s": median_dt, "maximum_relative_jitter": jitter,
        "parameter_condition_number_scaled": identifiability_condition,
        "split_index": split, "prediction_rad": predicted.tolist(),
        "delay_grid_s": delay_grid.tolist(),
        "delay_profile_train_rmse_rad": [float(np.sqrt(row[0])) for row in grid_results],
        "accepted": bool(refined.success and residual_ok and not warnings),
        "warnings": warnings, "excitation_warning": "; ".join(warnings) if warnings else None,
        "confidence": "screening only; no statistical confidence interval or cross-session validation",
        "limitations": [
            "Only a closed-loop equivalent is identified; Kp/Kd, inertia and friction are not identifiable from u,q alone.",
            "Delay includes host timestamps, transport, SDK and actuator effects; it is not pure motor latency.",
            "Commands are zero-order held; pre-recording command is assumed equal to the first command.",
            "Holdout is a contiguous 30% free run using parameters fitted on the first 70%; q is not reset at the split.",
            "Acceptance thresholds are screening heuristics, not task-specific sim2real tolerances.",
        ],
    }


def fit_rigid_transform(source, target) -> dict:
    """Proper Kabsch transform: target ~= (R @ source.T).T + translation.

    Points should be in meters for the returned *_m units. Noncollinear planar
    data are valid; collinear or coincident correspondences are refused.
    """
    source, target = np.asarray(source, dtype=float), np.asarray(target, dtype=float)
    if source.ndim != 2 or source.shape[1:] != (3,) or source.shape != target.shape or len(source) < 3:
        raise ValueError("source and target must have matching (N,3) shapes, N >= 3")
    if not np.all(np.isfinite(source)) or not np.all(np.isfinite(target)):
        raise ValueError("point correspondences must be finite")
    source_center, target_center = source.mean(axis=0), target.mean(axis=0)
    a, b = source - source_center, target - target_center
    for points in (a, b):
        values = np.linalg.svd(points, compute_uv=False)
        if values[0] <= 1e-12 or values[1] <= values[0] * 1e-8:
            raise ValueError("point correspondences must be noncollinear")
    u, _, vt = np.linalg.svd(a.T @ b)
    handedness = np.diag([1.0, 1.0, np.linalg.det(vt.T @ u.T)])
    rotation = vt.T @ handedness @ u.T
    translation = target_center - rotation @ source_center
    residual = source @ rotation.T + translation - target
    rmse = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    scale = float(np.sqrt(np.mean(np.sum(b**2, axis=1))))
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, translation
    accepted = rmse <= max(1e-6, 0.01 * scale)
    return {
        "rotation": rotation.tolist(), "translation_m": translation.tolist(),
        "transform_4x4": transform.tolist(), "rmse_m": rmse,
        "rotation_determinant": float(np.linalg.det(rotation)),
        "accepted": bool(accepted),
        "warnings": [] if accepted else ["rigid residual exceeds 1% of point-cloud spread; check correspondences/scale/handedness"],
        "limitations": "Rigid correspondence fit only; no held-out calibration or timestamp/extrinsic drift validation.",
    }


def fit_hinge(dq, ddq, tau) -> dict:
    """Fit independent measured torque: tau=I*ddq+b*dq+fc*sign(dq)+offset.

    This assumes gravity/known external load have already been subtracted.
    Position-tracking error and uncalibrated motor current are not torque.
    """
    dq, ddq, torque = _vectors(min_samples=20, dq=dq, ddq=ddq, tau=tau)
    if np.std(dq) < 1e-4 or np.std(ddq) < 1e-4:
        raise ValueError("hinge identification needs independently varying velocity and acceleration")
    if not (np.any(dq > 1e-4) and np.any(dq < -1e-4)):
        raise ValueError("both directions are needed to separate Coulomb friction and torque offset")
    design = np.column_stack((ddq, dq, np.sign(dq), np.ones(len(dq))))
    scales = np.sqrt(np.mean(design**2, axis=0))
    normalized = design / scales
    singular = np.linalg.svd(normalized, compute_uv=False)
    condition = float(singular[0] / max(singular[-1], 1e-30))
    if np.linalg.matrix_rank(normalized) < 4 or condition > 1e6:
        raise ValueError("rank-deficient or ill-conditioned hinge excitation; signals must vary independently")
    coefficients = np.linalg.lstsq(normalized, torque, rcond=None)[0] / scales
    inertia, damping, coulomb, offset = map(float, coefficients)
    predicted = design @ coefficients
    rmse = _rmse(predicted - torque)
    warnings = []
    if inertia <= 0 or damping < -1e-10 or coulomb < -1e-10:
        warnings.append("nonphysical inertia/damping/friction; verify torque calibration, signs, loads and excitation")
    if rmse > max(1e-6, 0.15 * float(np.std(torque))):
        warnings.append("torque residual exceeds 15% of torque standard deviation")
    if condition > 100:
        warnings.append("weak parameter separation; repeat with richer independent excitation")
    return {
        "inertia_kg_m2": inertia, "viscous_Nm_s_per_rad": damping,
        "coulomb_Nm": coulomb, "offset_Nm": offset,
        "rmse_Nm": rmse, "condition_number_scaled": condition,
        "accepted": not warnings, "warnings": warnings,
        "limitations": "Requires independently calibrated N m torque with gravity/external load removed; no confidence interval or held-out validation.",
    }


def fit_contact(normal_force, tangent_at_slip) -> dict:
    """Fit slip-onset Ft=mu*Fn through the origin using positive force magnitudes.

    Each row is an independently measured slip onset at a stated material pair,
    not an arbitrary timestep during sticking. This estimates static friction;
    dynamic friction/compliance/restitution require separate experiments.
    """
    normal, tangent = _vectors(min_samples=3, normal_force=normal_force, tangent_at_slip=tangent_at_slip)
    if np.any(normal <= 0) or np.any(tangent <= 0):
        raise ValueError("normal and slip-onset tangential force magnitudes must be strictly positive N")
    mu = float(np.dot(normal, tangent) / np.dot(normal, normal))
    residual = mu * normal - tangent
    rmse = _rmse(residual)
    ratios = tangent / normal
    warnings = []
    if rmse > 0.15 * float(np.sqrt(np.mean(tangent**2))):
        warnings.append("slip force does not support a single Coulomb coefficient across these trials")
    if np.ptp(normal) < 0.1 * float(np.mean(normal)):
        warnings.append("normal loads span less than 10%; load dependence is untested")
    return {
        "static_friction_mu": mu, "rmse_N": rmse,
        "trial_mu": ratios.tolist(), "trial_mu_std": float(np.std(ratios, ddof=1)),
        "accepted": not warnings, "warnings": warnings,
        "limitations": "Slip-onset static friction only; measured force magnitudes and correct material pairing are required. No dynamic friction, compliance or restitution estimate.",
    }
