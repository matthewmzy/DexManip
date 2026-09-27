"""Local SI protocol/plans. No robot imports or writes; outputs go to explicit run directories."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
EXPERIMENT = Path(__file__).resolve().parents[1]
PR_SHA = "573175a2936253fe41aebb1d246bed47f3ced947"
HAND_JOINTS = [
    "thumb_CMC_FE", "thumb_CMC_AA", "thumb_MCP_FE", "thumb_MCP_AA", "thumb_IP",
    "index_MCP_FE", "index_MCP_AA", "index_PIP", "index_DIP",
    "middle_MCP_FE", "middle_MCP_AA", "middle_PIP", "middle_DIP",
    "ring_MCP_FE", "ring_MCP_AA", "ring_PIP", "ring_DIP",
    "pinky_CMC", "pinky_MCP_FE", "pinky_MCP_AA", "pinky_PIP", "pinky_DIP",
]
ARM_JOINTS = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"]
CHANNELS = {
    **{f"{s}_hand": [f"{s}_{j}" for j in HAND_JOINTS] for s in ("left", "right")},
    **{f"{s}_arm": [f"{s}_{j}_joint" for j in ARM_JOINTS] for s in ("left", "right")},
}
H2_BODY_JOINTS = [
    f"{side}_{joint}_joint" for side in ("left", "right")
    for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
] + [f"waist_{axis}_joint" for axis in ("yaw", "roll", "pitch")] + CHANNELS["left_arm"] + CHANNELS["right_arm"] + ["head_pitch_joint", "head_yaw_joint"]


def dump(path, obj):
    """Never overwrite an existing measurement/report by accident."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write("\n")


def priors():
    # Data-only snapshot; no IsaacLab or real controller imports.
    source = ROOT / "assets/vendor_priors/sharpa_official.json"
    values = json.loads(source.read_text())["values"]
    if values["SHARPA_JOINTS"] != HAND_JOINTS:
        raise ValueError("Local joint order differs from audited PR; review mapping first")
    rows = {}
    for name in HAND_JOINTS:
        cls = name if name in values["OFFICIAL_USD_GAINS"] else name.split("_", 1)[1]
        kp, kd, armature, friction = values["OFFICIAL_USD_GAINS"][cls]
        effort, velocity = values["URDF_LIMITS"][cls]
        rows[name] = dict(kp_Nm_per_rad=kp * 180 / math.pi, kd_Nm_s_per_rad=kd * 180 / math.pi,
                          armature=armature, physx_joint_friction_dimensionless=friction,
                          asset_effort_limit_Nm=effort, asset_velocity_limit_rad_s=velocity)
    return dict(source=str(source.relative_to(ROOT)), sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                status="vendor_prior_not_measurement_of_this_robot", joints=rows)


def template():
    return {
        "schema_version": 1, "source_pr_sha": PR_SHA,
        "hardware": {"hand_serials": {"left": None, "right": None}, "firmware": None,
                     "bridge_sha": None, "position_mode_confirmed": False,
                     "speed_coeff": None, "current_coeff": None, "state_hz_measured": None,
                     "h2_support_and_controller": "吊装固定，站立控制开启；SI中头腰腿保持不动",
                     "deployment_transport": "dds_bridge", "camera_serial": None},
        "plan": {"channel": "right_hand", "joint_names": CHANNELS["right_hand"],
                 "center_rad": None, "reviewed_lower_rad": None, "reviewed_upper_rad": None,
                 "selected_indices": [5], "amplitude_rad": math.radians(2),
                 "max_command_speed_rad_s": 0.35, "sample_hz": 50,
                 "frequencies_hz": [0.2, 0.5, 1.0], "cycles_per_frequency": 3,
                 "hold_s": 1.0, "ramp_s": 1.0, "reviewed_clearance": False},
        "manual_measurements": {
            "object_mass_kg": None, "object_dimensions_m": None,
            "hand_mount_transform": "pending", "camera_to_base": "pending",
            "finger_object_friction": "pending", "object_table_friction": "pending",
            "scissors_hinge": "pending", "tactile_if_used": "pending",
        },
        "notes": ["null means unmeasured, never zero", "No real excitation executor is included: PR bridge repeats last command without timeout.",
                  "Plan checks joint boxes and speed, not self/environment collision or hardware safety.",
                  "Use reviewed local operating intervals; never sweep to mechanical stops."]}


def build_plan(config):
    p = config["plan"]
    channel = p["channel"]
    if channel not in CHANNELS or p["joint_names"] != CHANNELS[channel]:
        raise ValueError("channel/joint_names must exactly match audited order")
    if 'usd_planning' in config:
        certificate = config['usd_planning']
        selected = p['selected_indices']
        if (selected != [p['joint_names'].index(certificate['checked_joint'])]
                or p['amplitude_rad'] > certificate['checked_amplitude_rad']
                or p['center_rad'] != [certificate['center_joint_positions_rad'][n] for n in p['joint_names']]):
            raise ValueError('USD-checked pose/joint/range changed; rerun usd-plan rather than reuse its geometry check')
    n = len(CHANNELS[channel])
    vectors = []
    for key in ("center_rad", "reviewed_lower_rad", "reviewed_upper_rad"):
        x = np.asarray(p[key], dtype=float)
        if x.shape != (n,) or not np.isfinite(x).all():
            raise ValueError(f"{key} requires {n} finite measured/reviewed radians")
        vectors.append(x)
    center, low, high = vectors
    if p["reviewed_clearance"] is not True:
        raise ValueError("reviewed_clearance must be true after physical/geometry review")
    if np.any(low >= high) or np.any(center < low) or np.any(center > high):
        raise ValueError("invalid operating intervals or center outside intervals")
    selected = p["selected_indices"]
    if not selected or any(type(i) is not int or not 0 <= i < n for i in selected) or len(set(selected)) != len(selected):
        raise ValueError("selected_indices must be unique valid integer indices")
    for key in ("sample_hz", "amplitude_rad", "max_command_speed_rad_s", "hold_s", "ramp_s", "cycles_per_frequency"):
        if not np.isfinite(p[key]) or p[key] <= 0:
            raise ValueError(f"{key} must be finite and positive")
    hz, amp = p["sample_hz"], p["amplitude_rad"]
    freqs = np.asarray(p["frequencies_hz"], dtype=float)
    if freqs.ndim != 1 or len(freqs) < 2 or not np.isfinite(freqs).all() or np.any(freqs <= 0) or max(freqs) >= hz / 10:
        raise ValueError("Use >=2 positive frequencies, each < sample_hz/10")
    if hz > 1000 or p["cycles_per_frequency"] > 100:
        raise ValueError("Plan duration/rate exceeds supported bounds")
    rows, stages = [], []

    def append_segment(j, delta, label):
        a = np.tile(center, (len(delta), 1))
        a[:, j] += delta
        start = sum(len(r) for r in rows)
        rows.append(a)
        stages.append(dict(joint_index=j, joint_name=CHANNELS[channel][j], stage=label,
                           start_sample=start, end_sample=start + len(a)))

    for j in selected:
        if center[j] - amp < low[j] or center[j] + amp > high[j]:
            raise ValueError(f"excitation exceeds reviewed operating interval: {CHANNELS[channel][j]}")
        # Smooth bidirectional plateaus expose lag/asymmetry without discontinuous steps.
        r = np.linspace(0, 1, max(3, int(hz * p["ramp_s"])))
        smooth = (1 - np.cos(math.pi * r)) / 2
        hold = np.ones(max(2, int(hz * p["hold_s"])))
        delta = np.concatenate([hold * 0, amp*smooth, amp*hold, amp*(1-smooth), hold*0,
                                -amp*smooth, -amp*hold, -amp*(1-smooth), hold*0])
        append_segment(j, delta, "bidirectional_ramped_plateaus")
        for f in freqs:
            duration = float(p["cycles_per_frequency"]) / f
            if duration * hz > 500000:
                raise ValueError("Requested frequency produces excessive plan length")
            t = np.linspace(0, duration, int(math.ceil(duration*hz)) + 1)
            envelope = np.sin(math.pi * np.clip(t / p["ramp_s"], 0, 1) / 2)**2
            envelope *= np.sin(math.pi * np.clip((duration-t) / p["ramp_s"], 0, 1) / 2)**2
            append_segment(j, amp*envelope*np.sin(2*math.pi*f*t), f"sine_{f:g}_Hz")
            append_segment(j, hold * 0, "rest")
    q = np.concatenate(rows)
    peak_speed = float(np.max(np.abs(np.diff(q, axis=0))) * hz)
    if peak_speed > p["max_command_speed_rad_s"]:
        raise ValueError(f"peak command speed {peak_speed:.4g} exceeds reviewed cap; lower amplitude/frequency or lengthen ramps")
    return dict(schema_version=1, channel=channel, joint_names=CHANNELS[channel], sample_hz=hz,
                t_s=(np.arange(len(q))/hz).tolist(), q_command_rad=q.tolist(), stages=stages,
                peak_command_speed_rad_s=peak_speed, executable=False,
                status="local_plan_only_not_collision_or_hardware_validated")


def hand_event(message, side, kind, stamp):
    if side not in ("left", "right") or kind not in ("state", "command"):
        raise ValueError("invalid hand event")
    motors = message.motor_state if kind == "state" else message.motor_cmd
    if len(motors) != 22:
        raise ValueError(f"Expected exactly 22 Sharpa motors, received {len(motors)}")
    q = np.asarray([float(m.q) for m in motors])
    if not np.isfinite(q).all():
        raise ValueError("nonfinite Sharpa q")
    if kind == "state":
        q = np.deg2rad(q)
    return dict(kind=kind, channel=f"{side}_hand", t_host_s=stamp, q_rad=q.tolist(),
                device_timestamp=None, raw_q_unit="degrees" if kind == "state" else "radians")


def joint_window(t, u, max_delay_s):
    """Command-only crop for sequential single-joint plans; no measured q is used."""
    if not math.isfinite(max_delay_s) or max_delay_s < 0:
        raise ValueError("max_delay_s must be finite and nonnegative")
    threshold = max(0.001, 0.02*float(np.ptp(u)))
    active = np.flatnonzero(np.abs(u-u[0]) >= threshold)
    if len(active) < 2:
        raise ValueError("No active command window")
    padding = max(1.0,max_delay_s)
    start = max(0, int(np.searchsorted(t,t[active[0]]-padding)))
    end = min(len(t),int(np.searchsorted(t,t[active[-1]]+padding,side="right")))
    metadata = dict(start_sample=start,end_sample_exclusive=end,
        source_start_s=float(t[start]),source_end_s=float(t[end-1]),
        requested_padding_s=padding,command_threshold_rad=threshold,
        rule="first-to-last deviation from first command, plus padding; intended for sequential joint plans, not arbitrary trial segmentation")
    return slice(start,end),metadata


def align_events(events, channel, hz):
    """Explicit resampling; refuses long holes, malformed order, and no command overlap."""
    if not math.isfinite(hz) or not 1 <= hz <= 1000:
        raise ValueError("analysis sample_hz must be 1..1000")
    groups = {}
    quality = {}
    for kind in ("command", "state"):
        group = [e for e in events if e.get("channel") == channel and e.get("kind") == kind]
        if len(group) < 20:
            raise ValueError(f"{channel}: need >=20 real {kind} messages, received {len(group)}")
        # File order is callback enqueue order, not a fabricated device clock.
        t = np.array([e["t_host_s"] for e in group], dtype=float)
        q = np.array([e["q_rad"] for e in group], dtype=float)
        if q.shape != (len(t), len(CHANNELS[channel])) or not np.isfinite(q).all() or not np.isfinite(t).all():
            raise ValueError(f"{channel}/{kind}: invalid dimensions/nonfinite values")
        dt = np.diff(t)
        if np.any(dt <= 0):
            raise ValueError(f"{channel}/{kind}: duplicate/nonmonotonic receive times")
        median = float(np.median(dt))
        # Repeated command stream is required: event-driven sparse commands need separate segmentation.
        if max(dt) > 3 * median:
            raise ValueError(f"{channel}/{kind}: gap >3 median periods; segment/re-record, do not fit across dropout")
        groups[kind] = (t, q)
        quality[kind] = dict(messages=len(t), receive_hz=1/median, median_period_s=median,
                             p95_period_s=float(np.percentile(dt,95)), max_period_s=float(max(dt)),
                             source_freshness="unknown_no_device_timestamp" if channel.endswith("hand") else "inspect_device_tick")
    tc, qc = groups["command"]
    ts, qs = groups["state"]
    start, end = max(tc[0], ts[0]), min(tc[-1], ts[-1])
    if end - start < 4:
        raise ValueError("Need >=4 seconds of overlapping command/state")
    if hz > min(quality[k]["receive_hz"] for k in quality) * 1.25:
        raise ValueError("analysis sample_hz exceeds observed stream rate; lower --sample-hz")
    t = np.arange(start, end, 1/hz)
    u = qc[np.searchsorted(tc, t, side="right") - 1]
    q = np.column_stack([np.interp(t, ts, qs[:,j]) for j in range(qs.shape[1])])
    if channel.endswith("arm"):
        tick = [e.get("device_timestamp") for e in events if e.get("channel") == channel and e.get("kind") == "state"]
        if any(type(x) is not int or x < 0 for x in tick) or any(b <= a for a,b in zip(tick,tick[1:])):
            raise ValueError("H2 tick missing/repeated/backwards; segment reset/wrap or investigate stale source")
        quality["state"]["source_freshness"] = "monotonic_tick_verified_unit_unknown"
    quality["alignment"] = "command zero-order hold, state linear interpolation on host receive clock; NOT wire/actuator latency"
    quality["overlap_start_host_s"] = float(start)
    quality["overlap_end_host_s"] = float(end)
    quality["alignment_delay_resolution_floor_s"] = 1/hz
    quality["delay_caveat"] = "Resampling quantizes command transitions by up to one analysis period; sub-period fitted delays are not timing accuracy. Bridge may repeat stale hardware positions."
    return t-t[0], u, q, quality
