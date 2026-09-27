"""Passive DDS recorder. Intentionally has no publisher/controller/SDK connect helpers."""
from __future__ import annotations

import json
import math
import queue
import threading
import time
from collections import Counter
from pathlib import Path

from si_core import hand_event, dump, PR_SHA, H2_BODY_JOINTS


def record(run_dir, interface, duration, domain=0, include_h2=False, config_path=None):
    if not interface or not math.isfinite(duration) or not 0 < duration <= 86400:
        raise ValueError("explicit interface and duration in (0, 86400] required")
    if not isinstance(domain, int) or not 0 <= domain <= 232:
        raise ValueError("DDS domain must be an integer in [0,232]")
    # Lazy import: every offline command works without DDS/Sharpa/IsaacLab.
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, LowCmd_, LowState_

    config = json.loads(Path(config_path).read_text()) if config_path else None
    dest = Path(run_dir)
    dest.mkdir(parents=True, exist_ok=False)
    dump(dest / "session.json", dict(schema_version=1, source="hardware_dds_passive",
        audited_pr_sha=PR_SHA, interface=interface, domain=domain, requested_duration_s=duration,
        include_h2=include_h2, utc_start=time.strftime("%Y-%m-%dT%H:%M:%SZ",time.gmtime()),
        hardware_configuration_verified=False, hardware_metadata=config.get("hardware") if config else None,
        publishers_created=0,
        time_basis="host monotonic receive callback", sharpa_device_timestamps=False,
        sharpa_valid_fields=["q"], h2_torque_semantics="tau_est is uncalibrated estimate if present"))
    if config is not None:
        dump(dest/"operator_config.json",config)
    events = queue.Queue(maxsize=200000)
    fatal = []
    guard = threading.Lock()
    enabled = True
    counts = Counter()
    start = time.monotonic()
    subscribers = []

    def callback(decoder):
        def receive(msg):
            nonlocal enabled
            with guard:
                if not enabled:
                    return
                try:
                    stamp = time.monotonic() - start
                    for event in decoder(msg, stamp):
                        events.put_nowait(event)
                except Exception as exc:
                    if not fatal:
                        fatal.append(f"{type(exc).__name__}: {exc}")
                    enabled = False
        return receive

    def hand_decoder(side, kind):
        return lambda msg, stamp: [hand_event(msg, side, kind, stamp)]

    def arm_decoder(kind):
        def decode(msg, stamp):
            motors = msg.motor_state if kind == "state" else msg.motor_cmd
            if len(motors) < 29:
                raise ValueError("H2 message has fewer than 29 motors")
            result = []
            for side, first in (("left",15),("right",22)):
                event = dict(kind=kind, channel=f"{side}_arm", t_host_s=stamp,
                             q_rad=[float(motors[i].q) for i in range(first,first+7)],
                             device_timestamp=None)
                if kind == "state":
                    event["dq_rad_s"] = [float(motors[i].dq) for i in range(first,first+7)]
                    if hasattr(motors[first], "tau_est"):
                        event["tau_est_Nm_unvalidated"] = [float(motors[i].tau_est) for i in range(first,first+7)]
                    if hasattr(msg,"tick"):
                        event["device_timestamp"] = int(msg.tick)
                        event["device_timestamp_unit"] = "tick_unknown_verify_firmware"
                    for field in ("temperature", "motorstate", "mode", "vol"):
                        if hasattr(motors[first],field):
                            event[field] = [getattr(motors[i],field) for i in range(first,first+7)]
                else:
                    for field in ("kp","kd","tau","dq"):
                        event[field] = [float(getattr(motors[i], field)) for i in range(first,first+7)]
                    event["arm_sdk_weight"] = float(motors[31].q) if len(motors)>31 else None
                json.dumps(event, allow_nan=False)
                result.append(event)
            if kind == "state":
                base = dict(kind="auxiliary",channel="h2_base",t_host_s=stamp,
                    waist_q_rad=[float(motors[i].q) for i in range(12,15)],
                    waist_dq_rad_s=[float(motors[i].dq) for i in range(12,15)])
                if len(motors) < len(H2_BODY_JOINTS):
                    raise ValueError("H2 full fixed-body snapshot requires head and all 31 body motors")
                base["joint_positions_rad"] = {name:float(motors[i].q) for i,name in enumerate(H2_BODY_JOINTS)}
                base["joint_velocities_rad_s"] = {name:float(motors[i].dq) for i,name in enumerate(H2_BODY_JOINTS)}
                if hasattr(msg,"imu_state"):
                    imu = msg.imu_state
                    for field in ("quaternion", "gyroscope", "accelerometer", "rpy"):
                        if hasattr(imu,field):
                            base["imu_"+field] = list(getattr(imu,field))
                json.dumps(base,allow_nan=False)
                result.append(base)
            return result
        return decode

    stop_reason = "duration_elapsed"
    try:
        ChannelFactoryInitialize(domain, interface)
        specs = []
        for side in ("left","right"):
            specs.extend([(f"rt/sharpa/{side}/state", HandState_, hand_decoder(side,"state")),
                          (f"rt/sharpa/{side}/cmd", HandCmd_, hand_decoder(side,"command"))])
        if include_h2:
            specs.extend([("rt/lowstate",LowState_,arm_decoder("state")),
                          ("rt/arm_sdk",LowCmd_,arm_decoder("command"))])
        for topic, datatype, decoder in specs:
            sub = ChannelSubscriber(topic, datatype)
            subscribers.append(sub)
            sub.Init(callback(decoder), 100)
        print(f"Passive recording to {dest}; no command publishers created", flush=True)
        with (dest/"events.jsonl").open("x", encoding="utf-8") as f:
            def write_event(event):
                f.write(json.dumps(event,allow_nan=False)+"\n")
                counts[f'{event["channel"]}/{event["kind"]}'] += 1
            try:
                while time.monotonic()-start < duration and not fatal:
                    try:
                        write_event(events.get(timeout=0.1))
                    except queue.Empty:
                        pass
            except KeyboardInterrupt:
                stop_reason = "operator_stopped_recorder_robot_unaffected"
            finally:
                with guard:
                    enabled = False
                while not events.empty():
                    write_event(events.get_nowait())
    except Exception as exc:
        stop_reason = "recording_error"
        fatal.append(f"{type(exc).__name__}: {exc}")
    finally:
        with guard:
            enabled = False
        for sub in subscribers:
            close = getattr(sub,"Close",None)
            if close:
                try:
                    close()
                except Exception as exc:
                    fatal.append(f"close: {exc}")
        if fatal:
            stop_reason = "recording_error"
        dump(dest/"record_status.json",dict(counts=dict(counts), errors=fatal,
            stop_reason=stop_reason, elapsed_s=time.monotonic()-start,
            valid_for_analysis=bool(counts) and not fatal,
            robot_stopped_by_this_script=False))
    if fatal or not counts:
        raise RuntimeError(f"No usable recording: {fatal or 'no DDS samples received'}; see record_status.json")
    return dest
