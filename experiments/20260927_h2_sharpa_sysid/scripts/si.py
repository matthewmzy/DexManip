#!/usr/bin/env python3
"""H2 + Sharpa SI local entry. All output is local; hardware access is passive DDS only.

Examples and reproducible entry points live in ../bashes/. No SSH, robot startup,
controller constructors, publishers, zero poses, or remote writes are performed.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import html
import json
import math
import os
import sys
from pathlib import Path

# The project's native geometry stack is stable with single-threaded BLAS/OMP.
# Set this before NumPy loads when using the optional USD command directly.
if len(sys.argv)>1 and sys.argv[1]=='usd-plan':
    os.environ['OPENBLAS_NUM_THREADS']='1'
    os.environ['OMP_NUM_THREADS']='1'
import numpy as np

from si_core import CHANNELS, PR_SHA, align_events, build_plan, dump, joint_window, priors, template


def initialize(args):
    dest = Path(args.out)
    dest.mkdir(parents=True, exist_ok=False)
    dump(dest/"config.json", template())
    dump(dest/"vendor_priors.json", priors())
    # Header-only files are intentionally not fake measurements.
    for name, columns in {
        "contact.csv": ["normal_force_N", "tangent_force_at_slip_N"],
        "hinge.csv": ["dq_rad_s", "ddq_rad_s2", "tau_Nm"],
        "extrinsics.csv": ["source_x_m", "source_y_m", "source_z_m", "target_x_m", "target_y_m", "target_z_m"],
    }.items():
        with (dest/name).open("x") as f:
            csv.writer(f).writerow(columns)
    print(dest/"config.json")


def plan(args):
    config = json.loads(Path(args.config).read_text())
    result = build_plan(config)
    result["config_sha256"] = hashlib.sha256(Path(args.config).read_bytes()).hexdigest()
    dump(args.out, result)
    print(f"Saved local plan, {len(result['t_s'])} samples, {result['t_s'][-1]:.1f} s. Does not execute motion.")


def draw_svg(path, t, u, q, prediction, title):
    arrays = [u,q] + ([np.asarray(prediction)] if prediction is not None else [])
    lo, hi = min(float(np.min(x)) for x in arrays), max(float(np.max(x)) for x in arrays)
    span = max(hi-lo,1e-5)
    ids = np.linspace(0,len(t)-1,min(len(t),800)).astype(int)
    polylines = []
    for data,color,label in zip(arrays,["#9199a2","#146ac4","#d45525"],["command","measured","free-run prediction"]):
        points = " ".join(f"{55+900*t[i]/t[-1]:.2f},{250-190*(data[i]-lo)/span:.2f}" for i in ids)
        polylines.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.5" points="{points}"/>')
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 300">
<rect width="1000" height="300" fill="white"/><g font-family="sans-serif" font-size="13" fill="#222">
<text x="55" y="22">{html.escape(title)}</text><text x="55" y="44">radians; grey command, blue measured, orange model</text>
<text x="3" y="66">{hi:.3g}</text><text x="3" y="250">{lo:.3g}</text><text x="55" y="280">0 s</text>
<text x="880" y="280">{t[-1]:.1f} s</text></g>{''.join(polylines)}</svg>'''
    with Path(path).open("x") as f:
        f.write(svg)


def analyze(args):
    from si_analysis import fit_servo

    run = Path(args.run)
    session = json.loads((run/"session.json").read_text())
    status_path = run/"record_status.json"
    if session.get("source") == "hardware_dds_passive":
        if not status_path.exists() or not json.loads(status_path.read_text()).get("valid_for_analysis"):
            raise ValueError("Hardware recording incomplete/invalid; inspect record_status.json before analysis")
    events_path = run/"events.jsonl"
    with events_path.open() as f:
        events = [json.loads(line) for line in f if line.strip()]
    if not events:
        raise ValueError("Empty log")
    dest = Path(args.out) if args.out else run/"analysis"
    dest.mkdir(parents=True,exist_ok=False)
    requested = args.channel or [c for c in CHANNELS if any(e.get("channel")==c for e in events)]
    if not requested:
        raise ValueError("No supported command/state channels")
    if args.joint and any(j not in [n for c in requested for n in CHANNELS[c]] for j in args.joint):
        raise ValueError("--joint must match a requested channel's exact joint name")
    report = dict(schema_version=1, source=session.get("source","unknown"),
                  source_log_sha256=hashlib.sha256(events_path.read_bytes()).hexdigest(),
                  hardware_calibration_complete=False, simulation_config_modified=False,
                  model="effective first-order position servo + host-observed delay",
                  physical_kp_kd_inertia_friction_identified=False, channels={},
                  unmeasured=["physical_motor_parameters", "task_contact_validation", "multi_pose_multi_load_repeatability",
                              "camera_latency_and_geometry", "h2_balance_and_mount_compliance"])
    for channel in requested:
        try:
            t,u,q,quality = align_events(events,channel,args.sample_hz)
            # Arm commands matter only while the arm_sdk controller is fully engaged.
            if channel.endswith("arm"):
                weights = [e.get("arm_sdk_weight") for e in events if e.get("channel")==channel and e.get("kind")=="command"]
                if not weights or any(w is None or not math.isfinite(w) or abs(w-1)>1e-3 for w in weights):
                    raise ValueError("H2 arm_sdk weight not fixed at 1; position-only fitting would mix controller blending")
                for field in ("kp","kd"):
                    gains = [e.get(field) for e in events if e.get("channel")==channel and e.get("kind")=="command"]
                    if any(g is None for g in gains) or not np.allclose(gains,gains[0],rtol=1e-5,atol=1e-6):
                        raise ValueError(f"H2 {field} missing/changing; segment by control configuration first")
                for field in ("dq","tau"):
                    feedforward = [e.get(field) for e in events if e.get("channel")==channel and e.get("kind")=="command"]
                    if any(g is None for g in feedforward) or not np.allclose(feedforward,feedforward[0],rtol=1e-5,atol=1e-6):
                        raise ValueError(f"H2 {field} feedforward missing/changing: retain log for multi-input/simulator identification; q-only model refused")
            result = dict(quality=quality,joints={})
            report["channels"][channel] = result
            for j,name in enumerate(CHANNELS[channel]):
                if args.joint and name not in args.joint:
                    continue
                if np.ptp(u[:,j]) < 0.005:
                    result["joints"][name] = dict(status="unidentified_insufficient_excitation",
                        observed_motion_peak_to_peak_rad=float(np.ptp(q[:,j])),
                        note="Non-excited joint motion is useful for coupling inspection, not a fitted servo")
                    continue
                try:
                    window,window_info = joint_window(t,u[:,j],args.max_delay_s)
                    wt = t[window]-t[window][0]
                    wu,wq = u[window,j],q[window,j]
                    fit = fit_servo(wt,wu,wq,max_delay_s=args.max_delay_s)
                    fit["analysis_window"] = window_info
                    prediction = fit.pop("prediction_rad",None)
                    fit["status"] = "equivalent_model_candidate" if fit.get("accepted") else "rejected_model"
                    result["joints"][name] = fit
                    draw_svg(dest/f"{name}.svg",wt,wu,wq,prediction,name)
                except ValueError as exc:
                    result["joints"][name] = dict(status="rejected_fit",reason=str(exc))
        except ValueError as exc:
            report["channels"][channel] = dict(status="rejected_channel",reason=str(exc))
    dump(dest/"report.json",report)
    lines = ["# SI analysis", "",f"Source: `{report['source']}`. Hardware SI is **not complete**.", "",
             "Fits are closed-loop equivalents, not physical Kp/Kd/inertia/friction. Delay includes host timing and bridge effects.","",
             "| Channel / joint | Status | Holdout RMSE (rad) |", "|---|---|---|"]
    for channel,res in report["channels"].items():
        if "reason" in res:
            lines.append(f"| {channel} | rejected: {res['reason']} | — |")
        for name,fit in res.get("joints",{}).items():
            value = fit.get("holdout_rmse_rad",fit.get("holdout_rmse", "—"))
            lines.append(f"| {name} | {fit['status']} | {value} |")
    lines.extend(["", "Input/response/prediction plots:", ""])
    lines.extend(f"![{p.stem}]({p.name})" for p in sorted(dest.glob("*.svg")))
    with (dest/"report.md").open("x") as f:
        f.write("\n".join(lines)+"\n")
    count = sum(bool(j.get("accepted")) for c in report["channels"].values() for j in c.get("joints",{}).values())
    print(f"{dest/'report.md'}; {count} accepted equivalent-model candidates; no simulator settings written")
    return report


def measure(args):
    from si_analysis import fit_contact, fit_hinge, fit_rigid_transform
    with Path(args.input).open() as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError("Measurement table is empty; fill real measurements with the stated units")
    def col(name):
        return np.array([float(row[name]) for row in rows])
    if args.kind == "contact":
        result = fit_contact(col("normal_force_N"), col("tangent_force_at_slip_N"))
    elif args.kind == "hinge":
        result = fit_hinge(col("dq_rad_s"),col("ddq_rad_s2"),col("tau_Nm"))
    else:
        source = np.column_stack([col(f"source_{x}_m") for x in "xyz"])
        target = np.column_stack([col(f"target_{x}_m") for x in "xyz"])
        result = fit_rigid_transform(source,target)
    dump(args.out,dict(kind=args.kind, input_sha256=hashlib.sha256(Path(args.input).read_bytes()).hexdigest(),
                       result=result, status="candidate_requires_independent_validation"))
    print(args.out)


def demo(args):
    dest = Path(args.out)
    dest.mkdir(parents=True,exist_ok=False)
    hz, tau, delay, gain, bias = 50,0.09,0.04,0.97,0.003
    t = np.arange(0,32,1/hz)
    waveform = 0.035*np.sin(2*np.pi*0.31*t)+0.018*np.sin(2*np.pi*0.83*t)+0.01*np.sin(2*np.pi*1.71*t)
    # Distinct holdout waveform, independent of parameter fitting.
    split = int(.7*len(t))
    waveform[split:] = .033*np.sin(2*np.pi*.47*t[split:])+.021*np.sin(2*np.pi*1.23*t[split:])
    u = np.full((len(t),22),0.35)
    u[:,5] += waveform
    q = u.copy()
    a = math.exp(-1/hz/tau)
    q[0,5] = gain*u[0,5]+bias
    for k in range(1,len(t)):
        q[k,5] = a*q[k-1,5]+(1-a)*(gain*u[max(k-1-round(delay*hz),0),5]+bias)
    q[:,5] += np.random.default_rng(8).normal(0,0.00008,len(t))
    dump(dest/"session.json",dict(schema_version=1,source="synthetic_software_test_not_robot_measurement",source_pr_sha=PR_SHA))
    dump(dest/"truth.json",dict(tau_s=tau,delay_s=delay,gain=gain,bias_rad=bias))
    with (dest/"events.jsonl").open("x") as f:
        for k,stamp in enumerate(t):
            for kind,data in (("command",u),("state",q)):
                f.write(json.dumps(dict(kind=kind,channel="right_hand",t_host_s=float(stamp),q_rad=data[k].tolist(),device_timestamp=None))+"\n")
    config = template()
    p = config["plan"]
    p.update(center_rad=[0.35]*22,reviewed_lower_rad=[0.1]*22,reviewed_upper_rad=[0.7]*22,reviewed_clearance=True)
    config["notes"].append("Synthetic plan bounds only; not approved for hardware")
    dump(dest/"synthetic_config.json",config)
    dump(dest/"synthetic_plan.json",build_plan(config))
    report = analyze(argparse.Namespace(run=str(dest),out=None,channel=["right_hand"],joint=None,sample_hz=hz,max_delay_s=.15))
    fit = report["channels"]["right_hand"]["joints"]["right_index_MCP_FE"]
    if not fit.get("accepted"):
        raise RuntimeError("Synthetic acceptance failed; inspect analysis/report.json")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command",required=True)
    p = sub.add_parser("init",help="write unmeasured config and CSV templates")
    p.add_argument("--out",required=True); p.set_defaults(func=initialize)
    p = sub.add_parser("inspect",help="show pinned channel map and vendor priors; no hardware")
    p.set_defaults(func=lambda args: print(json.dumps(dict(source_pr_sha=PR_SHA,channels=CHANNELS,priors=priors()),indent=2)))
    p = sub.add_parser("plan",help="create local bounded plan; never execute motion")
    p.add_argument("--config",required=True); p.add_argument("--out",required=True); p.set_defaults(func=plan)
    p = sub.add_parser("record",help="passively subscribe cmd/state, save local JSONL")
    p.add_argument("--out",required=True); p.add_argument("--interface",required=True)
    p.add_argument("--duration",type=float,default=60); p.add_argument("--domain",type=int,default=0)
    p.add_argument("--config",help="optional operator hardware metadata; copied locally into session")
    p.add_argument("--include-h2",action="store_true")
    def record_command(args):
        from si_dds import record
        return record(args.out,args.interface,args.duration,args.domain,args.include_h2,args.config)
    p.set_defaults(func=record_command)
    p = sub.add_parser("usd-plan",help="USD-only pose/range screening; fix head/waist/legs, table removed")
    p.add_argument("--out",required=True); p.add_argument("--from-run")
    p.add_argument("--joint",action="append",help="exact arm/hand joint; default all 58")
    p.add_argument("--amplitude-deg",type=float,default=2.)
    p.add_argument("--step-deg",type=float,default=.25)
    p.add_argument("--seconds",type=float,default=120.)
    def usd_plan_command(args):
        from si_usd_plan import prepare
        return prepare(args)
    p.set_defaults(func=usd_plan_command)
    p = sub.add_parser("analyze",help="fit held-out response; export candidates and SVG plots")
    p.add_argument("--run",required=True); p.add_argument("--out")
    p.add_argument("--channel",choices=list(CHANNELS),action="append")
    p.add_argument("--joint",action="append"); p.add_argument("--sample-hz",type=float,default=50)
    p.add_argument("--max-delay-s",type=float,default=.15); p.set_defaults(func=analyze)
    p = sub.add_parser("measure",help="analyze fixture measurements / corresponding 3D points")
    p.add_argument("kind",choices=["contact","hinge","extrinsics"])
    p.add_argument("--input",required=True); p.add_argument("--out",required=True); p.set_defaults(func=measure)
    p = sub.add_parser("demo",help="offline known-model pipeline test, synthetic only")
    p.add_argument("--out",required=True); p.set_defaults(func=demo)
    args = parser.parse_args()
    try:
        args.func(args)
    except (ValueError, RuntimeError, OSError, ImportError, KeyError) as exc:
        parser.exit(2,f"SI error: {exc}\n")


if __name__ == "__main__":
    main()
