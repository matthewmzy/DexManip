"""Offline SI pose/range generation from the project's verified USD export.

Only arm/hand joint targets are produced. Head/waist/legs remain fixed geometry.
This module never opens DDS or executes movement. Outputs live in --out.
"""
from __future__ import annotations

import hashlib
import itertools
import json
import math
from pathlib import Path
import time

import numpy as np

from si_core import CHANNELS, H2_BODY_JOINTS, build_plan, dump, template

MOVABLE = tuple(n for channel in CHANNELS.values() for n in channel)
FIXED = tuple(n for n in H2_BODY_JOINTS if n not in MOVABLE)


def nominal_pose(model):
    """Local scene reference, explicitly not the robot's measured start pose."""
    q = {n:0.0 for n,j in model.joints.items() if j['enabled'] and j['type']=='PhysicsRevoluteJoint'}
    for side in ('left','right'):
        q[f'{side}_hip_pitch_joint'] = -.1
        q[f'{side}_knee_joint'] = .3
        q[f'{side}_ankle_pitch_joint'] = -.2
    q['head_pitch_joint'] = .6
    return q


def recorded_pose(run, max_skew_s=.05):
    """Extract a synchronized full pose from passive logs, never invent fixed joints."""
    run = Path(run)
    status = json.loads((run/'record_status.json').read_text())
    if not status.get('valid_for_analysis'):
        raise ValueError('Recording is incomplete/invalid')
    latest = {}
    selected = None
    with (run/'events.jsonl').open() as handle:
        for line in handle:
            event = json.loads(line)
            c = event.get('channel')
            wanted = (c=='h2_base' and event.get('kind')=='auxiliary') or (c in ('left_hand','right_hand') and event.get('kind')=='state')
            if not wanted:
                continue
            latest[c] = event
            if set(latest) != {'h2_base','left_hand','right_hand'}:
                continue
            stamps = [float(e['t_host_s']) for e in latest.values()]
            if not np.isfinite(stamps).all() or max(stamps)-min(stamps)>max_skew_s:
                continue
            base = latest['h2_base'].get('joint_positions_rad')
            if not isinstance(base,dict) or set(base)!=set(H2_BODY_JOINTS):
                continue
            q = dict(base)
            valid = True
            for side in ('left','right'):
                a = latest[f'{side}_hand'].get('q_rad',[])
                if len(a)!=22:
                    valid = False
                    break
                q.update(zip(CHANNELS[f'{side}_hand'],a))
            if valid and np.isfinite(list(q.values())).all():
                selected = (q,dict(kind='recorded_snapshot',run=str(run.resolve()),
                    maximum_receive_skew_s=max(stamps)-min(stamps),host_time_s=max(stamps),
                    note='Recorded pose only; must match a fresh live state before any future execution'))
    if selected is None:
        raise ValueError('Need near-synchronous left/right hand + all 31 H2 body positions; re-record using --include-h2')
    selected[1]['log_sha256']=hashlib.sha256((run/'events.jsonl').read_bytes()).hexdigest()
    return selected


def candidate_poses(start, model):
    """Deterministic small candidate set; 17 non-arm joints never change."""
    yield 'input_pose',dict(start)
    for bend, shoulder, elbow in itertools.product((.15,.25),(.6,.35,.9),(.6,1.0)):
        q = dict(start)
        for side, sign in (('left',1),('right',-1)):
            arm = [shoulder,sign*.18,0.,elbow,0.,0.,0.]
            q.update(zip(CHANNELS[f'{side}_arm'],arm))
            for n in CHANNELS[f'{side}_hand']:
                low,high = model.joints[n]['limits_radians']
                value = 0. if n.endswith('_AA') else bend
                if n.endswith('thumb_CMC_AA'):
                    value = -.15
                q[n] = float(np.clip(value,low+.04,high-.04))
        yield f'bend_{bend}_shoulder_{shoulder}_elbow_{elbow}',q


def pose_segment(a,b,step_rad):
    if set(a)!=set(b) or set(a)!=set(MOVABLE)|set(FIXED) or not math.isfinite(step_rad) or step_rad<=0:
        raise ValueError('Segment requires equal complete joint maps and positive finite step')
    if not np.isfinite(list(a.values())+list(b.values())).all():
        raise ValueError('Segment joint positions must be finite')
    if any(a[n]!=b[n] for n in FIXED):
        raise ValueError('Head/waist/leg movement is prohibited in SI planning')
    distance = max(abs(float(b[n])-float(a[n])) for n in a)
    count = max(1,int(math.ceil(distance/step_rad)))
    for k in range(count+1):
        alpha = k/count
        # Preserve fixed floating values exactly, not just within interpolation tolerance.
        yield {n:a[n] if n in FIXED else (1-alpha)*a[n]+alpha*b[n] for n in a}


def check_segment(audit,a,b,step_rad,deadline):
    checked = 0
    for q in pose_segment(a,b,step_rad):
        if time.monotonic()>=deadline:
            return dict(status='unknown',reason='total_time_budget',checked_samples=checked)
        result = audit.check_pose(q,deadline=deadline)
        checked += 1
        if result['status']!='pass':
            return dict(status=result['status'],checked_samples=checked,failure=result)
    return dict(status='pass',checked_samples=checked,
                scope='dense sampled joint interpolation, not continuous collision detection or dynamic simulation')


def prepare(args):
    from si_usd_geometry import SIUSDGeometry
    if not math.isfinite(args.seconds) or args.seconds<=0:
        raise ValueError('--seconds must be finite positive')
    if not math.isfinite(args.amplitude_deg) or not 0<args.amplitude_deg<=10:
        raise ValueError('--amplitude-deg must be in (0,10]')
    if not math.isfinite(args.step_deg) or not 0<args.step_deg<=.5:
        raise ValueError('--step-deg must be in (0,0.5]')
    selected = args.joint or list(MOVABLE)
    if len(set(selected))!=len(selected) or any(n not in MOVABLE for n in selected):
        raise ValueError('Only unique arm/hand joint names may be excited')
    out = Path(args.out)
    out.mkdir(parents=True,exist_ok=False)
    audit = SIUSDGeometry()
    if args.from_run:
        start,source = recorded_pose(args.from_run)
    else:
        start = nominal_pose(audit.model)
        source = dict(kind='nominal_usd_scene_reference_not_measured',
                      basis='h2_sharpa.py init_state: hips=-.1, knees=.3, ankles=-.2, head pitch=.6')
    fixed = {n:start[n] for n in FIXED}
    report = dict(schema_version=1,source_pose=source,planning_conditions=dict(
        table_removed=True,objects_in_hands=False,head_waist_legs_fixed=True,
        fixed_joint_positions_rad=fixed,movable_joint_names=list(MOVABLE)),
        geometry=audit.manifest(),executable=False,hardware_executed=False,
        nominal_start_to_live_robot_not_assumed=True,candidates=[],trials=[],status='unknown')
    deadline = time.monotonic()+args.seconds
    center = None
    for label,q in candidate_poses(start,audit.model):
        if time.monotonic()>=deadline:
            break
        result = audit.check_pose(q,deadline=deadline)
        report['candidates'].append(dict(label=label,result=result))
        if result['status']=='pass':
            # A nominal scene pose is not a claim about the actual starting robot.
            transition = check_segment(audit,start,q,math.radians(args.step_deg),deadline) if args.from_run else dict(
                status='not_checked_no_measured_start',reason='Offline candidate generation; real entry path requires --from-run')
            report['candidates'][-1]['transition']=transition
            if transition['status']=='pass' or not args.from_run:
                center = q
                report['selected_center_label']=label
                break
    if center is not None:
        dump(out/'center.json',dict(joint_positions_rad=center,source_pose=source,
                                   fixed_joint_positions_rad=fixed,executable=False))
        for name in selected:
            if time.monotonic()>=deadline:
                report['trials'].append(dict(joint=name,status='unknown',reason='total_time_budget'))
                continue
            low,high = audit.model.joints[name]['limits_radians']
            amp = min(math.radians(args.amplitude_deg),center[name]-low-.005,high-center[name]-.005)
            if amp<math.radians(.25):
                report['trials'].append(dict(joint=name,status='rejected',reason='insufficient_USD_limit_margin'))
                continue
            a,b = dict(center),dict(center)
            a[name] -= amp; b[name] += amp
            result = check_segment(audit,a,b,math.radians(args.step_deg),deadline)
            trial = dict(joint=name,amplitude_rad=amp,status=result['status'],geometry_check=result)
            report['trials'].append(trial)
            if result['status']!='pass':
                continue
            channel = next(c for c,names in CHANNELS.items() if name in names)
            config = template()
            p = config['plan'];names=CHANNELS[channel]
            # A checked one-joint interval is not permission to move the rest
            # of this hand/arm anywhere inside their full USD limits.
            interval = [amp if n==name else 1e-9 for n in names]
            p.update(channel=channel,joint_names=names,center_rad=[center[n] for n in names],
                reviewed_lower_rad=[center[n]-width for n,width in zip(names,interval)],
                reviewed_upper_rad=[center[n]+width for n,width in zip(names,interval)],
                selected_indices=[names.index(name)],amplitude_rad=amp,reviewed_clearance=True)
            config['usd_planning'] = dict(center_joint_positions_rad=center,fixed_joint_positions_rad=fixed,
                scope='single joint only; reviewed_clearance is this sampled USD geometry check, not hardware validation',
                checked_joint=name,checked_amplitude_rad=amp,
                source_pose=source,geometry_report='../../usd_report.json')
            plan = build_plan(config)
            plan['usd_sampled_range_check']=result
            plan['usd_fixed_joint_positions_rad']=fixed
            plan['held_joint_positions_rad']={n:value for n,value in center.items() if n!=name}
            plan['status']='USD_sampled_geometry_checked_local_plan_only'
            trial['directory']=f'trials/{name}'
            dump(out/trial['directory']/'config.json',config)
            dump(out/trial['directory']/'plan.json',plan)
        report['status']='pass' if all(r['status']=='pass' for r in report['trials']) else 'partial'
    else:
        report['status']='no_admitted_center_or_transition'
    report['elapsed_geometry_s']=args.seconds-max(0.,deadline-time.monotonic())
    report['limitations']=['Discrete geometric screening only; unknown never treated as pass.',
        'Only the one selected joint may move in each trial; other arms/hands must hold the exported center.',
        'Fixed head/waist/leg values must match the real test; this tool does not command or enforce body control.',
        'No table/object collision geometry is included, per the user-defined empty workspace.',
        'No real-robot motion executor is present.']
    dump(out/'usd_report.json',report)
    print(f"{out/'usd_report.json'}: {report['status']}; {sum(r['status']=='pass' for r in report['trials'])}/{len(selected)} joint trials admitted; no hardware commands",flush=True)
    return report
