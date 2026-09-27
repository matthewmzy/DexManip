"""Offline SI invariants; no controller imports or robot connections.

The cached-USD integration class requires geometry packages and the private asset bundle.
Missing optional geometry dependencies skip only that class, never substitute
another collision implementation. Other tests use explicit fake geometry.
"""
from contextlib import redirect_stdout
import copy
import hashlib
import importlib.util
import io
from itertools import combinations
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest
from unittest import mock

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import si_core
import si_dds
import si_usd_plan as planning


def fake_model():
    names = tuple(si_core.H2_BODY_JOINTS) + tuple(si_core.CHANNELS["left_hand"]) + tuple(si_core.CHANNELS["right_hand"])
    return types.SimpleNamespace(joints={name: dict(name=name, enabled=True,
        type="PhysicsRevoluteJoint", limits_radians=[-2.0, 2.0]) for name in names})


def snapshot_events():
    return [dict(kind="auxiliary", channel="h2_base", t_host_s=10.0,
                 joint_positions_rad={name: i / 100 for i, name in enumerate(si_core.H2_BODY_JOINTS)}),
            dict(kind="state", channel="left_hand", t_host_s=10.01, q_rad=[0.1] * 22),
            dict(kind="state", channel="right_hand", t_host_s=10.02, q_rad=[0.2] * 22)]


def write_record(folder, events, valid=True):
    (folder / "record_status.json").write_text(json.dumps(dict(valid_for_analysis=valid)))
    (folder / "events.jsonl").write_text("\n".join(json.dumps(event) for event in events) + "\n")


class FixedBodyAndSnapshotTests(unittest.TestCase):
    def test_only_arms_and_hands_are_movable(self):
        self.assertEqual(len(si_core.H2_BODY_JOINTS), 31)
        self.assertEqual(len(planning.MOVABLE), 58)
        self.assertEqual(len(planning.FIXED), 17)
        self.assertFalse(set(planning.MOVABLE) & set(planning.FIXED))
        self.assertEqual(set(planning.MOVABLE) | set(planning.FIXED), set(fake_model().joints))
        self.assertTrue(all(any(part in name for part in ("hip", "knee", "ankle", "waist", "head"))
                            for name in planning.FIXED))

    def test_full_pose_and_candidates_preserve_every_fixed_value(self):
        model = fake_model()
        start = planning.nominal_pose(model)
        self.assertEqual(len(start), 75)
        for index, name in enumerate(planning.FIXED):
            start[name] = (index + 1) / 137
        original = dict(start)
        candidates = list(planning.candidate_poses(start, model))
        self.assertGreater(len(candidates), 1)
        for label, q in candidates:
            with self.subTest(candidate=label):
                self.assertEqual(set(q), set(start))
                self.assertEqual({n: q[n].hex() for n in planning.FIXED},
                                 {n: start[n].hex() for n in planning.FIXED})
        self.assertEqual(start, original, "candidate generation must not mutate the recorded pose")

    def test_interpolation_keeps_fixed_body_bit_exact_at_every_sample(self):
        start = planning.nominal_pose(fake_model())
        for index, name in enumerate(planning.FIXED):
            start[name] = (index + 1) / 137
        start[planning.FIXED[0]] = -0.0
        end = dict(start)
        end[si_core.CHANNELS["left_arm"][0]] += 0.31
        end[si_core.CHANNELS["right_hand"][5]] -= 0.23
        samples = list(planning.pose_segment(start, end, 0.017))
        self.assertEqual(samples[0], start)
        self.assertEqual(samples[-1], end)
        for q in samples:
            self.assertEqual(len(q), 75)
            self.assertEqual({n: q[n].hex() for n in planning.FIXED},
                             {n: start[n].hex() for n in planning.FIXED})
        steps = np.diff([[q[n] for n in planning.MOVABLE] for q in samples], axis=0)
        self.assertLessEqual(float(np.abs(steps).max()), 0.017 + 1e-12)

    def test_interpolation_rejects_fixed_movement_incomplete_and_nonfinite_maps(self):
        start = planning.nominal_pose(fake_model())
        end = dict(start)
        end["head_yaw_joint"] = 0.001
        cases = [(start, end, 0.01), (start, start, 0), (start, start, -0.01),
                 (start, start, float("nan"))]
        missing = dict(start)
        del missing[si_core.CHANNELS["right_hand"][0]]
        cases.append((missing, missing, 0.01))
        extra = dict(start, invented_joint=0.0)
        cases.append((extra, extra, 0.01))
        for value in (float("nan"), float("inf")):
            invalid = dict(start)
            invalid[si_core.CHANNELS["left_arm"][0]] = value
            cases.append((invalid, invalid, 0.01))
        for a, b, step in cases:
            with self.subTest(keys=len(a), step=step):
                with self.assertRaises(ValueError):
                    list(planning.pose_segment(a, b, step))

    def test_recorded_snapshot_has_all_75_values_and_source_hash(self):
        with tempfile.TemporaryDirectory() as folder:
            run = Path(folder)
            events = snapshot_events()
            write_record(run, events)
            q, metadata = planning.recorded_pose(run)
            self.assertEqual(set(q), set(fake_model().joints))
            for name, value in events[0]["joint_positions_rad"].items():
                self.assertEqual(q[name], value)
            self.assertEqual([q[n] for n in si_core.CHANNELS["left_hand"]], [0.1] * 22)
            self.assertEqual([q[n] for n in si_core.CHANNELS["right_hand"]], [0.2] * 22)
            self.assertAlmostEqual(metadata["maximum_receive_skew_s"], 0.02)
            self.assertEqual(metadata["log_sha256"], hashlib.sha256((run / "events.jsonl").read_bytes()).hexdigest())
            self.assertEqual(metadata["kind"], "recorded_snapshot")

    def test_recorded_snapshot_rejects_missing_async_and_nonfinite_states(self):
        cases = {}
        cases["no_right_hand"] = snapshot_events()[:2]
        cases["missing_body_joint"] = snapshot_events()
        del cases["missing_body_joint"][0]["joint_positions_rad"]["head_yaw_joint"]
        cases["extra_body_joint"] = snapshot_events()
        cases["extra_body_joint"][0]["joint_positions_rad"]["invented_joint"] = 0.0
        cases["short_hand"] = snapshot_events()
        cases["short_hand"][1]["q_rad"].pop()
        cases["asynchronous"] = snapshot_events()
        cases["asynchronous"][2]["t_host_s"] = 10.06
        cases["nan_body"] = snapshot_events()
        cases["nan_body"][0]["joint_positions_rad"]["head_yaw_joint"] = float("nan")
        cases["nan_hand"] = snapshot_events()
        cases["nan_hand"][1]["q_rad"][4] = float("nan")
        cases["nan_time"] = snapshot_events()
        cases["nan_time"][2]["t_host_s"] = float("nan")
        for label, events in cases.items():
            with self.subTest(case=label), tempfile.TemporaryDirectory() as folder:
                run = Path(folder)
                write_record(run, events)
                with self.assertRaises(ValueError):
                    planning.recorded_pose(run)
        with tempfile.TemporaryDirectory() as folder:
            write_record(Path(folder), snapshot_events(), valid=False)
            with self.assertRaisesRegex(ValueError, "invalid"):
                planning.recorded_pose(folder)


class AdmissionTests(unittest.TestCase):
    def run_prepare(self, out, statuses, joint=None):
        audit = mock.Mock(model=fake_model())
        audit.manifest.return_value = dict(test_geometry=True)
        audit.check_pose.side_effect = lambda *a, **kw: dict(status=next(statuses))
        module = types.ModuleType("si_usd_geometry")
        module.SIUSDGeometry = mock.Mock(return_value=audit)
        args = types.SimpleNamespace(out=str(out), seconds=10., amplitude_deg=1., step_deg=0.5,
            from_run=None, joint=[si_core.CHANNELS["right_hand"][5]] if joint is None else joint)
        with mock.patch.dict(sys.modules, {"si_usd_geometry": module}), \
                mock.patch.object(planning, "candidate_poses", side_effect=lambda start, model: [("input", start)]), \
                redirect_stdout(io.StringIO()):
            report = planning.prepare(args)
        return report, audit

    def test_unknown_collision_or_timeout_never_admit_a_segment(self):
        start = planning.nominal_pose(fake_model())
        end = dict(start)
        end[si_core.CHANNELS["left_arm"][0]] = 0.1
        for status in ("unknown", "collision", "limits"):
            audit = mock.Mock()
            audit.check_pose.side_effect = [dict(status="pass"), dict(status=status)]
            result = planning.check_segment(audit, start, end, .01, time.monotonic() + 10)
            self.assertEqual(result["status"], status)
            self.assertEqual(audit.check_pose.call_count, 2)
        audit = mock.Mock()
        result = planning.check_segment(audit, start, end, .01, time.monotonic() - 1)
        self.assertEqual(result["status"], "unknown")
        audit.check_pose.assert_not_called()

    def test_no_plan_when_center_or_transition_or_trial_does_not_pass(self):
        for stage in ("center", "transition", "trial"):
            for status in ("unknown", "collision"):
                prefix = {"center": [], "transition": ["pass"], "trial": ["pass"] * 3}[stage]
                with self.subTest(stage=stage, status=status), tempfile.TemporaryDirectory() as folder:
                    out = Path(folder) / "result"
                    report, _ = self.run_prepare(out, iter(prefix + [status]))
                    self.assertNotEqual(report["status"], "pass")
                    self.assertEqual(list(out.rglob("plan.json")), [])
                    self.assertFalse(report["executable"])
                    self.assertFalse(report["hardware_executed"])

    def test_admitted_plan_is_single_joint_and_keeps_full_fixed_body_record(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / "result"
            report, audit = self.run_prepare(out, iter(["pass"] * 100))
            self.assertEqual(report["status"], "pass")
            paths = list(out.rglob("plan.json"))
            self.assertEqual(len(paths), 1)
            plan = json.loads(paths[0].read_text())
            self.assertFalse(plan["executable"])
            self.assertEqual(set(plan["usd_fixed_joint_positions_rad"]), set(planning.FIXED))
            center = json.loads((out / "center.json").read_text())["joint_positions_rad"]
            self.assertEqual(len(center), 75)
            selected_name = si_core.CHANNELS["right_hand"][5]
            self.assertEqual(plan["held_joint_positions_rad"],
                             {name: value for name, value in center.items() if name != selected_name})
            self.assertEqual(len(plan["held_joint_positions_rad"]), 74)
            values = np.asarray(plan["q_command_rad"])
            changed = np.any(np.abs(values - np.asarray([center[n] for n in plan["joint_names"]])) > 1e-12, axis=0)
            self.assertEqual(np.flatnonzero(changed).tolist(), [5])
            for call in audit.check_pose.call_args_list:
                self.assertEqual(len(call.args[0]), 75)
                self.assertEqual({n: call.args[0][n] for n in planning.FIXED},
                                 {n: center[n] for n in planning.FIXED})

    def test_usd_certificate_reuse_rejects_center_joint_or_range_expansion(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / "result"
            self.run_prepare(out, iter(["pass"] * 100))
            config = json.loads(next(out.rglob("config.json")).read_text())
            original = si_core.build_plan(config)
            self.assertFalse(original["executable"])
            p = config["plan"]
            selected = p["selected_indices"][0]
            center = np.asarray(p["center_rad"])
            expected_widths = np.full(len(center), 1e-9)
            expected_widths[selected] = p["amplitude_rad"]
            np.testing.assert_allclose(p["reviewed_lower_rad"], center - expected_widths, atol=0, rtol=0)
            np.testing.assert_allclose(p["reviewed_upper_rad"], center + expected_widths, atol=0, rtol=0)

            variants = {}
            variants["center"] = copy.deepcopy(config)
            # Remains within the narrow held interval: the certificate itself
            # must reject even a center change that interval checks would admit.
            variants["center"]["plan"]["center_rad"][0] += 1e-10
            variants["joint"] = copy.deepcopy(config)
            variants["joint"]["plan"]["selected_indices"] = [selected + 1]
            variants["amplitude"] = copy.deepcopy(config)
            variants["amplitude"]["plan"]["amplitude_rad"] += 1e-12
            for change, altered in variants.items():
                with self.subTest(change=change), self.assertRaisesRegex(ValueError, "USD-checked"):
                    si_core.build_plan(altered)

            smaller = copy.deepcopy(config)
            smaller["plan"]["amplitude_rad"] *= .5
            reduced = si_core.build_plan(smaller)
            offsets = np.asarray(reduced["q_command_rad"]) - center
            self.assertLessEqual(float(np.abs(offsets[:, selected]).max()), smaller["plan"]["amplitude_rad"] + 1e-12)
            self.assertEqual(np.flatnonzero(np.any(np.abs(offsets) > 1e-12, axis=0)).tolist(), [selected])

    def test_fixed_or_duplicate_joints_are_refused_before_planning(self):
        for selection in (["head_yaw_joint"], ["waist_yaw_joint"],
                          [si_core.CHANNELS["left_arm"][0]] * 2):
            with self.subTest(joints=selection), tempfile.TemporaryDirectory() as folder:
                out = Path(folder) / "result"
                with self.assertRaisesRegex(ValueError, "arm/hand"):
                    self.run_prepare(out, iter([]), joint=selection)
                self.assertFalse(out.exists())


class BodyDDSCaptureTests(unittest.TestCase):
    def test_passive_dds_retains_all_31_body_positions_and_velocities(self):
        # Reuse the pre-existing explicit fake SDK; the real unitree package is
        # never imported, and creation of a publisher raises in this fixture.
        import test_pipeline
        motors = [types.SimpleNamespace(q=(i + 1) * .01, dq=(i + 1) * -.02) for i in range(35)]
        commands = [types.SimpleNamespace(q=0., dq=0., tau=0., kp=40., kd=1.) for _ in range(35)]
        messages = {"rt/lowstate": types.SimpleNamespace(motor_state=motors, tick=123),
                    "rt/arm_sdk": types.SimpleNamespace(motor_cmd=commands)}
        modules, channel, subscribers = test_pipeline.PassiveDDSTests().fake_sdk(h2_messages=messages)
        ticks = iter(np.arange(0, 10, .001))
        with tempfile.TemporaryDirectory() as folder, mock.patch.dict(sys.modules, modules), \
                mock.patch.object(si_dds.time, "monotonic", side_effect=lambda: next(ticks)), redirect_stdout(io.StringIO()):
            dest = si_dds.record(Path(folder) / "capture", "fake0", .004, include_h2=True)
            events = [json.loads(line) for line in (dest / "events.jsonl").read_text().splitlines()]
            body = next(e for e in events if e["channel"] == "h2_base")
            self.assertEqual(body["joint_positions_rad"], {n: motors[i].q for i, n in enumerate(si_core.H2_BODY_JOINTS)})
            self.assertEqual(body["joint_velocities_rad_s"], {n: motors[i].dq for i, n in enumerate(si_core.H2_BODY_JOINTS)})
            q, _ = planning.recorded_pose(dest)
            self.assertEqual(len(q), 75)
            self.assertEqual(q["head_yaw_joint"], motors[30].q)
        channel.ChannelPublisher.assert_not_called()
        self.assertTrue(all(sub.closed for sub in subscribers))


GEOMETRY_DEPENDENCIES = ("scipy", "rtree", "trimesh")
HAS_GEOMETRY_DEPENDENCIES = all(importlib.util.find_spec(name) is not None for name in GEOMETRY_DEPENDENCIES)


GEOMETRY_ASSET_FILES = (
    "H2_with_sharpa_flat.usd", "model.json", "geometry.npz",
    "cooked89/export_manifest.json", "cooked89/results.json", "cooked89/cooked_shapes.npz",
)
HAS_GEOMETRY_ASSETS = all((si_core.ROOT / "assets/h2_sharpa" / name).is_file()
                          for name in GEOMETRY_ASSET_FILES)


@unittest.skipUnless(HAS_GEOMETRY_DEPENDENCIES and HAS_GEOMETRY_ASSETS,
    "cached USD integration requires scipy/rtree/trimesh and the private assets/h2_sharpa bundle")
class CachedUSDIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from si_usd_geometry import SIUSDGeometry
        cls.audit = SIUSDGeometry()

    def test_manifest_tracks_current_source_and_has_complete_bilateral_pair_scope(self):
        audit = self.audit
        manifest = audit.manifest()
        self.assertEqual(manifest["USD_sha256"], hashlib.sha256(Path(manifest["source_usd"]).read_bytes()).hexdigest())
        self.assertEqual(len(audit.revolute_joint_names), 75)
        self.assertEqual(set(manifest["fixed_joint_names"]), set(planning.FIXED))
        moving = set(audit.moving_body_names)
        disabled = {frozenset(row["bodies"]) for row in audit.joint_exclusions}
        all_names = {body["name"] for body in audit.model.bodies}
        expected = {frozenset((a, b)) for a, b in combinations(all_names, 2)
                    if (a in moving or b in moving) and frozenset((a, b)) not in disabled}
        actual = {frozenset(row["bodies"]) for row in manifest["body_pairs"]}
        self.assertEqual(actual, expected)
        for pair in (("left_hand_C_MC", "right_hand_C_MC"),
                     ("left_index_DP", "right_index_DP"),
                     ("left_elbow_link", "right_elbow_link"),
                     ("left_index_DP", "left_middle_DP"),
                     ("right_index_DP", "right_middle_DP"),
                     ("left_hand_C_MC", "head_yaw_link"),
                     ("right_hand_C_MC", "torso_link")):
            self.assertIn(frozenset(pair), actual)

    def test_missing_extra_and_out_of_limit_joint_states_are_rejected(self):
        q = planning.nominal_pose(self.audit.model)
        self.assertEqual(len(q), 75)
        missing = dict(q)
        del missing["head_yaw_joint"]
        for invalid in (missing, dict(q, invented_joint=0.)):
            with self.assertRaisesRegex(ValueError, "every USD revolute joint"):
                self.audit.check_pose(invalid)
        name = si_core.CHANNELS["right_hand"][5]
        for value in (self.audit.model.joints[name]["limits_radians"][1] + .01, float("nan")):
            result = self.audit.check_pose(dict(q, **{name: value}))
            self.assertEqual(result["status"], "limits")
            self.assertFalse(result["complete_pair_check"])

    def test_complete_triangle_index_matches_exhaustive_aabb_candidates(self):
        from si_usd_geometry import GUARD
        asset = self.audit._triangles("right_wrist_yaw_link")
        triangles, tree = asset["triangles"], asset["tree"]
        low, high = triangles.min(1) - GUARD, triangles.max(1) + GUARD
        self.assertEqual(len(tree), len(triangles))
        queries = [(low[i], high[i]) for i in (0, len(triangles) // 2, len(triangles) - 1)]
        queries += [(low.min(0), high.max(0)), (high.max(0) + 1., high.max(0) + 2.)]
        for query_low, query_high in queries:
            exhaustive = np.flatnonzero(np.all(high >= query_low, axis=1)
                                       & np.all(low <= query_high, axis=1)).tolist()
            indexed = sorted(tree.intersection(tuple(np.r_[query_low, query_high])))
            self.assertEqual(indexed, exhaustive)

    def test_model_source_hash_mismatch_fails_before_using_geometry(self):
        from usd_model27 import USDModel
        manifest = copy.deepcopy(self.audit.model.manifest)
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            source = folder / "different.usd"
            source.write_text("not the USD named by the existing manifest")
            manifest["source_asset"]["path"] = str(source)
            (folder / "model.json").write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, "USD changed"):
                USDModel(folder, source_usd=source)

    def test_bilateral_thumb_collision_witness_and_separated_full_pose(self):
        q = {name: 0.0 for name in self.audit.revolute_joint_names}
        for side, sign in (("left", 1), ("right", -1)):
            q.update(zip(si_core.CHANNELS[f"{side}_arm"], [.6, sign * .18, 0., .6, 0., 0., 0.]))
            for name in si_core.CHANNELS[f"{side}_hand"]:
                q[name] = 0.0 if name.endswith("_AA") else .15
            q[f"{side}_thumb_CMC_AA"] = -.15
        result = self.audit.check_pose(q, deadline=time.monotonic() + 30.)
        self.assertEqual(result["cooked_status"], "pass", result)
        self.assertEqual(result["supplemental_visual_status"], "pass", result)
        self.assertEqual(result["status"], "pass", result)
        self.assertTrue(result["complete_pair_check"])

        for side in ("left", "right"):
            q[f"{side}_thumb_CMC_AA"] = 0.0
        collision = self.audit.check_pose(q, deadline=time.monotonic() + 30., first_failure=False)
        self.assertEqual(collision["status"], "collision", collision)
        self.assertEqual(collision["cooked_status"], "collision", collision)
        self.assertFalse(collision["complete_pair_check"])
        pairs = {frozenset(issue["bodies"]) for issue in collision["cooked_issues"]
                 if issue["status"] == "collision"}
        for side in ("left", "right"):
            self.assertIn(frozenset((f"{side}_hand_C_MC", f"{side}_thumb_MC")), pairs)


if __name__ == "__main__":
    unittest.main()
