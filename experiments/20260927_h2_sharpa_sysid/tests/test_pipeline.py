"""Protocol, passive-recorder, and analysis guard tests; fake DDS only."""

from contextlib import redirect_stdout
import copy
import io
import json
from pathlib import Path
import queue
import sys
import tempfile
import types
import unittest
from unittest import mock

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import si_core
import si_dds
import si


def hand_message(kind, values):
    motors = [types.SimpleNamespace(q=value) for value in values]
    return types.SimpleNamespace(**{("motor_state" if kind == "state" else "motor_cmd"): motors})


def regular_events(duration=6.0, hz=50):
    events = []
    for stamp in np.arange(0, duration, 1 / hz):
        for kind in ("command", "state"):
            q = np.zeros(22)
            q[5] = 0.03 * np.sin(stamp * 4)
            events.append(dict(kind=kind, channel="right_hand", t_host_s=float(stamp), q_rad=q.tolist()))
    return events


def regular_arm_events():
    events = regular_events()
    for event in events:
        event["channel"] = "right_arm"
        event["q_rad"] = event["q_rad"][:7]
        if event["kind"] == "state":
            event["device_timestamp"] = 1000 + round(event["t_host_s"] * 1000)
        else:
            event.update(arm_sdk_weight=1.0, kp=[40.0] * 7, kd=[1.0] * 7,
                         dq=[0.0] * 7, tau=[0.0] * 7)
    return events


class ProtocolTests(unittest.TestCase):
    def test_hand_units_and_order(self):
        degrees = np.linspace(-60, 60, 22)
        state = si_core.hand_event(hand_message("state", degrees), "right", "state", 0.2)
        command = si_core.hand_event(hand_message("command", np.deg2rad(degrees)), "right", "command", 0.1)
        np.testing.assert_allclose(state["q_rad"], command["q_rad"], atol=1e-15)
        self.assertEqual(state["raw_q_unit"], "degrees")
        self.assertEqual(command["raw_q_unit"], "radians")
        self.assertIsNone(state["device_timestamp"])

    def test_malformed_hand_messages_refused(self):
        for length in (0, 21, 23, 35):
            with self.assertRaises(ValueError):
                si_core.hand_event(hand_message("state", [0] * length), "left", "state", 0)
        nonfinite = [0] * 22
        nonfinite[10] = np.nan
        with self.assertRaises(ValueError):
            si_core.hand_event(hand_message("command", nonfinite), "left", "command", 0)
        missing_q = types.SimpleNamespace(motor_state=[types.SimpleNamespace() for _ in range(22)])
        with self.assertRaises((ValueError, AttributeError)):
            si_core.hand_event(missing_q, "left", "state", 0)

    def test_plan_requires_known_bounds_and_rejects_speed(self):
        config = si_core.template()
        with self.assertRaises(ValueError):
            si_core.build_plan(config)
        p = config["plan"]
        p.update(center_rad=[0.3] * 22, reviewed_lower_rad=[0.1] * 22,
                 reviewed_upper_rad=[0.7] * 22, reviewed_clearance=True)
        p["max_command_speed_rad_s"] = 0.001
        with self.assertRaisesRegex(ValueError, "speed"):
            si_core.build_plan(config)
        p["max_command_speed_rad_s"] = 0.35
        p["reviewed_lower_rad"][5] = 0.299
        with self.assertRaisesRegex(ValueError, "interval"):
            si_core.build_plan(config)

    def test_plan_never_excites_multiple_joints_at_once(self):
        config = si_core.template()
        config["plan"].update(center_rad=[0.3] * 22, reviewed_lower_rad=[0.1] * 22,
                              reviewed_upper_rad=[0.7] * 22, reviewed_clearance=True,
                              selected_indices=[5, 9])
        plan = si_core.build_plan(config)
        self.assertFalse(plan["executable"])
        q = np.asarray(plan["q_command_rad"])
        changed = np.abs(q - 0.3) > 1e-12
        self.assertTrue(np.all(np.sum(changed, axis=1) <= 1))
        self.assertEqual(set(np.flatnonzero(np.any(changed, axis=0))), {5, 9})
        np.testing.assert_allclose(q[[0, -1]], 0.3, atol=1e-12)

    def test_plan_review_requires_boolean_true(self):
        config = si_core.template()
        config["plan"].update(center_rad=[0.3] * 22, reviewed_lower_rad=[0.1] * 22,
                              reviewed_upper_rad=[0.7] * 22)
        for value in ("false", "true", 1, [True], None):
            with self.subTest(value=value):
                config["plan"]["reviewed_clearance"] = value
                with self.assertRaisesRegex(ValueError, "reviewed_clearance"):
                    si_core.build_plan(config)


class AlignmentTests(unittest.TestCase):
    def test_resampling_units_alignment_and_quality(self):
        events = regular_events()
        for event in events:
            if event["kind"] == "command":
                event["t_host_s"] += 0.01
                event["q_rad"] = [event["t_host_s"]] * 22
            else:
                event["q_rad"] = [2 * event["t_host_s"]] * 22
        t, command, measured, quality = si_core.align_events(events, "right_hand", 50)
        self.assertEqual(t[0], 0)
        # State is linearly interpolated to the first command at host 10 ms.
        np.testing.assert_allclose(measured[:, 5], 2 * (t + 0.01), atol=1e-12)
        self.assertAlmostEqual(command[0, 5], 0.01)
        self.assertIn("NOT wire/actuator latency", quality["alignment"])
        self.assertEqual(quality["state"]["source_freshness"], "unknown_no_device_timestamp")
        self.assertEqual(quality["alignment_delay_resolution_floor_s"], 0.02)
        self.assertIn("not timing accuracy", quality["delay_caveat"])

    def test_dropouts_missing_commands_and_order_refused(self):
        events = regular_events()
        dropouts = [e for e in events if e["kind"] != "state" or not 1 < e["t_host_s"] < 1.2]
        missing_commands = [e for e in events if e["kind"] != "command"]
        duplicate = copy.deepcopy(events)
        duplicate[40]["t_host_s"] = duplicate[38]["t_host_s"]
        malformed = copy.deepcopy(events)
        malformed[20]["q_rad"] = [0] * 21
        for dataset in (dropouts, missing_commands, duplicate, malformed):
            with self.subTest(size=len(dataset)):
                with self.assertRaises(ValueError):
                    si_core.align_events(dataset, "right_hand", 50)
        with self.assertRaises(ValueError):
            si_core.align_events(events, "right_hand", 100)

    def test_h2_missing_repeated_or_backwards_tick_refused(self):
        events = regular_arm_events()
        _, _, _, quality = si_core.align_events(events, "right_arm", 50)
        self.assertEqual(quality["state"]["source_freshness"], "monotonic_tick_verified_unit_unknown")
        for change in ("missing", "repeated", "backwards", "nonfinite", "fractional", "boolean"):
            dataset = copy.deepcopy(events)
            states = [e for e in dataset if e["kind"] == "state"]
            states[10]["device_timestamp"] = {
                "missing": None,
                "repeated": states[9]["device_timestamp"],
                "backwards": states[9]["device_timestamp"] - 1,
                "nonfinite": float("nan"), "fractional": 1234.5, "boolean": True,
            }[change]
            with self.subTest(change=change):
                with self.assertRaisesRegex(ValueError, "tick"):
                    si_core.align_events(dataset, "right_arm", 50)


class AnalyzeGuardTests(unittest.TestCase):
    def test_variable_or_missing_h2_feedforward_refuses_q_only_fit(self):
        for field in ("dq", "tau"):
            for change in ("variable", "missing"):
                with self.subTest(field=field, change=change), tempfile.TemporaryDirectory() as directory:
                    run = Path(directory)
                    events = regular_arm_events()
                    command = next(e for e in events if e["kind"] == "command" and e["t_host_s"] > 3)
                    if change == "variable":
                        command[field][0] = 0.1
                    else:
                        del command[field]
                    (run / "session.json").write_text(json.dumps({"source": "synthetic_guard_test"}))
                    (run / "events.jsonl").write_text("\n".join(json.dumps(e) for e in events))
                    args = types.SimpleNamespace(run=str(run), out=None, channel=["right_arm"],
                                                 joint=None, sample_hz=50, max_delay_s=0.15)
                    with mock.patch("si_analysis.fit_servo") as fit, redirect_stdout(io.StringIO()):
                        report = si.analyze(args)
                    fit.assert_not_called()
                    result = report["channels"]["right_arm"]
                    self.assertEqual(result["status"], "rejected_channel")
                    self.assertIn(f"{field} feedforward", result["reason"])
                    self.assertFalse(report["physical_kp_kd_inertia_friction_identified"])

    def test_sequential_joint_trials_get_individual_command_windows(self):
        events = regular_events(duration=24)
        for event in events:
            target = np.zeros(22)
            stamp = event["t_host_s"]
            for joint, start in ((5, 2.0), (9, 14.0)):
                relative = stamp - start
                if 0 <= relative <= 6:
                    target[joint] = 0.04 * np.sin(np.pi * relative / 6) * np.sin(2 * np.pi * 0.9 * relative)
            event["q_rad"] = target.tolist()
        fitted_inputs = []

        def fake_fit(t, u, q, max_delay_s):
            fitted_inputs.append((np.asarray(t), np.asarray(u)))
            split = int(0.7 * len(t))
            self.assertEqual(t[0], 0)
            self.assertLess(t[-1], 10, "a sequential joint must not fit another joint's idle section")
            self.assertGreater(np.ptp(u[:split]), 0.005)
            self.assertGreater(np.ptp(u[split:]), 0.005)
            self.assertTrue(np.all(np.abs(u[t < 0.8]) < 0.001), "keep neutral leading padding for command history")
            return dict(accepted=True, holdout_rmse_rad=0.0, prediction_rad=list(q), split_index=split)

        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run / "session.json").write_text(json.dumps({"source": "synthetic_window_test"}))
            (run / "events.jsonl").write_text("\n".join(json.dumps(e) for e in events))
            args = types.SimpleNamespace(run=str(run), out=None, channel=["right_hand"],
                                         joint=None, sample_hz=50, max_delay_s=0.15)
            with mock.patch("si_analysis.fit_servo", side_effect=fake_fit), redirect_stdout(io.StringIO()):
                report = si.analyze(args)
            self.assertEqual(len(fitted_inputs), 2)
            joints = report["channels"]["right_hand"]["joints"]
            for name, active_start in (("right_index_MCP_FE", 2.0), ("right_middle_MCP_FE", 14.0)):
                self.assertTrue(joints[name]["accepted"])
                window = joints[name]["analysis_window"]
                self.assertGreater(window["start_sample"], 0)
                self.assertLess(window["source_start_s"], active_start)
                self.assertGreater(window["source_end_s"], active_start + 6)
                self.assertEqual(window["requested_padding_s"], 1.0)


class PassiveDDSTests(unittest.TestCase):
    def fake_sdk(self, malformed=False, h2_messages=None):
        subscribers = []
        channel = types.ModuleType("unitree_sdk2py.core.channel")
        channel.ChannelFactoryInitialize = mock.Mock()
        channel.ChannelPublisher = mock.Mock(side_effect=AssertionError("publishers are forbidden"))

        class Subscriber:
            def __init__(self, topic, datatype):
                self.topic = topic
                self.closed = False
                subscribers.append(self)

            def Init(self, callback, depth):
                if h2_messages and self.topic in h2_messages:
                    callback(h2_messages[self.topic])
                    return
                kind = "state" if self.topic.endswith("/state") else "command"
                callback(hand_message(kind, [0.1] * (21 if malformed else 22)))

            def Close(self):
                self.closed = True

        channel.ChannelSubscriber = Subscriber
        idl = types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
        for name in ("HandCmd_", "HandState_", "LowCmd_", "LowState_"):
            setattr(idl, name, type(name, (), {}))
        names = ["unitree_sdk2py", "unitree_sdk2py.core", "unitree_sdk2py.idl",
                 "unitree_sdk2py.idl.unitree_hg", "unitree_sdk2py.idl.unitree_hg.msg"]
        modules = {name: types.ModuleType(name) for name in names}
        modules[channel.__name__] = channel
        modules[idl.__name__] = idl
        return modules, channel, subscribers

    def test_record_only_subscribes_and_closes_all_handles(self):
        modules, channel, subscribers = self.fake_sdk()
        clock = iter(np.arange(0, 10, 0.001))
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(sys.modules, modules), \
                mock.patch.object(si_dds.time, "monotonic", side_effect=lambda: next(clock)), redirect_stdout(io.StringIO()):
            dest = si_dds.record(Path(directory) / "record", "fake0", 0.004)
            status = json.loads((dest / "record_status.json").read_text())
            session = json.loads((dest / "session.json").read_text())
            self.assertTrue(status["valid_for_analysis"])
            self.assertFalse(status["robot_stopped_by_this_script"])
            self.assertEqual(session["publishers_created"], 0)
            self.assertEqual(len((dest / "events.jsonl").read_text().splitlines()), 4)
        channel.ChannelPublisher.assert_not_called()
        self.assertEqual(len(subscribers), 4)
        self.assertTrue(all(subscriber.closed for subscriber in subscribers))

    def test_callback_error_invalidates_log_and_never_publishes(self):
        modules, channel, subscribers = self.fake_sdk(malformed=True)
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(sys.modules, modules), redirect_stdout(io.StringIO()):
            dest = Path(directory) / "record"
            with self.assertRaises(RuntimeError):
                si_dds.record(dest, "fake0", 0.001)
            status = json.loads((dest / "record_status.json").read_text())
            self.assertFalse(status["valid_for_analysis"])
            self.assertTrue(status["errors"])
            self.assertEqual(status["stop_reason"], "recording_error")
            self.assertFalse(status["robot_stopped_by_this_script"])
        channel.ChannelPublisher.assert_not_called()
        self.assertTrue(all(subscriber.closed for subscriber in subscribers))

    def test_h2_idl_shaped_arrays_indices_and_auxiliary_are_serializable(self):
        # Match pinned HG IDL field shapes, including int8 temperature[2].
        # These are fake messages; this does not claim wire/firmware validation.
        state_motors = [types.SimpleNamespace(q=i * 0.01, dq=i * 0.001,
                          tau_est=i * 0.1, temperature=[30 + i, 31 + i],
                          motorstate=i, mode=1, vol=48.0) for i in range(35)]
        command_motors = [types.SimpleNamespace(q=i * 0.02, dq=0.03,
                            tau=0.02, kp=40.0, kd=1.0) for i in range(35)]
        command_motors[31].q = 1.0
        imu = types.SimpleNamespace(quaternion=[1.0, 0.0, 0.0, 0.0],
                                    gyroscope=[0.01, 0.02, 0.03],
                                    accelerometer=[0.0, 0.0, 9.81], rpy=[0.1, 0.2, 0.3])
        messages = {
            "rt/lowstate": types.SimpleNamespace(motor_state=state_motors, tick=123456, imu_state=imu),
            "rt/arm_sdk": types.SimpleNamespace(motor_cmd=command_motors),
        }
        modules, channel, subscribers = self.fake_sdk(h2_messages=messages)
        clock = iter(np.arange(0, 10, 0.001))
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(sys.modules, modules), \
                mock.patch.object(si_dds.time, "monotonic", side_effect=lambda: next(clock)), redirect_stdout(io.StringIO()):
            dest = si_dds.record(Path(directory) / "record", "fake0", 0.004, include_h2=True)
            events = [json.loads(line) for line in (dest / "events.jsonl").read_text().splitlines()]
            self.assertEqual(len(events), 9)
            event_map = {(e["channel"], e["kind"]): e for e in events}
            for side, start in (("left", 15), ("right", 22)):
                state = event_map[(f"{side}_arm", "state")]
                command = event_map[(f"{side}_arm", "command")]
                np.testing.assert_allclose(state["q_rad"], np.arange(start, start + 7) * 0.01)
                np.testing.assert_allclose(command["q_rad"], np.arange(start, start + 7) * 0.02)
                self.assertEqual(state["temperature"], [[30+i, 31+i] for i in range(start, start + 7)])
                self.assertEqual(state["device_timestamp"], 123456)
                self.assertEqual(command["arm_sdk_weight"], 1.0)
                self.assertEqual(command["dq"], [0.03] * 7)
                self.assertEqual(command["tau"], [0.02] * 7)
            base = event_map[("h2_base", "auxiliary")]
            np.testing.assert_allclose(base["waist_q_rad"], [0.12, 0.13, 0.14])
            self.assertEqual(base["imu_quaternion"], imu.quaternion)
            self.assertEqual(base["imu_accelerometer"], imu.accelerometer)
            self.assertEqual(base["imu_gyroscope"], imu.gyroscope)
            self.assertEqual(base["imu_rpy"], imu.rpy)
            json.dumps(events, allow_nan=False)
            self.assertTrue(json.loads((dest / "record_status.json").read_text())["valid_for_analysis"])
        channel.ChannelPublisher.assert_not_called()
        self.assertEqual(len(subscribers), 6)
        self.assertTrue(all(subscriber.closed for subscriber in subscribers))

    def test_keyboard_interrupt_stops_recorder_without_robot_command(self):
        modules, channel, subscribers = self.fake_sdk()

        class InterruptQueue(queue.Queue):
            def get(self, block=True, timeout=None):
                if timeout is not None:
                    raise KeyboardInterrupt
                return super().get(block=block, timeout=timeout)

        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(sys.modules, modules), \
                mock.patch.object(si_dds.queue, "Queue", InterruptQueue), redirect_stdout(io.StringIO()):
            dest = si_dds.record(Path(directory) / "record", "fake0", 10)
            status = json.loads((dest / "record_status.json").read_text())
            self.assertEqual(status["stop_reason"], "operator_stopped_recorder_robot_unaffected")
            self.assertFalse(status["robot_stopped_by_this_script"])
        channel.ChannelPublisher.assert_not_called()
        self.assertTrue(all(subscriber.closed for subscriber in subscribers))


if __name__ == "__main__":
    unittest.main()
