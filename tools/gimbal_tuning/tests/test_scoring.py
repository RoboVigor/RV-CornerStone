from dataclasses import replace
import unittest
from gimbal_tuning.protocol import Message as M, Telemetry, Status
from gimbal_tuning.scoring import Score, evaluate_trial, aggregate
from support import window


class ScoringTests(unittest.TestCase):
    def test_feasible_normalized_and_constraints_first(self):
        frames, expected, safety, perf = window()
        good = evaluate_trial(frames, expected, safety, perf)
        self.assertTrue(good.valid)
        self.assertTrue(good.feasible)
        self.assertGreater(good.utility, 0)
        self.assertIsNotNone(good.metrics["settling_ms"])
        frames2, expected, safety, perf = window([0] + [0.15] * 100)
        bad = evaluate_trial(frames2, expected, safety, perf)
        self.assertTrue(bad.valid)
        self.assertFalse(bad.feasible)
        self.assertLess(bad.utility, -1)
        result = aggregate([good, bad], repeats=2, scenario_count=1)
        self.assertFalse(result["feasible"])
        self.assertEqual(result["utility"], bad.utility)
        self.assertFalse(result["insufficient_repeats"])
        self.assertTrue(aggregate([good], 1, 1)["insufficient_repeats"])
        with self.assertRaises(ValueError):
            aggregate([good], 2, 1)

    def test_terminal_disabled_zero_is_required_only_at_verified_result_endpoint(self):
        frames, expected, safety, perf = window()
        score = evaluate_trial(frames, expected, safety, perf)
        self.assertTrue(score.valid, score.reasons)
        last = Telemetry.unpack(frames[100].payload)
        self.assertEqual(last.flags & 10, 8)
        for index, changes in ((20, dict(flags=8, output=0, command0=0, command1=0)),
                               (100, dict(flags=10)), (100, dict(flags=0)),
                               (100, dict(state=7))):
            altered = frames.copy()
            altered[index] = replace(altered[index], payload=replace(Telemetry.unpack(altered[index].payload), **changes).pack())
            with self.subTest(index=index, changes=changes):
                rejected = evaluate_trial(altered, expected, safety, perf)
                self.assertFalse(rejected.valid)
                self.assertIsNone(rejected.utility)

    def test_numerical_scoring_overflow_is_invalid_and_never_aggregated(self):
        frames, expected, safety, perf = window([0] + [0.15] * 100)
        for field in ("max_overshoot_rad", "final_error_rad"):
            pathological = dict(perf, **{field: 1e-320})
            score = evaluate_trial(frames, expected, safety, pathological)
            self.assertFalse(score.valid, score)
            self.assertIsNone(score.utility)
            self.assertIn("nonfinite", str(score.reasons))
            with self.assertRaises(ValueError):
                aggregate([score], 1, 1)
        for utility in (float("nan"), float("inf"), -float("inf")):
            with self.assertRaisesRegex(ValueError, "nonfinite"):
                Score(True, "COMPLETE", utility=utility)
            forged = Score(True, "COMPLETE", utility=1)
            forged.utility = utility
            with self.assertRaisesRegex(ValueError, "nonfinite"):
                aggregate([forged], 1, 1)
        with self.assertRaisesRegex(ValueError, "aggregate"):
            aggregate([Score(True, "COMPLETE", utility=-1.7e308), Score(True, "COMPLETE", utility=1.7e308)], 2, 1)

    def test_output_variation_preserves_activity_for_opposite_esc_directions(self):
        frames, expected, safety, perf = window()
        same = evaluate_trial(frames, expected, safety, perf)
        opposite_frames = []
        for frame in frames:
            if frame.type == M.TELEMETRY:
                row = Telemetry.unpack(frame.payload)
                frame = replace(frame, payload=replace(row, command1=-row.command1).pack())
            opposite_frames.append(frame)
        opposite = evaluate_trial(opposite_frames, expected, safety, perf)
        self.assertTrue(same.valid, same.reasons)
        self.assertTrue(opposite.valid, opposite.reasons)
        self.assertEqual(same.metrics["output_variation"], 50)
        self.assertEqual(opposite.metrics["output_variation"], 50)
        self.assertEqual(same.metrics["normalized_cost"], opposite.metrics["normalized_cost"])
        self.assertEqual(same.utility, opposite.utility)

    def test_settling_requires_continuous_final_hold(self):
        values = [0] * 30 + [0.1] * 60 + [0.13] + [0.1] * 10
        score = evaluate_trial(*window(values))
        self.assertTrue(score.valid)
        self.assertFalse(score.feasible)
        self.assertIsNone(score.metrics["settling_ms"])

    def test_tail_oscillation_constraint_even_inside_error_band(self):
        values = [0.1] * 70 + [0.092 if i % 2 else 0.108 for i in range(31)]
        frames, expected, safety, perf = window(values)
        perf["sustained_oscillation_limit_rad"] = 0.01
        score = evaluate_trial(frames, expected, safety, perf)
        self.assertTrue(score.valid)
        self.assertFalse(score.feasible)
        self.assertIn("tail_oscillation", score.reasons)

    def test_speed_uses_speed_units_and_final_ramped_reference(self):
        frames, expected, safety, perf = window([0.1] * 101, mode=1)
        perf.update(final_error_rad=1e-5, final_error_rad_s=0.02)
        score = evaluate_trial(frames, expected, safety, perf)
        self.assertTrue(score.valid)
        self.assertTrue(score.feasible)
        self.assertEqual(score.metrics["unit"], "rad/s")
        self.assertGreaterEqual(score.metrics["settling_ms"], 200)
        self.assertEqual(score.metrics["reference_ready_ms"], 200)
        frames, expected, safety, perf = window([0.095] * 101, mode=1)
        perf.update(final_error_rad=1e-5, final_error_rad_s=0.02)
        self.assertTrue(evaluate_trial(frames, expected, safety, perf).feasible)

    def test_reject_missing_reordered_duplicate_nonfinite_and_identity(self):
        mutations = [
            lambda f: f.pop(0), lambda f: f.pop(20), lambda f: f.insert(20, f[20]),
            lambda f: f.__setitem__(slice(20, 22), [f[21], f[20]]),
            lambda f: f.__setitem__(20, replace(f[20], boot_id=10)),
            lambda f: f.__setitem__(20, replace(f[20], session=4)),
            lambda f: f.__setitem__(20, replace(f[20], payload=replace(Telemetry.unpack(f[20].payload), pos0=float("nan")).pack())),
            lambda f: f.__setitem__(20, replace(f[20], payload=replace(Telemetry.unpack(f[20].payload), trial_id=2).pack())),
            lambda f: f.__setitem__(20, replace(f[20], payload=replace(Telemetry.unpack(f[20].payload), active_version=3).pack())),
            lambda f: f.__setitem__(20, replace(f[20], payload=replace(Telemetry.unpack(f[20].payload), applied_seq=2).pack())),
        ]
        for mutate in mutations:
            frames, expected, safety, perf = window()
            perf["max_missing_fraction"] = 0
            mutate(frames)
            with self.subTest(mutation=mutate):
                score = evaluate_trial(frames, expected, safety, perf)
                self.assertFalse(score.valid)
                self.assertIsNone(score.utility)

    def test_gap_drop_counter_partial_and_zero_confirmations(self):
        for kind in ("gap", "counter", "partial", "zero", "status", "late_fault", "zero_command"):
            frames, expected, safety, perf = window()
            if kind == "gap":
                del frames[20:25]
            elif kind == "counter":
                frames[100] = replace(frames[100], payload=replace(Telemetry.unpack(frames[100].payload), tx_dropped=100).pack())
            elif kind == "partial":
                expected["trial"]["duration_ms"] = 2000
            elif kind == "zero":
                frames.pop(-2)
            elif kind == "status":
                frames.pop(-1)
            elif kind == "late_fault":
                frames.append(replace(frames[-1], payload=replace(Status.unpack(frames[-1].payload), fault=14).pack()))
            else:
                frames[100] = replace(frames[100], payload=replace(Telemetry.unpack(frames[100].payload), command0=1).pack())
            with self.subTest(kind=kind):
                score = evaluate_trial(frames, expected, safety, perf)
                self.assertFalse(score.valid)
                self.assertIsNone(score.utility)

    def test_wrap_is_valid_but_reset_is_not(self):
        score = evaluate_trial(*window(wrap=True))
        self.assertTrue(score.valid, score.reasons)
        frames, expected, safety, perf = window()
        frames[50] = replace(frames[50], payload=replace(Telemetry.unpack(frames[50].payload), sample_seq=1).pack())
        self.assertFalse(evaluate_trial(frames, expected, safety, perf).valid)


if __name__ == "__main__":
    unittest.main()
