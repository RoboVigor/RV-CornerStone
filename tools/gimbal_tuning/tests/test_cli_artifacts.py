from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from gimbal_tuning.cli import main, preflight_output_path


class ArtifactOutputTests(unittest.TestCase):
    def invoke(self, args):
        output, error = io.StringIO(), io.StringIO()
        with redirect_stdout(output), redirect_stderr(error):
            code = main(args)
        return code, output.getvalue(), error.getvalue()

    def test_optimize_rejects_inside_output_before_config_manager_or_port(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "new-run"
            for destination in (run, run / "best.json", run / "nested" / ".." / "best.json"):
                with self.subTest(destination=destination), \
                     patch("gimbal_tuning.cli.Configuration.load") as load, \
                     patch("gimbal_tuning.cli.ExperimentManager") as manager, \
                     patch("gimbal_tuning.cli.SerialTransport") as serial:
                    code, _, error = self.invoke([
                        "optimize", "--config", str(root / "not-loaded.json"),
                        "--run-dir", str(run), "--port", "MOCK", "--authorize-motion",
                        "--export-best", str(destination),
                    ])
                    self.assertEqual(code, 2)
                    self.assertIn("outside immutable evidence directory", error)
                    load.assert_not_called()
                    manager.assert_not_called()
                    serial.assert_not_called()
                    self.assertFalse(run.exists())

    def test_analysis_outputs_cannot_mutate_either_input_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            legacy, conditional = root / "legacy", root / "conditional"
            for run in (legacy, conditional):
                run.mkdir()
                (run / "manifest.json").write_bytes(b"original manifest\n")
                (run / "raw.jsonl").write_bytes(b"original evidence\n")
            before = {p: p.read_bytes() for run in (legacy, conditional) for p in run.iterdir()}
            cases = [(["analyze", str(legacy)], legacy),
                     (["replay", str(legacy), "--propose"], legacy),
                     (["compare", str(legacy), str(conditional)], legacy),
                     (["compare", str(legacy), str(conditional)], conditional)]
            for args, target in cases:
                for destination in (target / "derived.json", target / "manifest.json"):
                    with self.subTest(args=args, destination=destination), \
                         patch("gimbal_tuning.cli.replay") as replay, \
                         patch("gimbal_tuning.cli.compare_baselines") as compare:
                        code, _, error = self.invoke(args + ["--output", str(destination)])
                        self.assertEqual(code, 2)
                        self.assertIn("outside immutable evidence directory", error)
                        replay.assert_not_called()
                        compare.assert_not_called()
            after = {p: p.read_bytes() for run in (legacy, conditional) for p in run.iterdir()}
            self.assertEqual(before, after)

    def test_preflight_resolves_symlinked_roots_parents_and_existing_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "evidence"
            run.mkdir()
            original = run / "manifest.json"
            original.write_bytes(b"unchanged\n")
            alias, file_alias, future_alias = root / "alias", root / "file-alias", root / "future-alias"
            future = root / "future-run"
            try:
                alias.symlink_to(run, target_is_directory=True)
                file_alias.symlink_to(original)
                future_alias.symlink_to(future, target_is_directory=True)
            except OSError as exc:
                self.skipTest(f"symlinks unavailable: {exc}")
            for destination, evidence in ((alias / "new.json", run), (file_alias, run),
                                          (run / "new.json", alias), (future_alias / "best.json", future)):
                with self.subTest(destination=destination), self.assertRaisesRegex(ValueError, "immutable evidence"):
                    preflight_output_path(destination, evidence)
            self.assertEqual(original.read_bytes(), b"unchanged\n")
            self.assertFalse(future.exists())
            with patch("gimbal_tuning.cli.ExperimentManager") as manager:
                code, _, error = self.invoke(["optimize", "--simulate", "--config", "not-read.json",
                                              "--run-dir", str(future), "--export-best", str(future_alias / "best.json")])
            self.assertEqual(code, 2)
            self.assertIn("immutable evidence", error)
            manager.assert_not_called()

    def test_outside_output_and_stdout_remain_supported(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "evidence"
            destination = run / ".." / "derived.json"
            self.assertEqual(preflight_output_path(destination, run), (root / "derived.json").resolve())
            self.assertIsNone(preflight_output_path(None, run))
            with patch("gimbal_tuning.cli.replay", return_value={"offline": True}) as replay:
                code, _, error = self.invoke(["analyze", str(run), "--output", str(destination)])
            self.assertEqual(code, 0, error)
            replay.assert_called_once()
            self.assertEqual(json.loads((root / "derived.json").read_text()), {"offline": True})
            self.assertFalse(run.exists())
            with patch("gimbal_tuning.cli.replay", return_value={"offline": True}):
                code, output, error = self.invoke(["replay", str(run)])
            self.assertEqual(code, 0, error)
            self.assertEqual(json.loads(output), {"offline": True})


if __name__ == "__main__":
    unittest.main()
