"""Exercise task CLI handoff without ROS, hardware, or real permit artifacts."""
from contextlib import ExitStack
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, mock_open, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import task_flight_runtime as entry


class ResidentTaskEntryTests(unittest.TestCase):
    def run_entry(self, live=False, resident=True, binding_failure=False):
        events = []
        permit = SimpleNamespace(session_id='fixture-only',
            consume=lambda path: events.append('consume'))
        binding = Mock()
        def check():
            events.append('check')
            if binding_failure:
                raise ValueError('resident stale')
        binding.check.side_effect = check
        runner = Mock(side_effect=lambda *a, **kw: events.append('run') or 0)
        fake_runtime = SimpleNamespace(
            confirm=lambda session: events.append('confirm'), run=runner)
        argv = ['task-entry', '--release', '/fixture/release', '--params', '/fixture/params']
        if resident:
            argv += ['--resident-status', '/fixture/status']
        if live:
            argv += ['--authorize-real-flight']
        opened = mock_open()
        with ExitStack() as stack:
            stack.enter_context(patch.object(sys, 'argv', argv))
            stack.enter_context(patch.object(entry, 'validate', return_value=(permit, {})))
            stack.enter_context(patch('resident_status_binding.ResidentStatusBinding', return_value=binding))
            stack.enter_context(patch.dict(sys.modules, {'live_flight_runtime': fake_runtime}))
            stack.enter_context(patch('builtins.open', opened))
            stack.enter_context(patch.object(entry.fcntl, 'flock'))
            stack.enter_context(patch('sys.stdout', new_callable=io.StringIO))
            stack.enter_context(patch('sys.stderr', new_callable=io.StringIO))
            if binding_failure:
                with self.assertRaises(SystemExit):
                    entry.main()
            else:
                self.assertEqual(entry.main(), 0)
        return events, runner, binding, opened

    def test_default_checks_only(self):
        events, runner, _, opened = self.run_entry()
        self.assertEqual(events, [])
        runner.assert_not_called()
        opened.assert_not_called()

    def test_bound_entry_preserves_confirmation_and_one_use_handoff(self):
        events, runner, binding, opened = self.run_entry(live=True)
        self.assertEqual(events, ['confirm', 'check', 'consume', 'run'])
        opened.assert_called_once_with('/tmp/robocup_task_control.lock', 'a')
        self.assertIs(runner.call_args.kwargs['resident_binding'], binding)
        self.assertTrue(runner.call_args.kwargs['task'])
        binding.stop.assert_not_called()
        binding.close.assert_not_called()

    def test_stale_binding_cannot_consume_or_start(self):
        events, runner, _, _ = self.run_entry(live=True, binding_failure=True)
        self.assertEqual(events, ['confirm', 'check'])
        runner.assert_not_called()

    def test_legacy_entry_retains_ev_owner_lock(self):
        events, runner, _, opened = self.run_entry(live=True, resident=False)
        self.assertEqual(events, ['confirm', 'consume', 'run'])
        opened.assert_called_once_with('/tmp/robocup_flight_runtime_shadow.lock', 'a')
        self.assertIsNone(runner.call_args.kwargs['resident_binding'])


if __name__ == '__main__':
    unittest.main()
