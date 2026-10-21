"""Numerical and artifact checks for the passive hover diagnostic, without ROS."""
import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zipfile


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'hover_drift_check', ROOT / 'scripts/analysis/hover_drift_check.py')
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


def pose_row(receipt, xyz=(0., 0., 0.), yaw_deg=0., stamp=None, frame='map'):
    angle = math.radians(yaw_deg)
    stamp = int((100. + receipt) * 1e9) if stamp is None else stamp
    return dict(
        received_monotonic_s=float(receipt),
        received_ros_ns=int((100. + receipt) * 1e9) + 20_000_000,
        stamp_ns=stamp, frame=frame, position=list(xyz),
        quaternion=[0., 0., math.sin(angle / 2), math.cos(angle / 2)],
        yaw_rad=angle)


class PoseMeasurements(unittest.TestCase):
    def assertVectorAlmostEqual(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b)

    def test_endpoint_averages_keep_transient_excursion_visible(self):
        rows = [pose_row(t, (1., 2., 3.)) for t in (0, 1, 4)]
        rows += [pose_row(6, (2., 2., 3.))]
        rows += [pose_row(t, (1.3, 2.4, 3.)) for t in (10, 11, 14)]
        out = m.summarize_pose(rows, 0., 14.)
        self.assertEqual(out['valid_pose_count'], 7)
        self.assertVectorAlmostEqual(out['initial_mean_xyz_m'], [1., 2., 3.])
        self.assertVectorAlmostEqual(out['final_mean_xyz_m'], [1.3, 2.4, 3.])
        self.assertVectorAlmostEqual(
            out['mean_window_displacement_xyz_m'], [.3, .4, 0.])
        self.assertAlmostEqual(out['mean_window_displacement_norm_m'], .5)
        self.assertAlmostEqual(out['max_displacement_from_initial_mean_m'], 1.)
        self.assertVectorAlmostEqual(out['xyz_range_m'], [1., .4, 0.])

    def test_yaw_crossing_pi_is_a_small_change(self):
        rows = [pose_row(t, yaw_deg=179.) for t in (0, 1, 4)]
        rows += [pose_row(6, yaw_deg=180.)]
        rows += [pose_row(t, yaw_deg=-179.) for t in (10, 11, 14)]
        out = m.summarize_pose(rows, 0., 14.)
        self.assertAlmostEqual(out['yaw_drift_deg'], 2.)
        self.assertAlmostEqual(out['yaw_range_deg'], 2.)
        self.assertAlmostEqual(out['mean_window_displacement_norm_m'], 0.)

    def test_invalid_poses_are_counted_without_contaminating_displacement(self):
        rows = [pose_row(0), pose_row(1), pose_row(2), pose_row(10)]
        rows[1]['position'][0] = float('nan')
        rows[2]['quaternion'] = [0., 0., 0., 0.]
        out = m.summarize_pose(rows, 0., 10.)
        self.assertEqual(out['count'], 4)
        self.assertEqual(out['valid_pose_count'], 2)
        self.assertEqual(out['invalid_pose_count'], 2)
        self.assertAlmostEqual(out['mean_window_displacement_norm_m'], 0.)

    def test_missing_pose_does_not_report_zero_drift(self):
        out = m.summarize_pose([], 0., 20.)
        self.assertEqual(out['count'], 0)
        self.assertNotIn('mean_window_displacement_norm_m', out)

    def test_changed_reference_or_child_frame_cannot_measure_displacement(self):
        for field in ('frame', 'child_frame'):
            with self.subTest(field=field):
                rows = [pose_row(0), pose_row(14, (10., 0., 0.))]
                rows[0][field] = 'before'
                rows[1][field] = 'after'
                out = m.summarize_pose(rows, 0., 14.)
                self.assertEqual(out['valid_pose_count'], 2)
                self.assertFalse(out['coordinate_frame_consistent'])
                self.assertNotIn('mean_window_displacement_norm_m', out)

    def test_quaternion_validation_does_not_accept_fabricated_yaw(self):
        self.assertAlmostEqual(m.quaternion_yaw([0., 0., 0., 1.]), 0.)
        for q in ([0., 0., 0., 0.], [0., 0., 0., 2.],
                  [float('nan'), 0., 0., 1.], [0., float('inf'), 0., 1.]):
            with self.subTest(q=q), self.assertRaises(ValueError):
                m.quaternion_yaw(q)


class StreamMeasurements(unittest.TestCase):
    def test_imu_population_spread_excludes_invalid_vectors(self):
        rows = [pose_row(t) for t in (0, 1, 2)]
        for row, value in zip(rows, (0., 2., float('nan'))):
            row['linear_acceleration'] = [value, 0., -9.81]
            row['angular_velocity'] = [0., 0., value]
        out = m.summarize_imu(rows, 0., 2.)
        for field, axis in (('linear_acceleration', 0), ('angular_velocity', 2)):
            self.assertEqual(out[field]['valid_count'], 2)
            self.assertEqual(out[field]['invalid_count'], 1)
            self.assertAlmostEqual(out[field]['mean_xyz'][axis], 1.)
            self.assertAlmostEqual(out[field]['std_xyz'][axis], 1.)
        self.assertAlmostEqual(out['linear_acceleration']['std_xyz'][2], 0.)

    def test_stationary_pose_with_advancing_stamp_is_not_frozen(self):
        rows = [pose_row(t) for t in range(11)]
        out = m.summarize_stream(rows, 0., 10.)
        self.assertEqual(out['invalid_stamp_count'], 0)
        self.assertEqual(out['nonincreasing_stamp_count'], 0)
        self.assertAlmostEqual(out['receipt_hz'], 1.)
        self.assertAlmostEqual(out['max_receive_gap_s'], 1.)
        self.assertAlmostEqual(out['end_receive_age_s'], 0.)

    def test_fresh_receipts_do_not_hide_a_frozen_source_stamp(self):
        rows = [pose_row(t, stamp=100_000_000_000) for t in range(11)]
        out = m.summarize_stream(rows, 0., 10.)
        self.assertEqual(out['nonincreasing_stamp_count'], 10)
        self.assertAlmostEqual(out['receipt_hz'], 1.)
        self.assertAlmostEqual(out['header_age_s']['max'], 10.02)

    def test_disconnection_at_end_remains_visible(self):
        rows = [pose_row(t) for t in (0, 1, 2)]
        out = m.summarize_stream(rows, 0., 20.)
        self.assertAlmostEqual(out['end_receive_age_s'], 18.)
        self.assertEqual(out['count'], 3)

    def test_reordered_stamps_and_frame_changes_are_retained(self):
        rows = [pose_row(0, stamp=101_000_000_000),
                pose_row(1, stamp=100_000_000_000, frame='odom'),
                pose_row(2, stamp=0)]
        out = m.summarize_stream(rows, 0., 2.)
        self.assertEqual(out['nonincreasing_stamp_count'], 2)
        self.assertEqual(out['invalid_stamp_count'], 1)
        self.assertEqual(sorted(out['frames']), ['map', 'odom'])


class GroundTelemetryGuards(unittest.TestCase):
    def observer(self):
        collector = m.Collector.__new__(m.Collector)
        collector.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
        collector.ON_GROUND = 1
        collector.IN_AIR = {2, 3, 4}
        collector.latest = {
            m.STATE: dict(received_monotonic_s=42., stamp_ns=100_000_000_000,
                          connected=True, armed=False),
            m.EXTENDED: dict(received_monotonic_s=42., stamp_ns=100_000_000_000,
                             landed_state=1),
        }
        return collector

    def test_ground_state_requires_both_fresh_receipts_and_fresh_headers(self):
        collector = self.observer()
        with patch.object(m.time, 'monotonic', return_value=42.):
            self.assertTrue(collector.ground_ready())
            for topic, field, value in (
                (m.STATE, 'connected', False),
                (m.STATE, 'armed', True),
                (m.EXTENDED, 'landed_state', 2),
                (m.STATE, 'received_monotonic_s', 38.),
                (m.EXTENDED, 'received_monotonic_s', 38.),
                (m.STATE, 'stamp_ns', 96_000_000_000),
                (m.EXTENDED, 'stamp_ns', 96_000_000_000),
                (m.STATE, 'stamp_ns', 101_000_000_000),
                (m.EXTENDED, 'stamp_ns', 0),
            ):
                with self.subTest(topic=topic, field=field, value=value):
                    collector = self.observer()
                    collector.latest[topic][field] = value
                    self.assertFalse(collector.ground_ready())
            for topic in (m.STATE, m.EXTENDED):
                collector = self.observer()
                del collector.latest[topic]
                self.assertFalse(collector.ground_ready())

    def test_transient_armed_or_airborne_report_stays_latched(self):
        for topic, field, bad, good in (
            (m.STATE, 'armed', True, False),
            (m.EXTENDED, 'landed_state', 2, 1),
        ):
            with self.subTest(topic=topic):
                collector = self.observer()
                collector.phase = 'capture'
                collector.rows = {t: [] for t in m.TOPICS}
                collector.output = io.StringIO()
                collector.ground_fault = None
                msg = SimpleNamespace(
                    header=SimpleNamespace(frame_id='', stamp=SimpleNamespace(
                        sec=100, nanosec=0)),
                    armed=False, connected=True, mode='POSCTL', landed_state=1)
                receive = collector.callback(topic)
                with patch.object(m.time, 'monotonic', return_value=42.):
                    setattr(msg, field, bad)
                    receive(msg)
                    self.assertTrue(collector.ground_fault)
                    setattr(msg, field, good)
                    receive(msg)
                    self.assertTrue(collector.ground_fault)
                self.assertEqual(len(collector.rows[topic]), 2)


class SavedEvidence(unittest.TestCase):
    def save(self, folder, result, rows, **extra):
        report = dict(result=result, data_issues=[], **extra)
        m.save_artifacts(folder, report, rows, 0., 20.)
        return json.loads((folder / 'report.json').read_text())

    def test_partial_error_preserves_readings_missing_streams_and_zip(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / 'telemetry.jsonl').write_text('{"partial": true}\n')
            out = self.save(folder, 'ERROR', {m.RAW: [pose_row(1)]},
                            error='FC telemetry lost',
                            parameter_readbacks={'px4': {'status': 'ERROR',
                                                         'error': 'service timeout'}},
                            diagnostic_value=float('nan'))
            self.assertEqual(out['result'], 'ERROR')
            self.assertEqual(out['error'], 'FC telemetry lost')
            self.assertEqual(out['streams'][m.RAW]['count'], 1)
            self.assertEqual(out['streams'][m.POSES[-1]]['count'], 0)
            self.assertIn(m.POSES[-1], out['missing_optional_topics'])
            self.assertIsNone(out['diagnostic_value'])
            self.assertIn('FC telemetry lost', (folder / 'summary.txt').read_text())
            with zipfile.ZipFile(folder / 'diagnostic.zip') as archive:
                self.assertTrue({'report.json', 'summary.txt', 'telemetry.jsonl'}
                                <= set(archive.namelist()))
                saved = json.loads(archive.read('report.json'))
                self.assertEqual(saved['parameter_readbacks']['px4']['error'],
                                 'service timeout')
                self.assertEqual(saved['result'], 'ERROR')

    def test_frozen_source_and_changed_frame_preclude_complete_capture(self):
        for rows in (
            [pose_row(t, stamp=100_000_000_000) for t in range(21)],
            [pose_row(t, frame='map' if t < 10 else 'odom') for t in range(21)],
        ):
            with self.subTest(rows=rows), tempfile.TemporaryDirectory() as directory:
                out = self.save(Path(directory), 'CAPTURE_COMPLETE', {m.RAW: rows})
                self.assertEqual(out['result'], 'INCOMPLETE_DATA')
                self.assertTrue(out['data_issues'])

    def test_required_stream_lost_near_start_is_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            out = self.save(Path(directory), 'CAPTURE_COMPLETE',
                            {m.RAW: [pose_row(t) for t in (0, 1, 2)]})
            self.assertEqual(out['result'], 'INCOMPLETE_DATA')
            self.assertTrue(out['data_issues'])

    def test_missing_optional_streams_remain_explicit_without_fabricated_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            out = self.save(Path(directory), 'CAPTURE_COMPLETE',
                            {m.RAW: [pose_row(t) for t in range(21)]})
            self.assertEqual(out['result'], 'CAPTURE_COMPLETE')
            self.assertIn(m.CLOUD, out['missing_optional_topics'])
            self.assertNotIn('mean_window_displacement_norm_m',
                             out['streams'][m.POSES[-1]])


if __name__ == '__main__':
    unittest.main()
