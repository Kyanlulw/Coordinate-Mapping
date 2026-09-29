"""Analytic geometry and synthetic multi-frame regression tests."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

import camera_pixel_to_world as pixel
import lidar_to_geographic as geo
from merge_point_clouds import merge_frames, overlap_statistics, voxel_indices


class PixelTests(unittest.TestCase):
    def setUp(self):
        # fx=100, fy=200, cx=20, cy=30, camera center x=0.5, z=-0.1.
        self.projection = np.array([[100., 0, 20, -48], [0, 200, 30, 3], [0, 0, 1, .1]])

    def test_baseline_and_optical_depth(self):
        # Relative camera point [1,2,10] corresponds to LiDAR [1.5,2,9.9].
        xyz = pixel.backproject_pixels([[30, 70]], [10], self.projection)
        np.testing.assert_allclose(xyz, [[1.5, 2, 9.9]], rtol=0, atol=1e-12)

    def test_pixel_to_world_with_pose_and_lever_arm(self):
        pose = np.array([0, 9, 100, 0, 0, np.pi / 2])
        calibration = np.eye(4)
        calibration[:3, 3] = [.5, 0, -.1]
        _, world, coordinates, meta = pixel.convert_pixels(
            [[30, 70]], [10], self.projection, pose, calibration, geo.make_world_reference(pose))
        # IMU point [1,2,10], yaw +90 degrees -> ENU [-2,1,10].
        np.testing.assert_allclose(world, [[-2, 1, 10]], atol=2e-9, rtol=0)
        self.assertEqual(meta['utm_epsg'], 32632)
        self.assertTrue(np.isfinite(coordinates['ecef_m']).all())
        self.assertLess(meta['pixel_reprojection_max_error_px'], 1e-10)

    def test_nonpositive_missing_nonfinite_depth(self):
        for depth in ([0], [-1], [np.nan], [np.inf], [], [1, 2]):
            with self.subTest(depth=depth), self.assertRaises(ValueError):
                pixel.backproject_pixels([[10, 20]], depth, self.projection)
        with self.assertRaises(ValueError):
            pixel.backproject_pixels([[np.nan, 20]], [10], self.projection)

    def test_z_buffer_and_missing_depth(self):
        projection = np.column_stack((np.eye(3), np.zeros(3)))
        scan = np.array([[2, 2, 2, 1], [1, 1, 1, 1], [0, 0, -1, 1], [20, 0, 1, 1]])
        uv, depth, indices = pixel.visible_lidar(scan, projection, (10, 10))
        np.testing.assert_array_equal(indices, [1])
        np.testing.assert_allclose(depth, [1])
        _, selected, _, distances = pixel.lookup_lidar_depth([[1, 1]], uv, depth, indices, 0)
        self.assertEqual(selected[0], 1)
        self.assertEqual(distances[0], 0)
        with self.assertRaises(ValueError):
            pixel.lookup_lidar_depth([[5, 5]], uv, depth, indices, 2)
        with self.assertRaises(ValueError):
            pixel.lookup_lidar_depth([[1, 1]], uv, depth, indices, float('nan'))


class GeographicOutputTests(unittest.TestCase):
    def test_ecef_analytic_reference(self):
        coordinates, meta = geo.georeference_points(np.zeros((1, 3)), [0, 0, 10, 0, 0, 0], np.eye(4))
        np.testing.assert_allclose(coordinates['ecef_m'], [[6378147, 0, 0]], atol=1e-9, rtol=0)
        errors = geo.numerical_checks(np.zeros((1, 3)), coordinates, meta)
        for key in ('ecef_to_lidar_roundtrip_3d', 'utm_height_to_ecef_roundtrip_3d', 'enu_vs_proj_topocentric_3d'):
            self.assertLess(errors[key]['max_m'], 1e-7)

    def test_pcd_precision(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'test.pcd'
            xyz = np.array([[4150000.123456789, 600000.987654321, 4800000.123456789]])
            geo.write_xyz_pcd(path, xyz, [.4], 'EPSG:4978')
            text = path.read_text()
            self.assertIn('SIZE 8 8 8 4', text)
            recovered = np.loadtxt(text.split('DATA ascii\n')[1].splitlines(), ndmin=2)
            np.testing.assert_array_equal(recovered[:, :3], xyz)


class MultiFrameTests(unittest.TestCase):
    def make_sequence(self, folder):
        root = Path(folder)
        sequence = root / 'test_sync'
        (sequence / 'velodyne_points/data').mkdir(parents=True)
        (sequence / 'oxts/data').mkdir(parents=True)
        (root / 'calib_imu_to_velo.txt').write_text('R: 1 0 0 0 1 0 0 0 1\nT: 0 0 0\n')
        # Same static points observed after IMU ascends 2m and turns +90 degrees.
        scans = [[[10, 2, 5, .2], [8, -3, 4, .5]], [[2, -10, 3, .2], [-3, -8, 2, .5]]]
        for frame, scan in enumerate(scans):
            pose = np.zeros(30)
            pose[:6] = [0, 9, 100 + 2 * frame, 0, 0, frame * np.pi / 2]
            np.savetxt(sequence / f'oxts/data/{frame:010d}.txt', pose[None])
            np.asarray(scan, dtype='<f4').tofile(sequence / f'velodyne_points/data/{frame:010d}.bin')
        return sequence

    def test_static_scene_two_poses_and_provenance(self):
        with tempfile.TemporaryDirectory() as folder:
            sequence = self.make_sequence(folder)
            arrays, meta = merge_frames(sequence, [0, 1], sequence.parent, voxel_size=0)
            expected = [[10, 2, 5], [8, -3, 4], [10, 2, 5], [8, -3, 4]]
            np.testing.assert_allclose(arrays['world_xyz'], expected, atol=2e-9, rtol=0)
            np.testing.assert_array_equal(arrays['frame_id'], [0, 0, 1, 1])
            np.testing.assert_array_equal(arrays['point_index'], [0, 1, 0, 1])
            self.assertEqual(meta['input_point_count'], 4)
            self.assertEqual(meta['utm_epsg'], 32632)
            self.assertLess(meta['neighbor_frame_distances'][0]['p95_m'], 2e-9)
            self.assertLess(meta['world_ecef_roundtrip_3d']['max_m'], 2e-9)

    def test_export_cli_ecef_columns_and_extension(self):
        with tempfile.TemporaryDirectory() as folder:
            sequence = self.make_sequence(folder)
            output = Path(folder) / 'scan.csv'
            subprocess.run([sys.executable, str(geo.ROOT / 'lidar_to_geographic.py'),
                            '--sequence', str(sequence), '--frame', '0', '--output', str(output)],
                           check=True, capture_output=True, text=True)
            table = np.genfromtxt(output, delimiter=',', names=True)
            self.assertIn('ecef_x_m', table.dtype.names)
            self.assertTrue((Path(folder) / 'scan_ecef.pcd').is_file())
            self.assertTrue((Path(folder) / 'scan_utm.pcd').is_file())
            self.assertFalse((Path(folder) / 'scan_utm.ply').exists())
            metadata = json.loads(output.with_suffix('.json').read_text())
            self.assertEqual(metadata['point_count'], 2)
            self.assertIn('numerical_checks', metadata)

    def test_voxel_provenance_and_negative_coordinates(self):
        xyz = np.array([[.01, 0, 0], [.09, 0, 0], [-.01, 0, 0], [-.09, 0, 0]])
        np.testing.assert_array_equal(voxel_indices(xyz, .1), [0, 2])
        np.testing.assert_array_equal(voxel_indices(xyz, 0), [0, 1, 2, 3])
        for size in (-1, np.nan, np.inf):
            with self.assertRaises(ValueError):
                voxel_indices(xyz, size)

    def test_nonoverlap_is_not_hidden(self):
        stats = overlap_statistics(np.zeros((2, 3)), np.full((2, 3), 100))
        self.assertEqual(stats['fraction_within_threshold'], 0)
        self.assertIsNone(stats['within_threshold_rmse_m'])
        self.assertGreater(stats['all_samples_rmse_m'], 100)

    def test_reject_duplicate_or_missing_frames(self):
        with tempfile.TemporaryDirectory() as folder:
            sequence = self.make_sequence(folder)
            for frames in ([0], [0, 0], [1, 0], [-1, 0]):
                with self.assertRaises(ValueError):
                    merge_frames(sequence, frames, sequence.parent)
            with self.assertRaises(OSError):
                merge_frames(sequence, [0, 2], sequence.parent)


if __name__ == '__main__':
    unittest.main()
