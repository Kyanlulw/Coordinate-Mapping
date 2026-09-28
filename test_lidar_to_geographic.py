"""Coordinate convention and independent PROJ checks. Run with unittest."""

import unittest

import numpy as np
from pyproj import Transformer

import lidar_to_geographic as geo


class GeographicTransformTests(unittest.TestCase):
    def setUp(self):
        self.pose = np.array([49.03082736607, 8.3397870121153, 114.57347869873,
                              0.033758, -0.000435, -0.9423586732051])

    def test_kitti_angle_directions(self):
        # Positive yaw turns east to north; pitch lowers the nose; roll raises the left side.
        np.testing.assert_allclose(geo.imu_to_enu_rotation(0, 0, np.pi / 2) @ [1, 0, 0],
                                   [0, 1, 0], atol=1e-15)
        np.testing.assert_allclose(geo.imu_to_enu_rotation(0, np.pi / 2, 0) @ [1, 0, 0],
                                   [0, 0, -1], atol=1e-15)
        np.testing.assert_allclose(geo.imu_to_enu_rotation(np.pi / 2, 0, 0) @ [0, 1, 0],
                                   [0, 0, 1], atol=1e-15)

    def test_utm_reference_and_hemisphere(self):
        # On a UTM zone's central meridian at the equator, E=500000 and N=0.
        pose = np.array([0, 9, 0, 0, 0, 0])
        points, meta = geo.georeference_points(np.zeros((1, 3)), pose, np.eye(4))
        self.assertEqual(meta["utm_epsg"], 32632)
        np.testing.assert_allclose(points["utm_easting_m"], [500000], atol=1e-7)
        np.testing.assert_allclose(points["utm_northing_m"], [0], atol=1e-7)
        self.assertEqual(geo.automatic_utm_epsg(-33, 18), 32734)
        self.assertEqual(geo.automatic_utm_epsg(60, 4), 32632)  # Norway
        self.assertEqual(geo.automatic_utm_epsg(75, 10), 32633)  # Svalbard

    def test_imu_origin_and_calibration_direction(self):
        transform = np.eye(4)
        transform[:3, :3] = geo.imu_to_enu_rotation(0.02, -0.03, 0.1)
        transform[:3, 3] = [-0.8, 0.3, -0.8]
        # The IMU origin has coordinates equal to calibration translation in LiDAR.
        points, _ = geo.georeference_points(transform[None, :3, 3], self.pose, transform)
        np.testing.assert_allclose(points["enu_m"], [[0, 0, 0]], atol=1e-14)
        np.testing.assert_allclose(points["latitude_deg"], [self.pose[0]], atol=1e-10, rtol=0)
        np.testing.assert_allclose(points["longitude_deg"], [self.pose[1]], atol=1e-10, rtol=0)
        np.testing.assert_allclose(points["height_assumed_ellipsoid_m"], [self.pose[2]], atol=1e-7, rtol=0)

    def test_enu_ecef_against_proj_topocentric(self):
        xyz = np.array([[0, 0, 0], [10, 2, -1], [-80, -50, 15], [120, 40, -3]])
        points, _ = geo.georeference_points(xyz, self.pose, np.eye(4))
        lat, lon, height = self.pose[:3]
        independent = Transformer.from_pipeline(
            f"+proj=topocentric +ellps=WGS84 +lat_0={lat} +lon_0={lon} +h_0={height}"
        )
        independent_enu = np.column_stack(independent.transform(*points["ecef_m"].T, errcheck=True))
        np.testing.assert_allclose(independent_enu, points["enu_m"], atol=1e-8, rtol=0)

    def test_geographic_and_utm_inverse(self):
        xyz = np.array([[0, 0, 0], [10, 2, -1], [-80, -50, 15], [120, 40, -3]])
        points, meta = geo.georeference_points(xyz, self.pose, np.eye(4))
        to_ecef = Transformer.from_crs(4979, 4978, always_xy=True)
        reconstructed = np.column_stack(to_ecef.transform(
            points["longitude_deg"], points["latitude_deg"], points["height_assumed_ellipsoid_m"]
        ))
        np.testing.assert_allclose(reconstructed, points["ecef_m"], atol=1e-7, rtol=0)
        to_wgs = Transformer.from_crs(meta["utm_epsg"], 4326, always_xy=True)
        longitude, latitude = to_wgs.transform(points["utm_easting_m"], points["utm_northing_m"])
        np.testing.assert_allclose(latitude, points["latitude_deg"], atol=1e-10, rtol=0)
        np.testing.assert_allclose(longitude, points["longitude_deg"], atol=1e-10, rtol=0)

    def test_height_offset_and_invalid_inputs(self):
        points, meta = geo.georeference_points(np.zeros((1, 3)), self.pose, np.eye(4), height_offset_m=40)
        np.testing.assert_allclose(points["height_assumed_ellipsoid_m"], [self.pose[2] + 40], atol=1e-7, rtol=0)
        self.assertFalse(meta["height_datum_verified"])
        with self.assertRaises(ValueError):
            geo.georeference_points(np.array([[np.nan, 0, 0]]), self.pose, np.eye(4))
        with self.assertRaises(ValueError):
            geo.georeference_points(np.zeros((1, 3)), self.pose, np.eye(4), utm_epsg=32732)
        with self.assertRaises(ValueError):
            geo.automatic_utm_epsg(89, 8)


if __name__ == "__main__":
    unittest.main()
