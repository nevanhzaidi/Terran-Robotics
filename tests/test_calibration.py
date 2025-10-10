"""
Tests for the Multi-Sensor Extrinsics Calibration system
"""

import unittest
import numpy as np
import pandas as pd
import json
import os
import sys

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from april_tag_simulator import AprilTagSimulator
from calibrator import HighAccuracyCalibrator


class TestAprilTagSimulator(unittest.TestCase):
    """Test cases for AprilTag simulator."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.camera_intrinsics = {
            "cam1": {"fx": 920.0, "fy": 918.0, "cx": 640.0, "cy": 360.0},
            "cam2": {"fx": 915.0, "fy": 917.0, "cx": 640.0, "cy": 360.0},
            "tag_size_m": 0.12
        }
        self.simulator = AprilTagSimulator(self.camera_intrinsics, tag_size=0.12)
    
    def test_add_april_tag(self):
        """Test adding AprilTags to the world."""
        position = np.array([1.0, 2.0, 3.0])
        tag_id = 1
        
        self.simulator.add_april_tag(position, tag_id)
        
        self.assertEqual(len(self.simulator.tags_world), 1)
        self.assertEqual(self.simulator.tags_world[0]['id'], tag_id)
        np.testing.assert_array_equal(self.simulator.tags_world[0]['position'], position)
    
    def test_generate_camera_poses(self):
        """Test camera pose generation."""
        poses = self.simulator.generate_camera_poses(num_poses=10)
        
        self.assertEqual(len(poses), 10)
        
        # Check that all poses are valid 4x4 transformation matrices
        for pose in poses:
            self.assertEqual(pose.shape, (4, 4))
            # Check that rotation part is orthogonal
            R = pose[:3, :3]
            np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)
            # Check that determinant is 1
            self.assertAlmostEqual(np.linalg.det(R), 1.0, places=10)
    
    def test_project_points(self):
        """Test 3D to 2D point projection."""
        # Add a tag
        self.simulator.add_april_tag(np.array([0, 0, 1.0]), 1)
        
        # Create a simple camera pose (looking down Z axis)
        pose = np.eye(4)
        pose[2, 3] = 2.0  # Move camera 2m forward
        
        # Project tag corners
        tag = self.simulator.tags_world[0]
        corners_3d = tag['corners_local'] + tag['position']
        
        camera_matrix = np.array([
            [920.0, 0, 640.0],
            [0, 918.0, 360.0],
            [0, 0, 1]
        ])
        
        corners_2d = self.simulator.project_points(corners_3d, camera_matrix, pose)
        
        self.assertEqual(corners_2d.shape, (4, 2))
        # All points should be in front of camera (positive Z)
        self.assertTrue(np.all(corners_2d[:, 0] > 0))  # u coordinates
        self.assertTrue(np.all(corners_2d[:, 1] > 0))  # v coordinates


class TestExtrinsicsCalibrator(unittest.TestCase):
    """Test cases for extrinsics calibrator."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.camera_intrinsics = {
            "cam1": {"fx": 920.0, "fy": 918.0, "cx": 640.0, "cy": 360.0},
            "cam2": {"fx": 915.0, "fy": 917.0, "cx": 640.0, "cy": 360.0},
            "tag_size_m": 0.12
        }
        self.calibrator = HighAccuracyCalibrator(self.camera_intrinsics)
    
    def test_triangulate_points(self):
        """Test 3D point triangulation."""
        # Create simple test case
        points_cam1 = np.array([[100, 100], [200, 100], [200, 200], [100, 200]])
        points_cam2 = np.array([[110, 100], [210, 100], [210, 200], [110, 200]])
        
        # Simple transform: cam2 is 10cm to the right of cam1
        T_cam2_cam1 = np.array([
            [1, 0, 0, 0.1],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        points_3d = []
        for i in range(len(points_cam1)):
            point_3d = self.calibrator._triangulate_point(points_cam1[i], points_cam2[i], T_cam2_cam1)
            if point_3d is not None:
                points_3d.append(point_3d)
            else:
                points_3d.append(np.array([0, 0, 1]))
        points_3d = np.array(points_3d)
        
        self.assertEqual(points_3d.shape, (4, 3))
        # Points should be in front of cameras
        self.assertTrue(np.all(points_3d[:, 2] > 0))
    
    def test_reproject_points(self):
        """Test 3D to 2D reprojection."""
        # Simple test case
        points_3d = np.array([[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]])
        
        T_cam2_cam1 = np.eye(4)
        
        # Test both cameras
        points_2d_cam1 = []
        points_2d_cam2 = []
        for point_3d in points_3d:
            points_2d_cam1.append(self.calibrator._reproject_point(point_3d, 'cam1', T_cam2_cam1))
            points_2d_cam2.append(self.calibrator._reproject_point(point_3d, 'cam2', T_cam2_cam1))
        points_2d_cam1 = np.array(points_2d_cam1)
        points_2d_cam2 = np.array(points_2d_cam2)
        
        self.assertEqual(points_2d_cam1.shape, (4, 2))
        self.assertEqual(points_2d_cam2.shape, (4, 2))
        
        # Points should be in image bounds
        self.assertTrue(np.all(points_2d_cam1[:, 0] > 0))
        self.assertTrue(np.all(points_2d_cam1[:, 1] > 0))
        self.assertTrue(np.all(points_2d_cam2[:, 0] > 0))
        self.assertTrue(np.all(points_2d_cam2[:, 1] > 0))


class TestIntegration(unittest.TestCase):
    """Integration tests for the complete system."""
    
    def test_end_to_end_calibration(self):
        """Test complete end-to-end calibration process."""
        # Create simulator
        camera_intrinsics = {
            "cam1": {"fx": 920.0, "fy": 918.0, "cx": 640.0, "cy": 360.0},
            "cam2": {"fx": 915.0, "fy": 917.0, "cx": 640.0, "cy": 360.0},
            "tag_size_m": 0.12
        }
        
        simulator = AprilTagSimulator(camera_intrinsics, tag_size=0.12)
        
        # Add some tags
        simulator.add_april_tag(np.array([0, 0, 1.0]), 1)
        simulator.add_april_tag(np.array([1.0, 0, 1.0]), 2)
        simulator.add_april_tag(np.array([0, 1.0, 1.0]), 3)
        
        # Define ground truth transform
        T_cam2_cam1_gt = np.array([
            [1, 0, 0, 0.1],
            [0, 1, 0, 0.05],
            [0, 0, 1, 0.02],
            [0, 0, 0, 1]
        ])
        
        # Generate detections
        detections = simulator.generate_detections(T_cam2_cam1_gt, num_poses=20, noise_std=0.1)
        
        # Should have some detections
        self.assertGreater(len(detections), 0)
        
        # Create calibrator and run calibration
        calibrator = HighAccuracyCalibrator(camera_intrinsics)
        results = calibrator.calibrate(detections)
        
        # Check results structure
        self.assertIn('T_cam2_cam1_matrix', results)
        self.assertIn('rpy_deg', results)
        self.assertIn('xyz_m', results)
        self.assertIn('reproj_rmse_px', results)
        self.assertIn('pairs_used', results)
        
        # Check that we used some data
        self.assertGreater(results['pairs_used'], 0)
        
        # Check that reprojection error is reasonable
        self.assertLess(results['reproj_rmse_px'], 10.0)  # Should be less than 10 pixels


if __name__ == '__main__':
    unittest.main()
