"""
High-Accuracy Multi-Camera Extrinsics Calibration
Uses analytical and optimization-based solutions for maximum accuracy.
"""

import numpy as np
import json
import pandas as pd
from typing import Dict, List, Tuple, Optional
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
import cv2


class HighAccuracyCalibrator:
    """High-accuracy calibrator using analytical and optimization methods."""
    
    def __init__(self, camera_intrinsics: Dict):
        """
        Initialize calibrator with camera intrinsic parameters.
        
        Args:
            camera_intrinsics: Dictionary containing camera parameters
        """
        self.camera_intrinsics = camera_intrinsics
        self.camera_matrices = {}
        
        # Build camera matrices
        for cam_name, params in camera_intrinsics.items():
            if cam_name == "tag_size_m":
                continue
            self.camera_matrices[cam_name] = np.array([
                [params['fx'], 0, params['cx']],
                [0, params['fy'], params['cy']],
                [0, 0, 1]
            ])
    
    def solve_essential_matrix(self, detections: pd.DataFrame) -> np.ndarray:
        """
        Solve camera extrinsics using essential matrix approach.
        
        Args:
            detections: DataFrame containing AprilTag detections
            
        Returns:
            4x4 transformation matrix from cam1 to cam2
        """
        # Collect corresponding points
        points_cam1 = []
        points_cam2 = []
        
        grouped = detections.groupby(['ts_ns', 'tag_id'])
        
        for (timestamp, tag_id), group in grouped:
            if len(group) < 2:
                continue
                
            cam1_det = group[group['cam_id'] == 'cam1']
            cam2_det = group[group['cam_id'] == 'cam2']
            
            if len(cam1_det) == 0 or len(cam2_det) == 0:
                continue
            
            # Use center of tag as correspondence
            center_cam1 = np.array([
                (cam1_det.iloc[0]['u0'] + cam1_det.iloc[0]['u2']) / 2,
                (cam1_det.iloc[0]['v0'] + cam1_det.iloc[0]['v2']) / 2
            ])
            
            center_cam2 = np.array([
                (cam2_det.iloc[0]['u0'] + cam2_det.iloc[0]['u2']) / 2,
                (cam2_det.iloc[0]['v0'] + cam2_det.iloc[0]['v2']) / 2
            ])
            
            points_cam1.append(center_cam1)
            points_cam2.append(center_cam2)
        
        if len(points_cam1) < 8:
            return np.eye(4)
        
        points_cam1 = np.array(points_cam1)
        points_cam2 = np.array(points_cam2)
        
        try:
            # Compute essential matrix
            E, mask = cv2.findEssentialMat(points_cam1, points_cam2, 
                                          self.camera_matrices['cam1'],
                                          method=cv2.RANSAC, 
                                          prob=0.999, 
                                          threshold=0.5)
            
            # Recover pose
            _, R, t, mask = cv2.recoverPose(E, points_cam1, points_cam2, 
                                           self.camera_matrices['cam1'])
            
            # Build transformation matrix
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t.flatten()
            
            return T
        except:
            return np.eye(4)
    
    def optimize_transform(self, detections: pd.DataFrame, initial_transform: np.ndarray) -> np.ndarray:
        """
        Optimize transformation using non-linear least squares.
        
        Args:
            detections: DataFrame containing AprilTag detections
            initial_transform: Initial 4x4 transformation matrix
            
        Returns:
            Optimized 4x4 transformation matrix
        """
        # Convert to optimization parameters
        r = R.from_matrix(initial_transform[:3, :3])
        initial_params = np.concatenate([r.as_rotvec(), initial_transform[:3, 3]])
        
        def objective(params):
            # Convert parameters back to transformation matrix
            r = R.from_rotvec(params[:3])
            t = params[3:6]
            
            T = np.eye(4)
            T[:3, :3] = r.as_matrix()
            T[:3, 3] = t
            
            return self._compute_reprojection_error(T, detections)
        
        # Bounds for optimization
        bounds = [(-np.pi, np.pi), (-np.pi, np.pi), (-np.pi, np.pi),
                 (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5)]
        
        try:
            result = minimize(objective, initial_params, method='L-BFGS-B', 
                            bounds=bounds, options={'maxiter': 1000, 'ftol': 1e-12})
            
            # Convert back to transformation matrix
            r = R.from_rotvec(result.x[:3])
            t = result.x[3:6]
            
            T_optimized = np.eye(4)
            T_optimized[:3, :3] = r.as_matrix()
            T_optimized[:3, 3] = t
            
            return T_optimized
        except:
            return initial_transform
    
    def calibrate(self, detections: pd.DataFrame) -> Dict:
        """
        Perform high-accuracy calibration using essential matrix and optimization.
        
        Args:
            detections: DataFrame containing AprilTag detections
            
        Returns:
            Dictionary containing calibration results
        """
        print("Starting high-accuracy calibration...")
        
        # Step 1: Get initial estimate using essential matrix
        print("Computing initial estimate using essential matrix...")
        initial_transform = self.solve_essential_matrix(detections)
        
        # Step 2: Optimize using non-linear least squares
        print("Optimizing transformation...")
        T_optimized = self.optimize_transform(detections, initial_transform)
        
        # Compute final error
        final_error = self._compute_reprojection_error(T_optimized, detections)
        
        # Convert to RPY angles
        r = R.from_matrix(T_optimized[:3, :3])
        rpy_deg = r.as_euler('xyz', degrees=True)
        
        # Count pairs used
        pairs_used = len(detections.groupby(['ts_ns', 'tag_id']))
        
        results = {
            "T_cam2_cam1_matrix": T_optimized.tolist(),
            "rpy_deg": rpy_deg.tolist(),
            "xyz_m": T_optimized[:3, 3].tolist(),
            "reproj_rmse_px": final_error,
            "pairs_used": pairs_used
        }
        
        print(f"High-accuracy calibration completed!")
        print(f"Final reprojection error: {final_error:.6f} pixels")
        print(f"Pairs used: {pairs_used}")
        print(f"Translation: {T_optimized[:3, 3]}")
        print(f"Rotation (RPY): {rpy_deg}")
        
        return results
    
    def _compute_reprojection_error(self, T_cam2_cam1: np.ndarray, detections: pd.DataFrame) -> float:
        """
        Compute reprojection error for the given transformation.
        
        Args:
            T_cam2_cam1: 4x4 transformation matrix from cam1 to cam2
            detections: DataFrame containing AprilTag detections
            
        Returns:
            Root mean square reprojection error in pixels
        """
        total_error = 0.0
        num_points = 0
        
        # Process each detection pair
        grouped = detections.groupby(['ts_ns', 'tag_id'])
        
        for (timestamp, tag_id), group in grouped:
            if len(group) < 2:
                continue
                
            cam1_det = group[group['cam_id'] == 'cam1']
            cam2_det = group[group['cam_id'] == 'cam2']
            
            if len(cam1_det) == 0 or len(cam2_det) == 0:
                continue
            
            # Use center of tag as correspondence
            center_cam1 = np.array([
                (cam1_det.iloc[0]['u0'] + cam1_det.iloc[0]['u2']) / 2,
                (cam1_det.iloc[0]['v0'] + cam1_det.iloc[0]['v2']) / 2
            ])
            
            center_cam2 = np.array([
                (cam2_det.iloc[0]['u0'] + cam2_det.iloc[0]['u2']) / 2,
                (cam2_det.iloc[0]['v0'] + cam2_det.iloc[0]['v2']) / 2
            ])
            
            # Triangulate 3D point
            point_3d = self._triangulate_point(center_cam1, center_cam2, T_cam2_cam1)
            
            if point_3d is None:
                continue
            
            # Reproject and compute error
            center_cam1_reproj = self._reproject_point(point_3d, 'cam1', T_cam2_cam1)
            center_cam2_reproj = self._reproject_point(point_3d, 'cam2', T_cam2_cam1)
            
            error_cam1 = np.linalg.norm(center_cam1 - center_cam1_reproj)
            error_cam2 = np.linalg.norm(center_cam2 - center_cam2_reproj)
            
            total_error += error_cam1 + error_cam2
            num_points += 2
        
        return total_error / num_points if num_points > 0 else 0.0
    
    def _triangulate_point(self, point_cam1: np.ndarray, point_cam2: np.ndarray,
                          T_cam2_cam1: np.ndarray) -> Optional[np.ndarray]:
        """
        Triangulate a single 3D point from 2D correspondences.
        
        Args:
            point_cam1: 2D point in camera 1
            point_cam2: 2D point in camera 2
            T_cam2_cam1: 4x4 transformation matrix from cam1 to cam2
            
        Returns:
            3D point or None if triangulation fails
        """
        K1 = self.camera_matrices['cam1']
        K2 = self.camera_matrices['cam2']
        
        # Create projection matrices
        P1 = K1 @ np.hstack([np.eye(3), np.zeros((3, 1))])
        P2 = K2 @ T_cam2_cam1[:3, :]
        
        # Build system of equations
        A = np.zeros((4, 4))
        
        # Camera 1 equations
        u1, v1 = point_cam1
        A[0] = u1 * P1[2] - P1[0]
        A[1] = v1 * P1[2] - P1[1]
        
        # Camera 2 equations
        u2, v2 = point_cam2
        A[2] = u2 * P2[2] - P2[0]
        A[3] = v2 * P2[2] - P2[1]
        
        # Solve for 3D point
        try:
            _, _, V = np.linalg.svd(A)
            point_3d_homo = V[-1, :]
            if point_3d_homo[3] != 0:
                point_3d = point_3d_homo[:3] / point_3d_homo[3]
                return point_3d
        except:
            pass
        
        return None
    
    def _reproject_point(self, point_3d: np.ndarray, camera_name: str,
                        T_cam2_cam1: np.ndarray) -> np.ndarray:
        """
        Reproject a 3D point to 2D image coordinates.
        
        Args:
            point_3d: 3D point in world coordinates
            camera_name: Name of the camera ('cam1' or 'cam2')
            T_cam2_cam1: 4x4 transformation matrix from cam1 to cam2
            
        Returns:
            2D point in image coordinates
        """
        K = self.camera_matrices[camera_name]
        
        if camera_name == 'cam1':
            R = np.eye(3)
            t = np.zeros(3)
        else:
            R = T_cam2_cam1[:3, :3]
            t = T_cam2_cam1[:3, 3]
        
        # Transform to camera coordinates
        point_cam = R @ point_3d + t
        
        # Project to image
        point_2d_homo = K @ point_cam
        point_2d = point_2d_homo[:2] / point_2d_homo[2]
        
        return point_2d


def load_detections(filename: str) -> pd.DataFrame:
    """Load detections from CSV file."""
    return pd.read_csv(filename)


def load_camera_intrinsics(filename: str) -> Dict:
    """Load camera intrinsics from JSON file."""
    with open(filename, 'r') as f:
        return json.load(f)


def save_results(results: Dict, filename: str):
    """Save calibration results to JSON file."""
    with open(filename, 'w') as f:
        json.dump(results, f, indent=2)


if __name__ == "__main__":
    # Load data
    detections = load_detections("out/detections.csv")
    camera_intrinsics = load_camera_intrinsics("out/cams.json")
    
    # Create calibrator
    calibrator = HighAccuracyCalibrator(camera_intrinsics)
    
    # Perform calibration
    results = calibrator.calibrate(detections)
    
    # Save results
    save_results(results, "out/results.json")
    
    print("Calibration results saved to out/results.json")
