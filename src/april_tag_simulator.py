"""
AprilTag Detection Simulator for Multi-Camera Extrinsics Calibration
Generates synthetic AprilTag detection data for two cameras mounted on a rigid body.
"""

import numpy as np
import json
import pandas as pd
from typing import Dict, List, Tuple, Optional
import cv2


class AprilTagSimulator:
    """Simulates AprilTag detections for multi-camera calibration."""
    
    def __init__(self, camera_intrinsics: Dict, tag_size: float = 0.12):
        """
        Initialize the simulator.
        
        Args:
            camera_intrinsics: Dictionary with camera intrinsic parameters
            tag_size: Physical size of AprilTags in meters
        """
        self.camera_intrinsics = camera_intrinsics
        self.tag_size = tag_size
        self.tags_world = [] 
        self.detections = []
        
    def add_april_tag(self, position: np.ndarray, tag_id: int):
        """
        Add an AprilTag to the world at a fixed position.
        
        Args:
            position: 3D position [x, y, z] in world coordinates
            tag_id: Unique identifier for the tag
        """
        # Define tag corners in local tag frame (counter-clockwise)
        half_size = self.tag_size / 2
        tag_corners_local = np.array([
            [-half_size, -half_size, 0],
            [half_size, -half_size, 0],
            [half_size, half_size, 0],
            [-half_size, half_size, 0]
        ])
        
        self.tags_world.append({
            'id': tag_id,
            'position': position,
            'corners_local': tag_corners_local
        })
    
    def generate_camera_poses(self, num_poses: int = 50) -> List[np.ndarray]:
        """
        Generate a sequence of camera poses as the rig moves through the environment.
        
        Args:
            num_poses: Number of poses to generate
            
        Returns:
            List of 4x4 transformation matrices (world to camera)
        """
        poses = []
        
        # Create a circular trajectory with some height variation
        for i in range(num_poses):
            t = i / (num_poses - 1) * 2 * np.pi
            
            # Camera position (circular path with height variation)
            x = 2.0 * np.cos(t)
            y = 2.0 * np.sin(t)
            z = 1.5 + 0.3 * np.sin(2 * t)
            
            # Camera orientation (look towards center with some variation)
            center = np.array([0, 0, 1.0])
            camera_pos = np.array([x, y, z])
            
            # Look at center
            forward = center - camera_pos
            forward = forward / np.linalg.norm(forward)
            
            # Right vector (cross product with up)
            right = np.cross(forward, np.array([0, 0, 1]))
            right = right / np.linalg.norm(right)
            
            # Up vector
            up = np.cross(right, forward)
            
            # Create rotation matrix
            R = np.column_stack([right, up, -forward])
            
            # Create transformation matrix
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = camera_pos
            
            poses.append(T)
            
        return poses
    
    def project_points(self, points_3d: np.ndarray, camera_matrix: np.ndarray, 
                      pose: np.ndarray) -> np.ndarray:
        """
        Project 3D points to 2D image coordinates.
        
        Args:
            points_3d: 3D points in world coordinates (N, 3)
            camera_matrix: 3x3 camera intrinsic matrix
            pose: 4x4 transformation matrix (world to camera)
            
        Returns:
            2D points in image coordinates (N, 2)
        """
        # Transform points to camera coordinates
        points_3d_homo = np.column_stack([points_3d, np.ones(points_3d.shape[0])])
        points_cam = (pose @ points_3d_homo.T).T[:, :3]
        
        # Project to image plane
        points_2d_homo = (camera_matrix @ points_cam.T).T
        
        # Convert to pixel coordinates
        points_2d = points_2d_homo[:, :2] / points_2d_homo[:, 2:3]
        
        return points_2d
    
    def is_point_in_fov(self, point_3d: np.ndarray, pose: np.ndarray, 
                       image_size: Tuple[int, int] = (1280, 720)) -> bool:
        """
        Check if a 3D point is within the camera's field of view.
        
        Args:
            point_3d: 3D point in world coordinates
            pose: 4x4 transformation matrix (world to camera)
            image_size: (width, height) of the image
            
        Returns:
            True if point is in FOV
        """
        # Transform to camera coordinates
        point_cam = pose @ np.append(point_3d, 1)
        
        # Check if in front of camera
        if point_cam[2] <= 0:
            return False
            
        # Check if within reasonable bounds (assuming 60 degree FOV)
        max_distance = 10.0  # meters
        if np.linalg.norm(point_cam[:3]) > max_distance:
            return False
            
        return True
    
    def generate_detections(self, T_cam2_cam1: np.ndarray, 
                          num_poses: int = 50, 
                          noise_std: float = 0.5) -> pd.DataFrame:
        """
        Generate synthetic AprilTag detections for both cameras.
        
        Args:
            T_cam2_cam1: 4x4 transformation matrix from cam1 to cam2
            num_poses: Number of camera poses to simulate
            noise_std: Standard deviation of pixel noise
            
        Returns:
            DataFrame with detection data
        """
        poses_cam1 = self.generate_camera_poses(num_poses)
        detections = []
        
        for i, pose_cam1 in enumerate(poses_cam1):
            # Calculate cam2 pose relative to cam1
            pose_cam2 = pose_cam1 @ np.linalg.inv(T_cam2_cam1)
            
            timestamp = 1710000000000 + i * 100000000  # 0.1s intervals in nanoseconds
            
            for camera_name, pose in [("cam1", pose_cam1), ("cam2", pose_cam2)]:
                camera_params = self.camera_intrinsics[camera_name]
                camera_matrix = np.array([
                    [camera_params['fx'], 0, camera_params['cx']],
                    [0, camera_params['fy'], camera_params['cy']],
                    [0, 0, 1]
                ])
                
                for tag in self.tags_world:
                    # Check if tag is visible
                    if not self.is_point_in_fov(tag['position'], pose):
                        continue
                    
                    # Project tag corners to image
                    corners_3d = tag['corners_local'] + tag['position']
                    corners_2d = self.project_points(corners_3d, camera_matrix, pose)
                    
                    # Add noise
                    noise = np.random.normal(0, noise_std, corners_2d.shape)
                    corners_2d_noisy = corners_2d + noise
                    
                    # Create detection record
                    detection = {
                        'ts_ns': timestamp,
                        'cam_id': camera_name,
                        'tag_id': tag['id'],
                        'u0': corners_2d_noisy[0, 0],
                        'v0': corners_2d_noisy[0, 1],
                        'u1': corners_2d_noisy[1, 0],
                        'v1': corners_2d_noisy[1, 1],
                        'u2': corners_2d_noisy[2, 0],
                        'v2': corners_2d_noisy[2, 1],
                        'u3': corners_2d_noisy[3, 0],
                        'v3': corners_2d_noisy[3, 1]
                    }
                    
                    detections.append(detection)
        
        return pd.DataFrame(detections)
    
    def save_camera_intrinsics(self, filename: str):
        """Save camera intrinsics to JSON file."""
        with open(filename, 'w') as f:
            json.dump(self.camera_intrinsics, f, indent=2)
    
    def save_detections(self, detections: pd.DataFrame, filename: str):
        """Save detections to CSV file."""
        detections.to_csv(filename, index=False)


def create_sample_scenario():
    """Create a sample scenario with multiple AprilTags."""
    # Camera intrinsics
    camera_intrinsics = {
        "cam1": {"fx": 920.0, "fy": 918.0, "cx": 640.0, "cy": 360.0},
        "cam2": {"fx": 915.0, "fy": 917.0, "cx": 640.0, "cy": 360.0},
        "tag_size_m": 0.12
    }
    
    # Create simulator
    simulator = AprilTagSimulator(camera_intrinsics, tag_size=0.12)
    
    # Add AprilTags at various positions
    tag_positions = [
        np.array([0, 0, 1.0]),      # Center
        np.array([1.5, 0, 1.2]),    # Right
        np.array([-1.5, 0, 1.2]),   # Left
        np.array([0, 1.5, 1.2]),    # Front
        np.array([0, -1.5, 1.2]),   # Back
        np.array([2.0, 2.0, 0.8]),  # Corner
        np.array([-2.0, -2.0, 0.8]) # Opposite corner
    ]
    
    for i, pos in enumerate(tag_positions):
        simulator.add_april_tag(pos, tag_id=i)
    
    return simulator, camera_intrinsics


if __name__ == "__main__":
    # Create sample scenario
    simulator, camera_intrinsics = create_sample_scenario()
    
    # Define ground truth transform (cam2 relative to cam1)
    # cam2 is 10cm to the right and 5cm up from cam1
    T_cam2_cam1_gt = np.array([
        [1, 0, 0, 0.1],
        [0, 1, 0, 0.05],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # Generate detections
    detections = simulator.generate_detections(T_cam2_cam1_gt, num_poses=30)
    
    # Save data
    simulator.save_camera_intrinsics("out/cams.json")
    simulator.save_detections(detections, "out/detections.csv")
    
    print(f"Generated {len(detections)} detections")
    print(f"Detections saved to out/detections.csv")
    print(f"Camera intrinsics saved to out/cams.json")
