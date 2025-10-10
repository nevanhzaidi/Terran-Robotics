"""
Visualization tools for Multi-Sensor Extrinsics Calibration
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
from typing import Dict, List, Optional
import json


class CalibrationVisualizer:
    """Visualization tools for calibration results."""
    
    def __init__(self, camera_intrinsics: Dict):
        """
        Initialize visualizer.
        
        Args:
            camera_intrinsics: Dictionary with camera intrinsic parameters
        """
        self.camera_intrinsics = camera_intrinsics
    
    def plot_camera_trajectory(self, detections: pd.DataFrame, 
                              T_cam2_cam1: np.ndarray, 
                              save_path: Optional[str] = None):
        """
        Plot the camera trajectory and tag positions.
        
        Args:
            detections: DataFrame with detection data
            T_cam2_cam1: 4x4 transformation matrix from cam1 to cam2
            save_path: Optional path to save the plot
        """
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Extract camera poses from detections
        timestamps = sorted(detections['ts_ns'].unique())
        
        cam1_positions = []
        cam2_positions = []
        
        for ts in timestamps:
            ts_detections = detections[detections['ts_ns'] == ts]
            
            # Get cam1 and cam2 detections
            cam1_det = ts_detections[ts_detections['cam_id'] == 'cam1']
            cam2_det = ts_detections[ts_detections['cam_id'] == 'cam2']
            
            if len(cam1_det) > 0 and len(cam2_det) > 0:
                # Estimate camera position from tag detections
                # This is a simplified approach - in practice you'd use proper pose estimation
                cam1_pos = self._estimate_camera_position(cam1_det)
                if cam1_pos is not None:
                    cam1_positions.append(cam1_pos)
                    cam2_positions.append(cam1_pos + T_cam2_cam1[:3, 3])
        
        if cam1_positions:
            cam1_positions = np.array(cam1_positions)
            cam2_positions = np.array(cam2_positions)
            
            # Plot camera trajectories
            ax.plot(cam1_positions[:, 0], cam1_positions[:, 1], cam1_positions[:, 2], 
                   'b-', label='Camera 1', linewidth=2)
            ax.plot(cam2_positions[:, 0], cam2_positions[:, 1], cam2_positions[:, 2], 
                   'r-', label='Camera 2', linewidth=2)
            
            # Plot camera positions
            ax.scatter(cam1_positions[:, 0], cam1_positions[:, 1], cam1_positions[:, 2], 
                      c='blue', s=50, alpha=0.7)
            ax.scatter(cam2_positions[:, 0], cam2_positions[:, 1], cam2_positions[:, 2], 
                      c='red', s=50, alpha=0.7)
        
        # Plot AprilTags
        tag_positions = self._extract_tag_positions(detections)
        if tag_positions:
            tag_positions = np.array(tag_positions)
            ax.scatter(tag_positions[:, 0], tag_positions[:, 1], tag_positions[:, 2], 
                      c='green', s=100, marker='s', label='AprilTags', alpha=0.8)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Camera Trajectory and AprilTag Positions')
        ax.legend()
        
        # Set equal aspect ratio
        max_range = np.array([ax.get_xlim(), ax.get_ylim(), ax.get_zlim()]).ptp() / 2.0
        mid_x = (ax.get_xlim()[0] + ax.get_xlim()[1]) * 0.5
        mid_y = (ax.get_ylim()[0] + ax.get_ylim()[1]) * 0.5
        mid_z = (ax.get_zlim()[0] + ax.get_zlim()[1]) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def plot_reprojection_errors(self, detections: pd.DataFrame, 
                                T_cam2_cam1: np.ndarray,
                                save_path: Optional[str] = None):
        """
        Plot reprojection errors for each detection.
        
        Args:
            detections: DataFrame with detection data
            T_cam2_cam1: 4x4 transformation matrix from cam1 to cam2
            save_path: Optional path to save the plot
        """
        # This would require implementing the reprojection error computation
        # For now, create a placeholder plot
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # Plot detection count over time
        timestamps = sorted(detections['ts_ns'].unique())
        detection_counts = []
        
        for ts in timestamps:
            count = len(detections[detections['ts_ns'] == ts])
            detection_counts.append(count)
        
        ax1.plot(timestamps, detection_counts, 'b-', linewidth=2)
        ax1.set_xlabel('Timestamp (ns)')
        ax1.set_ylabel('Number of Detections')
        ax1.set_title('Detection Count Over Time')
        ax1.grid(True, alpha=0.3)
        
        # Plot detection distribution by camera
        camera_counts = detections['cam_id'].value_counts()
        ax2.bar(camera_counts.index, camera_counts.values, color=['blue', 'red'])
        ax2.set_xlabel('Camera')
        ax2.set_ylabel('Number of Detections')
        ax2.set_title('Detection Distribution by Camera')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def plot_calibration_results(self, results: Dict, 
                                ground_truth: Optional[np.ndarray] = None,
                                save_path: Optional[str] = None):
        """
        Plot calibration results and comparison with ground truth.
        
        Args:
            results: Calibration results dictionary
            ground_truth: Optional ground truth transformation matrix
            save_path: Optional path to save the plot
        """
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10))
        
        # Plot translation components
        xyz = results['xyz_m']
        ax1.bar(['X', 'Y', 'Z'], xyz, color=['red', 'green', 'blue'], alpha=0.7)
        ax1.set_ylabel('Translation (m)')
        ax1.set_title('Estimated Translation')
        ax1.grid(True, alpha=0.3)
        
        if ground_truth is not None:
            xyz_gt = ground_truth[:3, 3]
            ax1.bar(['X', 'Y', 'Z'], xyz_gt, color=['red', 'green', 'blue'], 
                   alpha=0.3, label='Ground Truth')
            ax1.legend()
        
        # Plot rotation components (RPY)
        rpy = results['rpy_deg']
        ax2.bar(['Roll', 'Pitch', 'Yaw'], rpy, color=['red', 'green', 'blue'], alpha=0.7)
        ax2.set_ylabel('Rotation (degrees)')
        ax2.set_title('Estimated Rotation (RPY)')
        ax2.grid(True, alpha=0.3)
        
        if ground_truth is not None:
            from scipy.spatial.transform import Rotation as R
            r_gt = R.from_matrix(ground_truth[:3, :3])
            rpy_gt = r_gt.as_euler('xyz', degrees=True)
            ax2.bar(['Roll', 'Pitch', 'Yaw'], rpy_gt, color=['red', 'green', 'blue'], 
                   alpha=0.3, label='Ground Truth')
            ax2.legend()
        
        # Plot reprojection error
        ax3.bar(['RMSE'], [results['reproj_rmse_px']], color='orange', alpha=0.7)
        ax3.set_ylabel('Reprojection Error (pixels)')
        ax3.set_title('Reprojection Error')
        ax3.grid(True, alpha=0.3)
        
        # Plot data usage
        ax4.bar(['Pairs Used'], [results['pairs_used']], color='purple', alpha=0.7)
        ax4.set_ylabel('Number of Pairs')
        ax4.set_title('Data Usage')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def _estimate_camera_position(self, detections: pd.DataFrame) -> Optional[np.ndarray]:
        """
        Estimate camera position from tag detections.
        This is a simplified approach for visualization.
        """
        if len(detections) == 0:
            return None
        
        # Use the first detection to estimate position
        # This is a very simplified approach
        det = detections.iloc[0]
        
        # Estimate distance based on tag size and image size
        # This is not accurate but good enough for visualization
        tag_size_pixels = np.sqrt((det['u2'] - det['u0'])**2 + (det['v2'] - det['v0'])**2)
        estimated_distance = 2.0  # Simplified assumption
        
        # Estimate position (simplified)
        center_u = (det['u0'] + det['u2']) / 2
        center_v = (det['v1'] + det['v3']) / 2
        
        # Convert to 3D position (simplified)
        fx = self.camera_intrinsics['cam1']['fx']
        fy = self.camera_intrinsics['cam1']['fy']
        cx = self.camera_intrinsics['cam1']['cx']
        cy = self.camera_intrinsics['cam1']['cy']
        
        x = (center_u - cx) * estimated_distance / fx
        y = (center_v - cy) * estimated_distance / fy
        z = estimated_distance
        
        return np.array([x, y, z])
    
    def _extract_tag_positions(self, detections: pd.DataFrame) -> List[np.ndarray]:
        """
        Extract AprilTag positions from detections.
        This is a simplified approach for visualization.
        """
        tag_positions = []
        
        # Group by tag ID
        for tag_id in detections['tag_id'].unique():
            tag_detections = detections[detections['tag_id'] == tag_id]
            
            if len(tag_detections) > 0:
                # Use first detection to estimate position
                pos = self._estimate_camera_position(tag_detections)
                if pos is not None:
                    tag_positions.append(pos)
        
        return tag_positions


def create_visualization_report(detections_file: str, results_file: str, 
                              camera_intrinsics_file: str,
                              ground_truth_file: Optional[str] = None):
    """
    Create a comprehensive visualization report.
    
    Args:
        detections_file: Path to detections CSV file
        results_file: Path to results JSON file
        camera_intrinsics_file: Path to camera intrinsics JSON file
        ground_truth_file: Optional path to ground truth JSON file
    """
    # Load data
    detections = pd.read_csv(detections_file)
    
    with open(results_file, 'r') as f:
        results = json.load(f)
    
    with open(camera_intrinsics_file, 'r') as f:
        camera_intrinsics = json.load(f)
    
    ground_truth = None
    if ground_truth_file and os.path.exists(ground_truth_file):
        with open(ground_truth_file, 'r') as f:
            ground_truth = np.array(json.load(f)['T_cam2_cam1_matrix'])
    
    # Create visualizer
    visualizer = CalibrationVisualizer(camera_intrinsics)
    
    # Load transformation matrix
    T_cam2_cam1 = np.array(results['T_cam2_cam1_matrix'])
    
    # Create visualizations
    print("Creating visualization report...")
    
    # Plot camera trajectory
    visualizer.plot_camera_trajectory(detections, T_cam2_cam1, 
                                    save_path="out/camera_trajectory.png")
    
    # Plot reprojection errors
    visualizer.plot_reprojection_errors(detections, T_cam2_cam1,
                                      save_path="out/reprojection_errors.png")
    
    # Plot calibration results
    visualizer.plot_calibration_results(results, ground_truth,
                                      save_path="out/calibration_results.png")
    
    print("Visualization report saved to out/ directory")


if __name__ == "__main__":
    import os
    
    # Create output directory
    os.makedirs("out", exist_ok=True)
    
    # Create visualization report
    create_visualization_report("out/detections.csv", "out/results.json", 
                              "out/cams.json")

