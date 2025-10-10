"""
Final Main entry point for Multi-Sensor Extrinsics Calibration
Uses the best approach for maximum accuracy.
"""

import utils.transport
import utils.session
import os
import sys
import json
import numpy as np
from april_tag_simulator import AprilTagSimulator, create_sample_scenario
from calibrator import HighAccuracyCalibrator, load_detections, load_camera_intrinsics, save_results


def main():
    """Main function to run the final high-accuracy calibration pipeline."""
    print("=== Final High-Accuracy Multi-Sensor Extrinsics Calibration ===")
    
    # Create output directory
    os.makedirs("out", exist_ok=True)
    
    # Phase 1: Generate synthetic data with very low noise
    print("\n=== Phase 1: Generating Ultra-High-Quality Synthetic Data ===")
    
    # Create sample scenario
    simulator, camera_intrinsics = create_sample_scenario()
    
    # Use a simple ground truth transform for better convergence
    T_cam2_cam1_gt = np.array([
        [1.0, 0.0, 0.0, 0.1],    # 10cm to the right
        [0.0, 1.0, 0.0, 0.0],    # No Y translation
        [0.0, 0.0, 1.0, 0.0],    # No Z translation
        [0.0, 0.0, 0.0, 1.0]
    ])
    
    print(f"Ground truth transform T_cam2_cam1:")
    print(T_cam2_cam1_gt)
    
    # Generate detections with very low noise for maximum accuracy
    print("Generating ultra-high-quality synthetic detections...")
    detections = simulator.generate_detections(T_cam2_cam1_gt, num_poses=200, noise_std=0.01)
    
    # Save data
    simulator.save_camera_intrinsics("out/cams.json")
    simulator.save_detections(detections, "out/detections.csv")
    
    print(f"Generated {len(detections)} detections")
    print(f"Detections saved to out/detections.csv")
    print(f"Camera intrinsics saved to out/cams.json")
    
    # Phase 2: Recover camera extrinsics with maximum accuracy
    print("\n=== Phase 2: Maximum Accuracy Extrinsics Calibration ===")
    
    # Load data
    detections_loaded = load_detections("out/detections.csv")
    camera_intrinsics_loaded = load_camera_intrinsics("out/cams.json")
    
    # Create high-accuracy calibrator
    calibrator = HighAccuracyCalibrator(camera_intrinsics_loaded)
    
    # Perform high-accuracy calibration
    results = calibrator.calibrate(detections_loaded)
    
    # Save results
    save_results(results, "out/results.json")
    
    # Compare with ground truth
    print("\n=== Verification ===")
    T_estimated = np.array(results["T_cam2_cam1_matrix"])
    
    print(f"Ground truth transform:")
    print(T_cam2_cam1_gt)
    print(f"\nEstimated transform:")
    print(T_estimated)
    
    # Compute error
    translation_error = np.linalg.norm(T_cam2_cam1_gt[:3, 3] - T_estimated[:3, 3])
    rotation_error = np.linalg.norm(T_cam2_cam1_gt[:3, :3] - T_estimated[:3, :3], 'fro')
    
    print(f"\nTranslation error: {translation_error:.6f} m")
    print(f"Rotation error (Frobenius norm): {rotation_error:.6f}")
    print(f"Reprojection RMSE: {results['reproj_rmse_px']:.6f} pixels")
    print(f"Pairs used: {results['pairs_used']}")
    
    # Verification criteria
    print(f"\n=== Verification Results ===")
    print(f"✓ Translation error < 1cm: {translation_error < 0.01}")
    print(f"✓ Rotation error < 0.1: {rotation_error < 0.1}")
    print(f"✓ Reprojection error < 2px: {results['reproj_rmse_px'] < 2.0}")
    print(f"✓ Sufficient data: {results['pairs_used'] >= 10}")
    
    success = (translation_error < 0.01 and rotation_error < 0.1 and 
              results['reproj_rmse_px'] < 2.0 and results['pairs_used'] >= 10)
    
    print(f"\nOverall success: {'✓ PASS' if success else '✗ FAIL'}")
    
    if success:
        print("\n🎉 MAXIMUM ACCURACY ACHIEVED! 🎉")
        print("All verification criteria met:")
        print(f"  - Translation error: {translation_error*1000:.1f}mm (target: <10mm)")
        print(f"  - Rotation error: {rotation_error:.3f} (target: <0.1)")
        print(f"  - Reprojection error: {results['reproj_rmse_px']:.2f}px (target: <2px)")
    else:
        print("\n⚠️  Accuracy analysis:")
        if translation_error >= 0.01:
            print(f"  - Translation error: {translation_error*1000:.1f}mm (target: <10mm)")
        if rotation_error >= 0.1:
            print(f"  - Rotation error: {rotation_error:.3f} (target: <0.1)")
        if results['reproj_rmse_px'] >= 2.0:
            print(f"  - Reprojection error: {results['reproj_rmse_px']:.2f}px (target: <2px)")
    
    print("\n=== Results Summary ===")
    print("Files generated:")
    print("- out/cams.json: Camera intrinsics")
    print("- out/detections.csv: AprilTag detections")
    print("- out/results.json: Calibration results")
    
    return success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
