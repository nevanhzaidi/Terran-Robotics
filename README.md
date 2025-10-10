# Multi-Sensor Extrinsics Calibration from AprilTag Detections

This project implements a complete solution for estimating the rigid transform T_cam2_cam1 between two cameras mounted on a rigid body using AprilTag detections. The solution includes both synthetic data generation and calibration algorithms.

## Overview

The system consists of two main phases:

1. **Phase 1: Synthetic Data Generation** - Creates realistic AprilTag detection data for two cameras moving through an environment
2. **Phase 2: Extrinsics Calibration** - Recovers the rigid transform between cameras using the detection data

## Features

- ✅ **Synthetic Data Generation**: Realistic AprilTag detection simulation with configurable noise
- ✅ **Robust Calibration**: Non-linear optimization for accurate transform estimation
- ✅ **Multiple Camera Support**: Handles different intrinsic parameters for each camera
- ✅ **Comprehensive Testing**: Unit tests and integration tests for verification
- ✅ **Visualization Tools**: 3D trajectory plotting and error analysis
- ✅ **Data Format Compliance**: Full support for required JSON/CSV formats

## Project Structure

```
├── src/                          # Core implementation
│   ├── april_tag_simulator.py   # Synthetic data generation
│   ├── calibrator.py            # High-accuracy calibration algorithms
│   ├── main.py                  # Main entry point
│   └── visualization.py         # Visualization tools
├── tests/                       # Test suite
│   └── test_calibration.py      # Comprehensive tests
├── bin/                         # CLI entry points
│   └── calibrate               # Command-line interface
├── out/                         # Output directory (generated)
├── .gitignore                   # Git ignore rules
├── run.sh                       # Setup and run script
├── run_tests.sh                 # Test runner script
└── README.md                    # This file
```

## Quick Start

### Prerequisites

- Python 3.7+
- Required packages: numpy, opencv-python, scipy, matplotlib, pandas

### Running the Solution

1. **Run the complete pipeline:**
   ```bash
   ./run.sh
   ```

2. **Run tests:**
   ```bash
   ./run_tests.sh
   ```

3. **Generate visualizations:**
   ```bash
   python src/visualization.py
   ```

### Manual Setup

If you prefer manual setup:

```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install numpy opencv-python scipy matplotlib pandas

# Run the solution
python src/main.py

# Run tests
python -m pytest tests/ -v
```

## Usage

### Basic Usage

The main script runs the complete pipeline:

```python
from src.main import main
success = main()
```

### Custom Scenarios

You can create custom scenarios by modifying the simulator:

```python
from src.april_tag_simulator import AprilTagSimulator

# Define camera intrinsics
camera_intrinsics = {
    "cam1": {"fx": 920.0, "fy": 918.0, "cx": 640.0, "cy": 360.0},
    "cam2": {"fx": 915.0, "fy": 917.0, "cx": 640.0, "cy": 360.0},
    "tag_size_m": 0.12
}

# Create simulator
simulator = AprilTagSimulator(camera_intrinsics, tag_size=0.12)

# Add AprilTags
simulator.add_april_tag(np.array([0, 0, 1.0]), tag_id=1)
simulator.add_april_tag(np.array([1.0, 0, 1.0]), tag_id=2)

# Define camera transform
T_cam2_cam1 = np.array([
    [1, 0, 0, 0.1],
    [0, 1, 0, 0.05],
    [0, 0, 1, 0.02],
    [0, 0, 0, 1]
])

# Generate detections
detections = simulator.generate_detections(T_cam2_cam1, num_poses=50)
```

### Calibration

Perform calibration on detection data:

```python
from src.extrinsics_calibration import ExtrinsicsCalibrator

# Load data
detections = pd.read_csv("out/detections.csv")
camera_intrinsics = json.load(open("out/cams.json"))

# Create calibrator
calibrator = ExtrinsicsCalibrator(camera_intrinsics)

# Perform calibration
results = calibrator.calibrate(detections)
```

## Data Formats

### Camera Intrinsics (cams.json)
```json
{
  "cam1": {"fx": 920.0, "fy": 918.0, "cx": 640.0, "cy": 360.0},
  "cam2": {"fx": 915.0, "fy": 917.0, "cx": 640.0, "cy": 360.0},
  "tag_size_m": 0.12
}
```

### Detections (detections.csv)
```csv
ts_ns,cam_id,tag_id,u0,v0,u1,v1,u2,v2,u3,v3
1710000000000,cam1,5,598.2,341.0,645.4,338.7,647.0,385.1,599.8,388.0
1710000000100,cam2,5,612.3,339.4,658.1,337.2,660.2,383.3,614.5,385.9
```

### Results (results.json)
```json
{
  "T_cam2_cam1_matrix": [[r11,r12,r13,tx], [r21,r22,r23,ty], [r31,r32,r33,tz], [0,0,0,1]],
  "rpy_deg": [roll, pitch, yaw],
  "xyz_m": [tx, ty, tz],
  "reproj_rmse_px": 0.72,
  "pairs_used": 124
}
```

## Algorithm Details

### Phase 1: Synthetic Data Generation

The simulator creates realistic AprilTag detection data by:

1. **Camera Pose Generation**: Creates a circular trajectory with height variation
2. **Tag Placement**: Positions AprilTags at fixed locations in the world
3. **Projection**: Projects 3D tag corners to 2D image coordinates using camera intrinsics
4. **Noise Addition**: Adds realistic pixel noise to simulate real-world conditions

### Phase 2: Extrinsics Calibration

The calibration process uses:

1. **Initial Estimation**: Uses essential matrix to estimate initial transform
2. **Triangulation**: Triangulates 3D points from corresponding 2D detections
3. **Non-linear Optimization**: Minimizes reprojection error using Levenberg-Marquardt
4. **Robust Estimation**: Handles outliers and missing data gracefully

## Verification

### How We Verify Correctness

1. **Ground Truth Comparison**: Compare estimated transform with known ground truth
2. **Reprojection Error**: Measure pixel-level accuracy of reprojected points
3. **Unit Tests**: Comprehensive test suite covering all components
4. **Integration Tests**: End-to-end testing of the complete pipeline

### Verification Criteria

- ✅ Translation error < 1cm
- ✅ Rotation error < 0.1 (Frobenius norm)
- ✅ Reprojection error < 2 pixels
- ✅ Sufficient data usage (≥10 pairs)

### Running Verification

```bash
# Run all tests
./run_tests.sh

# Run specific test
python -m pytest tests/test_calibration.py::TestIntegration::test_end_to_end_calibration -v
```

## Performance

### Typical Results

- **Translation Accuracy**: < 1cm error
- **Rotation Accuracy**: < 0.1 radian error
- **Reprojection Error**: < 1 pixel RMSE
- **Processing Time**: < 1 second for 50 poses
- **Data Requirements**: 10+ tag pairs for reliable calibration

### Optimization

The system uses several optimizations:

- **Efficient Triangulation**: OpenCV-based triangulation for speed
- **Sparse Optimization**: Only optimize 6 DOF (3 rotation + 3 translation)
- **Robust Estimation**: RANSAC-based outlier rejection
- **Vectorized Operations**: NumPy-based computations for speed

## Stretch Goals Implemented

### Visualization
- 3D trajectory plotting with matplotlib
- Camera position visualization
- AprilTag position display
- Error analysis plots

### Noise Analysis
- Configurable noise levels in simulation
- Impact analysis of noise on calibration accuracy
- Robust estimation techniques

### Real-Time Capabilities
- Optimized algorithms for fast processing
- Efficient data structures
- Minimal memory footprint

## Troubleshooting

### Common Issues

1. **Import Errors**: Ensure all dependencies are installed
2. **No Detections**: Check camera poses and tag visibility
3. **Poor Calibration**: Increase number of poses or add more tags
4. **Memory Issues**: Reduce number of poses or detections

### Debug Mode

Enable debug output by setting environment variable:

```bash
export DEBUG=1
python src/main.py
```

## Contributing

This project was developed as part of a technical challenge. The code is designed to be:

- **Modular**: Easy to extend with new features
- **Well-tested**: Comprehensive test coverage
- **Documented**: Clear documentation and comments
- **Efficient**: Optimized for performance

## License

This project is part of a technical challenge and is provided as-is for evaluation purposes.

## Contact

For questions about this implementation, please refer to the technical challenge documentation or contact the development team.
