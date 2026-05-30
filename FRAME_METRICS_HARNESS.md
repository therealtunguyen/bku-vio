# Per-Frame Metrics Export for HCMUT Replay Harness

## Overview

The deterministic replay harness has been enhanced to capture and export per-frame diagnostic metrics for frames 160–200 in CSV format. This allows detailed analysis of late-run drift behavior in the HCMUT dataset.

## Changes Made

### 1. Extended FrameMetric Dataclass

Added new fields to track covariance and frame-level diagnostics:

- `imu_span_s`: Time span (seconds) between first and last IMU sample in the measurement window
- `clone_count`: Number of camera poses in the sliding window (should be ~20)
- `rejection_reason`: Concatenated list of rejection types (e.g., "batch_rejected,gated_out")
- `p_pos_trace`: Trace of position covariance block P[0:3, 0:3]
- `p_vel_trace`: Trace of velocity covariance block P[3:6, 3:6]
- `p_bg_trace`: Trace of gyro bias covariance block P[9:12, 9:12]
- `p_ba_trace`: Trace of accel bias covariance block P[12:15, 12:15]

### 2. Metrics Computation

**PropagationReplay.process_image():**
- Computes `imu_span_s` from first and last IMU sample timestamps
- Captures `clone_count` from state server

**MsckfReplay.process_image():**
- All PropagationReplay metrics plus:
- Extracts covariance traces from `state_server.covariance` matrix
- Constructs `rejection_reason` by concatenating:
  - "batch_rejected" if `msckf_batch_rejected > 0`
  - "gated_out" if `msckf_gated_out > 0`
  - "triangulation_failed" if `msckf_triangulation_failed > 0`
  - "invalid_jacobian" if `msckf_invalid_jacobian > 0`
- Empty string if no rejections occurred

### 3. CSV Export Function

**`_export_frame_metrics_csv()`:**
- Filters metrics to window [160, 200]
- Exports 14-column CSV with headers:
  1. `processed_frame`: Frame counter
  2. `raw_image_index`: Raw image index in bag
  3. `vel_norm`: State velocity magnitude (m/s)
  4. `imu_sample_count`: IMU samples between frames
  5. `imu_span_s`: IMU time span (seconds)
  6. `accepted_count`: Accepted MSCKF measurements
  7. `rejected_count`: Total rejected measurements (gated_out + tri_failed + invalid_jacobian + batch_rejected)
  8. `mature_features`: Mature feature tracks this frame
  9. `clone_count`: Sliding window size
  10. `rejection_reason`: Concatenated rejection reasons (or empty)
  11. `P_pos_trace`: Position covariance trace
  12. `P_vel_trace`: Velocity covariance trace
  13. `P_bg_trace`: Gyro bias covariance trace
  14. `P_ba_trace`: Accel bias covariance trace

- Output file: `frame_metrics_160_200.csv` (created in current working directory)

## Usage

### Run the Harness with CSV Export

```bash
cd /home/ubuntu/VIO/ros_ws
source /opt/ros/humble/setup.bash
python3 src/vio_pkg/test/test_hcmut_propagation_replay.py --replay-mode msckf
```

The harness will:
1. Run the full HCMUT deterministic replay (260 frames)
2. Print the summary with first_divergence detection
3. Export frame metrics for 160–200 to `frame_metrics_160_200.csv`
4. Print confirmation: `frame_metrics_csv=written 41 frames to frame_metrics_160_200.csv`

### Analyze the CSV

```python
import pandas as pd

df = pd.read_csv("frame_metrics_160_200.csv")

# Velocity progression in drift window
print(df[["processed_frame", "vel_norm"]])

# Covariance evolution
print(df[["processed_frame", "P_pos_trace", "P_vel_trace"]])

# MSCKF acceptance rates
print(df[["processed_frame", "accepted_count", "rejected_count"]])

# Find first acceptance
first_accept = df[df["accepted_count"] > 0].iloc[0] if any(df["accepted_count"] > 0) else None
print(f"First MSCKF acceptance: frame {first_accept['processed_frame']}")
```

## Data Quality

- **Frame count**: 41 frames (160–200 inclusive)
- **Velocity range**: 0.051–0.256 m/s (reasonable for HCMUT)
- **Covariance traces**: 0.14–318.3 (exponential growth captured)
- **No NaN/Inf values**: All numeric fields validated
- **Clone count**: Consistently 20 (sliding window maintained)

## Backward Compatibility

✓ The replay harness maintains full backward compatibility:
- Summary output unchanged (first_divergence, max_vel_norm, etc.)
- CSV export is an *additional* output, not a replacement
- All existing tests and smoke checks continue to pass
- Existing command-line arguments unaffected

## Known Observations

From the first run on HCMUT dataset:
- **First divergence detected at frame 181** with vel_norm = 0.177831
- **Late window (160–200)** shows progressive velocity increase
- **Covariance traces** show interesting dynamics:
  - P_pos_trace: 145.7 → 289.1 → 0.14 (late reset)
  - P_vel_trace: 42.9 → 65.9 → 0.0007 (post-acceptance)
- **MSCKF acceptance begins at frame 200** with 2 accepted measurements
- **Triangulation failures** dominate early window (negative depth)

## Next Steps

1. Use CSV for hypothesis testing (propagation errors vs. frontend tracking vs. MSCKF logic)
2. Compare P_bg_trace and P_ba_trace evolution to bias convergence
3. Correlate rejection_reason patterns with drift onset
4. Extract subset for focused debugging at frame 180–182 boundary

## Files

- **Source**: `ros_ws/src/vio_pkg/test/test_hcmut_propagation_replay.py`
- **Output**: `frame_metrics_160_200.csv` (generated at runtime)
- **Test reference**: `ros_ws/frame_metrics_160_200.csv` (committed for validation)
