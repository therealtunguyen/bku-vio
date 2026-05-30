# Visual Frontend

The visual frontend is responsible for detecting and tracking image features across frames, then handing off mature feature tracks to the MSCKF backend for state updates.

---

## Architecture

```
vio_node.py (Thread 2: frontend_worker)
    │
    ▼
FeatureManager.process_image(timestamp, image, camera_pose)   ← Facade
    ├── IFeatureDetector.detect(image, mask)                  ← Strategy
    │       ShiTomasiDetector  (default)
    │       HarrisDetector     (alternative)
    │
    ├── IFeatureTracker.track(prev, curr, pts)                ← Strategy
    │       KLTTracker (pyramidal LK + forward-backward check)
    │
    └── returns List[FeatureTrack]  →  measurement_queue  →  MSCKF backend
```

**Design patterns used:**
- **Strategy** — detectors and trackers are swappable via `IFeatureDetector` / `IFeatureTracker` ABCs
- **Facade** — `FeatureManager.process_image()` is the single entry point; it hides the full detect-track-reject-manage pipeline
- **DTO** — `FrontendConfig` groups all tunable parameters; injected into every component

---

## Files

| File | Responsibility |
|------|----------------|
| `interfaces.py` | ABCs (`IFeatureDetector`, `IFeatureTracker`) + `FrontendConfig` dataclass |
| `detectors.py` | `ShiTomasiDetector`, `HarrisDetector` — both backed by `cv2.goodFeaturesToTrack` |
| `trackers.py` | `KLTTracker` — `cv2.calcOpticalFlowPyrLK` with forward-backward consistency check |
| `feature_manager.py` | `FeatureManager` — orchestrates the full per-frame pipeline |

---

## Per-frame Pipeline (FeatureManager)

```
Frame N arrives
│
├── First frame?
│     └── detect features across full image → create active tracks
│
└── Subsequent frame:
      1. Track all active features (KLT)
      2. Forward-backward consistency check  — rejects bad tracks
      3. RANSAC (findFundamentalMat)         — rejects geometric outliers  (needs ≥ 8 points)
      4. Update survived tracks (append observation + camera pose)
      5. Mature tracks that hit max_track_length, or lost tracks with ≥ min_track_length obs
      6. Detect new features in depleted 8×8 grid cells → add to active tracks
      └── return mature tracks to backend
```

---

## Configuration (`FrontendConfig`)

All parameters live in `interfaces.py` and are passed via constructor injection.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_features` | 200 | Target number of active tracks |
| `grid_rows` / `grid_cols` | 8 / 8 | Grid divisions for even feature distribution |
| `quality_level` | 0.01 | Shi-Tomasi / Harris corner quality threshold |
| `min_distance` | 10.0 px | Minimum distance between detected corners |
| `klt_win_size` | 21 | LK optical flow search window (pixels) |
| `klt_max_level` | 3 | Pyramid levels for LK |
| `fb_threshold` | 1.0 px | Max round-trip error for forward-backward check |
| `max_track_length` | 15 | Frames before a track is sent to the backend |
| `min_track_length` | 3 | Minimum observations for a lost track to be kept |
| `ransac_threshold` | 1.0 px | Reprojection threshold for fundamental matrix RANSAC |

To tune, edit `FrontendConfig()` in `vio_node.py`:

```python
self.frontend_config = FrontendConfig(max_features=150, max_track_length=10)
```

---

## Swapping the Detector or Tracker

The node currently uses `ShiTomasiDetector`. To switch to Harris, change two lines in `vio_node.py`:

```python
# from .frontend.detectors import ShiTomasiDetector  ← remove
from .frontend.detectors import HarrisDetector

self.detector = HarrisDetector(self.frontend_config)
```

To add a new detector (e.g. FAST), implement `IFeatureDetector` from `interfaces.py`:

```python
class FASTDetector(IFeatureDetector):
    def __init__(self, config: FrontendConfig): ...
    def detect(self, image, mask=None) -> List[np.ndarray]: ...
```

---

## Output: FeatureTrack

Mature tracks are `FeatureTrack` objects (defined in `utils/common.py`):

```python
@dataclass
class FeatureTrack:
    feature_id: int
    observations: List[np.ndarray]   # [u, v] pixel coords, one per frame
    camera_states: List[CameraPose]  # camera pose at each observation
```

`len(observations) == len(camera_states)` always holds.  
The MSCKF backend uses these for triangulation and measurement update.

---

## Verifying Your Work

There are two test scripts in `ros_ws/src/vio_pkg/test/`:

| Script | Images used | Requires ROS? |
|--------|-------------|---------------|
| `test_frontend.py` | Synthetic (generated) | No |
| `test_frontend_dataset.py` | Real EuRoC frames from the bag | Yes (container only) |

The teammate RGB-only bags under `dataset/ROSBAG_17_4_2026/` can be useful for
visual frontend, ArUco, and demo checks. They do not contain IMU data, so do not
use them as full VIO accuracy runs.

---

### Option A: Synthetic smoke test (no dataset needed)

```shell
docker exec -it vio-ros2 bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /home/ubuntu/VIO/ros_ws &&
  python3 src/vio_pkg/test/test_frontend.py
"
```

Or inside the container:

```shell
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws
python3 src/vio_pkg/test/test_frontend.py
```

**Expected output — all lines should show `[PASS]`:**

```
=======================================================
  VIO Frontend Smoke Tests
=======================================================

--- Test 1: ShiTomasiDetector ---
  [PASS] Returns a list
  [PASS] Detects > 0 points — 150 points
  [PASS] Respects max_features
  [PASS] Each point is np.ndarray shape (2,)
  [PASS] Mask respected (all x >= width/2)

--- Test 2: HarrisDetector ---
  [PASS] Returns a list
  [PASS] Detects > 0 points — 98 points
  [PASS] Respects max_features

--- Test 3: KLTTracker (identical frames — expect all tracked) ---
  [PASS] Returns same count as input
  [PASS] Status list matches
  [PASS] High track rate on identical frames (>80%) — 147/150 tracked

--- Test 4: KLTTracker (3px shifted frame) ---
  [PASS] Tracks survive a small shift (>50%) — 130/150 tracked

--- Test 5: FeatureManager (first frame) ---
  [PASS] No mature features on first frame
  [PASS] Active tracks initialised — 100 active
  [PASS] Respects max_features
  [PASS] Track IDs are unique

--- Test 6: FeatureManager (multi-frame lifecycle) ---
  [PASS] Active tracks non-empty after 8 frames — 80 active
  [PASS] Some tracks matured over 8 frames — 12 matured
  [PASS] Mature tracks have >= min_track_length observations
  [PASS] Mature tracks have matching camera_states
  [PASS] Feature IDs are globally unique across all matured tracks

--- Test 7: Grid-based feature distribution ---
  [PASS] Grid coverage >= 50% of cells — 14/16 cells occupied

=======================================================
  Done.
=======================================================
```

The exact numbers (point counts, matured tracks) will vary slightly by machine, but all `[PASS]` lines are required.

---

### Option B: Dataset integration test (real EuRoC frames)

Requires the bag at `/home/ubuntu/VIO/dataset/`.

```shell
docker exec -it vio-ros2 bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /home/ubuntu/VIO/ros_ws &&
  python3 src/vio_pkg/test/test_frontend_dataset.py
"
```

This reads 50 real frames from `/cam0/image_raw` (EuRoC V1_01_easy, 752×480) and runs:

- **Test 1** — Detector on real frames: average feature count, respects `max_features`
- **Test 2** — KLT tracking between consecutive frames: survival rate > 60%
- **Test 3** — Full `FeatureManager` pipeline over 50 frames: track lifecycle, ID uniqueness, obs/pose sync
- **Test 4** — Saves debug visualisation images to `/tmp/frontend_frame_00..02.png`

To inspect the visualisation images on your host machine:

```shell
docker cp vio-ros2:/tmp/frontend_frame_00.png .
docker cp vio-ros2:/tmp/frontend_frame_01.png .
docker cp vio-ros2:/tmp/frontend_frame_02.png .
```

Each image shows the real EuRoC frame with detected/tracked features overlaid as green dots.

#### Interpreting the visualisation frames

Each green dot is one `FeatureTrack` in `active_tracks` — it marks the latest observed pixel position `[u, v]` of that feature in that frame.

**Good output (frontend working correctly):**
- Dots are spread across the frame, not clustered in one region — confirms the 8×8 grid distribution is working
- Dots land on corners, edges, and textured surfaces — not on blank walls or uniform areas — confirms the detector is choosing trackable features
- Across `frame_00` → `frame_01` → `frame_02`, dots follow the same scene points as the camera moves slightly — confirms the tracker is holding on between frames

**Bad output (something is wrong):**

| Symptom | Likely cause |
|---------|-------------|
| All dots in one region | Grid occupancy mask not working |
| Dots on flat/featureless areas | `quality_level` too low |
| Very few dots overall | `max_features` too low, or image too dark |
| Dots jump wildly between frames | `fb_threshold` too loose, or frames are not sequential |

**What each frame tells you specifically:**

- `frame_00` — first frame, detect only (no tracking yet). Should show the most spatially uniform distribution.
- `frame_01` — first real tracking pass. Dots may shift slightly following camera motion. No RANSAC has run yet.
- `frame_02` — RANSAC has now run once. Expect slightly fewer dots as geometric outliers are dropped — this is normal and correct behaviour.

---

## Known Issues & Design Decisions

### IMU Buffer Pruning (`vio_node.py`)

**Problem statement**

After extracting IMU samples for a frame, the buffer must be pruned to prevent unbounded memory growth. A naive implementation might keep a fixed-duration tail (e.g. `timestamp > image_time - 0.1`) on the assumption that "some overlap is needed for the next window." This is wrong for two reasons:

1. **Dead data**: the slice `(image_time - 0.1, image_time]` was already consumed by the current frame's integration window. Keeping it wastes memory with no benefit.
2. **Misleading comment**: "keep a tail for the next window" implies those past samples will be re-used, but the next window needs future samples (timestamps > `image_time`) — not past ones.

**Why there is no data-loss risk from a slow camera**

Pruning only runs at frame-processing time. At that moment, IMU samples for the *next* frame haven't arrived yet (they have timestamps in the future). So regardless of camera frame rate — even 1 Hz — future IMU data can never be pruned by the current frame's cleanup step.

**Current fix**

```python
# Discard all consumed IMU data. Samples with timestamp >
# image_time have not arrived yet or just arrived and will
# be picked up by the next integration window.
self.imu_buffer = [m for m in self.imu_buffer if m.timestamp > image_time]
```

Prune exactly at `image_time`: keep only unconsumed future samples, discard everything that has already been extracted.
