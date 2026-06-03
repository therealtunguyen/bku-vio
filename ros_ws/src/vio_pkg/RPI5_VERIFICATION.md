# Raspberry Pi 5 Verification Matrix

## Prerequisite Gate

Do not start this matrix until workstation `m6_report.json` has `ok=true`, the
ROS package builds, the regression suite passes, and the HCMUT onset report has
a classification plus exact next fix target.

## Environment Constraints

- Raspberry Pi 5, 64-bit OS
- ROS 2 Humble
- Python environment with `numpy<2`
- Repository checkout and datasets available locally
- Record CPU temperature before and after each runtime case

## Matrix

| Order | Check | Command | Pass condition |
|---|---|---|---|
| 1 | Platform facts | `python3 tools/collect_rpi5_facts.py results/rpi5/platform_facts.json` | JSON records `machine`, memory, temperature, NumPy, and OpenCV |
| 2 | Build | `cd ros_ws && colcon build --packages-select vio_pkg` | exit `0` |
| 3 | Regression suite | `cd ros_ws && python3 src/vio_pkg/test/test_backend_consistency.py && python3 src/vio_pkg/test/test_runtime_safeguards.py && python3 src/vio_pkg/test/test_frontend.py` | all pass |
| 4 | Replay diagnostics | `cd ros_ws && python3 src/vio_pkg/test/tools/compare_onset_windows.py --output ../results/rpi5/onset_report.json` | artifact written |
| 5 | HCMUT smoke | `python3 tools/run_m6_eval.py --case hcmut_d455_smoke --results-root results/rpi5/hcmut_smoke --skip-replay --allow-failures` | HCMUT case summary has `ok=true` |
| 6 | EuRoC diagnostic slice | `/usr/bin/time -v python3 tools/run_m6_eval.py --case euroc_v101_easy --results-root results/rpi5/euroc_slice --stop-after-processed-frames 300 --skip-evo --skip-replay --allow-failures` | runtime health records zero queue overflow and zero gap skip |
| 7 | EuRoC full run | `python3 tools/run_m6_eval.py --case euroc_v101_easy --results-root results/rpi5/euroc_full --skip-replay --allow-failures` | EuRoC case summary has `ok=true` |

## Evidence Record

For each row, retain stdout/stderr, `/usr/bin/time -v` output where applicable,
the before/after platform facts JSON, and generated case summaries. A
Raspberry Pi 5 deployment claim requires the completed matrix, not only a
successful build.
