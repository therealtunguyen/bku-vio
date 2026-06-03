# M6 Acceptance Contract

The machine-readable source of truth is `tools/m6_acceptance.json`.

## EuRoC Full Run

Accept only when the authoritative runner records at least 2800 odometry poses,
finishes without timeout or worker crash, records zero image-queue overflow
warnings, records zero large-forward-gap skips, and produces SE(3)-aligned
translation APE RMSE at or below 1.0 m without scale correction. Treat APE RMSE
above 0.25 m as a baseline-regression warning even when the milestone threshold
still passes.

## EuRoC Replay Harness

Replay is a determinism and diagnostics contract, not an accuracy benchmark.
Accept when at least 400 processed frames complete, discontinuity count is zero,
and snapshot segment replay matches the full replay.

## HCMUT D455 Smoke and Replay

HCMUT has no accepted metric ground truth. Accept smoke only when the runtime
finishes without timeout, worker crash, queue overflow, image timestamp gap,
frame-gap reset, or IMU-init reset, and observes at least one accepted MSCKF
update. Accept replay when at least 260 frames complete deterministically with
zero discontinuities and the onset report contains a non-empty classification.
Do not report HCMUT metric accuracy.

## Machine-Readable Report

`tools/run_m6_eval.py` is the authoritative entrypoint. A workstation M6 report
passes only when both case summaries, the replay comparison artifact, explicit
checks, and the overall `ok` field are present and green.

## Raspberry Pi 5 Gate

Start target-hardware verification only after the workstation overall report is
green, the HCMUT onset investigation has a classification and exact next
target, the ROS package builds, and the regression suite passes.
