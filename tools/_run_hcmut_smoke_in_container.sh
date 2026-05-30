#!/usr/bin/env bash
# Smoke test for HCMUT vio_hcmut_dataset inside vio-ros2 container.
# Purpose: verify the pipeline runs end-to-end on D455 input.
# NOT for accuracy — no GT, calibration not validated. See docs/results/hcmut_smoke.md.
set -eo pipefail

source /opt/ros/humble/setup.bash
cd /home/ubuntu/ros_ws
source install/setup.bash

RUN_TAG=hcmut_smoke
RUN_DIR=/home/ubuntu/results/${RUN_TAG}
DATASET_DIR=/home/ubuntu/dataset
BAG=${DATASET_DIR}/vio_hcmut_dataset

rm -rf "${RUN_DIR}"
mkdir -p "${RUN_DIR}"

cleanup() {
    pkill -INT -f vio_system_node 2>/dev/null || true
    pkill -INT -f "ros2 bag record" 2>/dev/null || true
    pkill -INT -f "ros2 bag play" 2>/dev/null || true
    sleep 3
    pkill -9 -f vio_system_node 2>/dev/null || true
    pkill -9 -f "ros2 bag record" 2>/dev/null || true
    pkill -9 -f "ros2 bag play" 2>/dev/null || true
}
trap cleanup EXIT

echo "[smoke] starting vio_system_node with D455 intrinsics, QoS=best_effort..."
START_NS=$(date +%s%N)
ros2 run vio_pkg vio_system_node --ros-args \
    -p use_sim_time:=true \
    -p input_qos_reliability:=best_effort \
    -p camera_fx:=646.33728 \
    -p camera_fy:=645.676147 \
    -p camera_cx:=643.358276 \
    -p camera_cy:=362.999176 \
    -p camera_distortion:="[-0.05594548583030701, 0.06458555161952972, -0.0002526374883018434, 0.0008183500613085926, -0.021141313016414642]" \
    >"${RUN_DIR}/node.log" 2>&1 &
NODE_PID=$!

sleep 5

echo "[smoke] starting bag recorder (/vio/odometry only)..."
ros2 bag record --output "${RUN_DIR}/bag" --storage sqlite3 \
    /vio/odometry \
    >"${RUN_DIR}/record.log" 2>&1 &
REC_PID=$!

sleep 3

echo "[smoke] playing HCMUT bag with topic remap, rate 1.0 (~54s)..."
ros2 bag play "${BAG}" --clock --rate 1.0 \
    --remap /camera/camera/color/image_raw:=/cam0/image_raw \
            /camera/camera/imu:=/imu0 \
    >"${RUN_DIR}/play.log" 2>&1
echo "[smoke] bag play finished."

sleep 4

pkill -INT -f "ros2 bag record" 2>/dev/null || true
pkill -INT -f vio_system_node 2>/dev/null || true
sleep 3
pkill -9 -f "ros2 bag record" 2>/dev/null || true
pkill -9 -f vio_system_node 2>/dev/null || true

END_NS=$(date +%s%N)
WALL_S=$(awk "BEGIN{printf \"%.1f\", ($END_NS - $START_NS) / 1e9}")
echo "[smoke] total wall clock: ${WALL_S}s"

echo "[smoke] recorded /vio/odometry messages + position trajectory summary:"
python3 - <<EOF
import sqlite3, glob, sys, struct
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
dbs = sorted(glob.glob("${RUN_DIR}/bag/*.db3"))
if not dbs:
    print("  NO .db3 — record produced nothing"); sys.exit(2)

Odom = get_message("nav_msgs/msg/Odometry")
poses = []
for db in dbs:
    with sqlite3.connect(db) as c:
        for (name, n) in c.execute("select t.name, count(*) from messages m join topics t on m.topic_id=t.id group by t.name"):
            print(f"  topic {name}: {n} messages")
        for (data,) in c.execute(
            "select data from messages m join topics t on m.topic_id=t.id where t.name='/vio/odometry' order by timestamp"
        ):
            msg = deserialize_message(data, Odom)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.pose.position
            poses.append((t, p.x, p.y, p.z))

if not poses:
    print("  no /vio/odometry poses to summarize")
    sys.exit(0)

t0 = poses[0][0]
t_last = poses[-1][0]
duration = t_last - t0
print(f"  /vio/odometry duration (sim time): {duration:.2f} s")
print(f"  first pose: t={poses[0][0]:.3f} pos=({poses[0][1]:+.3f}, {poses[0][2]:+.3f}, {poses[0][3]:+.3f})")
print(f"  last  pose: t={poses[-1][0]:.3f} pos=({poses[-1][1]:+.3f}, {poses[-1][2]:+.3f}, {poses[-1][3]:+.3f})")

# Track max distance from origin to flag divergence.
import math
dmax = 0.0; tmax = 0.0
for (t, x, y, z) in poses:
    d = math.sqrt(x*x + y*y + z*z)
    if d > dmax:
        dmax, tmax = d, t
print(f"  max ||pos|| from origin: {dmax:.3f} m  @ sim t={tmax:.3f}")

# Per-axis range.
xs = [p[1] for p in poses]
ys = [p[2] for p in poses]
zs = [p[3] for p in poses]
print(f"  x range: [{min(xs):+.3f}, {max(xs):+.3f}]  ({max(xs)-min(xs):.3f} m)")
print(f"  y range: [{min(ys):+.3f}, {max(ys):+.3f}]  ({max(ys)-min(ys):.3f} m)")
print(f"  z range: [{min(zs):+.3f}, {max(zs):+.3f}]  ({max(zs)-min(zs):.3f} m)")

# Quick divergence heuristic: time at which ||pos|| first exceeds 10 m (a real handheld dataset rarely does in 54s).
DIVERGE_THRESH = 10.0
for (t, x, y, z) in poses:
    d = math.sqrt(x*x + y*y + z*z)
    if d > DIVERGE_THRESH:
        print(f"  FLAG: ||pos|| > {DIVERGE_THRESH} m at sim t={t-t0:.2f}s after start (likely divergence)")
        break
else:
    print(f"  no pose exceeded {DIVERGE_THRESH} m from origin (no obvious blow-up)")
EOF

echo "[smoke] node.log tail (last 8 lines):"
tail -n 8 "${RUN_DIR}/node.log" || true

touch "${RUN_DIR}/.done"
echo "[smoke] DONE."
