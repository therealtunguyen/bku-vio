#!/usr/bin/env bash
# Orchestrate the EuRoC V1_01_easy baseline run inside the vio-ros2 container.
# Not part of the team workflow — used only for the bootstrap run that fills
# RESULTS.md baseline. Team should use the commands in docs/results/RESULTS.md §3.A.
set -eo pipefail

# ROS setup scripts reference unset vars; do not enable nounset here.
source /opt/ros/humble/setup.bash
cd /home/ubuntu/ros_ws
source install/setup.bash

RUN_TAG=euroc_V1_01_easy
RUN_DIR=/home/ubuntu/results/${RUN_TAG}
DATASET_DIR=/home/ubuntu/dataset
BAG=${DATASET_DIR}/V1_01_easy
GT_CSV=${DATASET_DIR}/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv
BAG_RATE=0.2

rm -rf "${RUN_DIR}"
# DO NOT pre-create ${RUN_DIR}/bag — rosbag2 record refuses to write into an existing folder.
mkdir -p "${RUN_DIR}/eval" "${RUN_DIR}/plots"

cleanup() {
    echo "[run] cleanup: killing any lingering node/recorder/play processes..."
    pkill -INT -f vio_system_node 2>/dev/null || true
    pkill -INT -f "ros2 bag record" 2>/dev/null || true
    pkill -INT -f "ros2 bag play" 2>/dev/null || true
    sleep 3
    pkill -9 -f vio_system_node 2>/dev/null || true
    pkill -9 -f "ros2 bag record" 2>/dev/null || true
    pkill -9 -f "ros2 bag play" 2>/dev/null || true
}
trap cleanup EXIT

echo "[run] starting vio_system_node..."
ros2 run vio_pkg vio_system_node --ros-args \
    -p use_sim_time:=true \
    -p gt_csv_path:="${GT_CSV}" \
    -p imu_init_sample_count:=200 \
    >"${RUN_DIR}/node.log" 2>&1 &
NODE_PID=$!
echo "[run] vio_system_node bg PID=${NODE_PID}"

sleep 6  # let node create publishers

echo "[run] starting bag recorder..."
ros2 bag record --output "${RUN_DIR}/bag" --storage sqlite3 \
    /vio/odometry /vio/gt_path \
    >"${RUN_DIR}/record.log" 2>&1 &
REC_PID=$!
echo "[run] bag recorder bg PID=${REC_PID}"

sleep 4  # let recorder subscribe

echo "[run] playing bag at rate ${BAG_RATE} (foreground, ~$(awk "BEGIN{print int(145.6/${BAG_RATE})}")s)..."
ros2 bag play "${BAG}" --clock --rate "${BAG_RATE}" \
    >"${RUN_DIR}/play.log" 2>&1
echo "[run] bag play finished."

# Let the node and recorder drain.
sleep 6

echo "[run] stopping recorder and node (SIGINT, then SIGKILL after 4s)..."
pkill -INT -f "ros2 bag record" 2>/dev/null || true
pkill -INT -f vio_system_node 2>/dev/null || true
sleep 4
pkill -9 -f "ros2 bag record" 2>/dev/null || true
pkill -9 -f vio_system_node 2>/dev/null || true

echo "[run] recorded bag contents:"
python3 - <<EOF
import sqlite3, glob, sys
dbs = sorted(glob.glob("${RUN_DIR}/bag/*.db3"))
if not dbs:
    print("NO .db3 FILES — record produced nothing"); sys.exit(2)
for db in dbs:
    with sqlite3.connect(db) as c:
        for name, n in c.execute("select t.name, count(*) from messages m join topics t on m.topic_id=t.id group by t.name"):
            print(f"  {name:30s} {n}")
EOF

echo "[run] running evaluate_m4.py..."
python3 /home/ubuntu/tools/evaluate_m4.py "${RUN_DIR}/bag" "${RUN_DIR}/eval"

echo "[run] running plot_trajectory.py..."
python3 /home/ubuntu/tools/plot_trajectory.py \
    --gt   "${RUN_DIR}/eval/vio_gt_path.tum" \
    --vio  "${RUN_DIR}/eval/vio_odom.tum" \
    --out  "${RUN_DIR}/plots/trajectory_xy.png" \
    --title "EuRoC V1_01_easy - BKU-VIO baseline"

echo "[run] RMSE summary:"
evo_res "${RUN_DIR}/eval/ape_aligned.zip" --no_warnings | grep -E "rmse|mean|median|std" || true
echo "---"
evo_res "${RUN_DIR}/eval/rpe_1m_aligned.zip" --no_warnings | grep -E "rmse|mean|median|std" || true

# Persist commit hash for reproducibility.
git -C /home/ubuntu/ros_ws/.. rev-parse HEAD 2>/dev/null > "${RUN_DIR}/commit.txt" || echo "n/a (not a git checkout in container)" > "${RUN_DIR}/commit.txt"

touch "${RUN_DIR}/.done"
echo "[run] DONE."
