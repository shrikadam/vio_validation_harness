#!/bin/bash

# ==========================================
# Configuration Parameters
# ==========================================
BASE_DIR="$HOME/code/vio_validation_harness" 
SEQ=4
DATA_DIR="${BASE_DIR}/data/tumvi/bags"
RESULTS_DIR="${BASE_DIR}/results/openvins"

mkdir -p "${RESULTS_DIR}"

cleanup() {
    echo -e "\n[Script] Shutting down cleanly..."
    kill -INT $REC_PID $OV_PID 2>/dev/null
    wait $REC_PID $OV_PID 2>/dev/null
    echo "[Script] Cleanup complete."
}
trap cleanup EXIT

# ==========================================
# Execution
# ==========================================

echo "[Script] Starting estimator for sequence ${SEQ}..."
ros2 launch ov_msckf subscribe.launch.py config:=tum_vi &
OV_PID=$!
sleep 3

echo "[Script] Starting Python pose recorder..."
python3 "${BASE_DIR}/scripts/record_poses_openvins.py" "${RESULTS_DIR}/corridor${SEQ}.txt" &
REC_PID=$!
sleep 1

# Check if the python script failed
if ! kill -0 $REC_PID 2>/dev/null; then
    echo "[ERROR] Python recorder crashed on startup!"
    exit 1
fi

echo "[Script] Playing bag: dataset-corridor${SEQ}_512_16_ros2..."
ros2 bag play "${DATA_DIR}/dataset-corridor${SEQ}_512_16_ros2"

echo "[Script] Bag finished. Triggering cleanup..."