#!/bin/bash
set -e

# Default configuration values based on your workspace
BASE_DIR="/home/shri/code/vio_validation_harness"
SEQ="dataset-corridor4_512_16"
DATASET_TYPE=0      # 0 for EuRoC-format dataset
USE_LCD=0           # 0 to disable Loop Closure Detector, 1 to enable
LOG_OUTPUT=1        # 1 to enable logging of output files

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    -b|--base_dir)
      BASE_DIR="$2"
      shift 2
      ;;
    -seq|--seq)
      SEQ="$2"
      shift 2
      ;;
    -lcd)
      USE_LCD=1
      shift
      ;;
    -nolog)
      LOG_OUTPUT=0
      shift
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

# Define paths matching your exact folder layout (accessible on host and mounted inside container)
DATASET_PATH="$BASE_DIR/data/tumvi/euroc/$SEQ"
OUTPUT_PATH="$BASE_DIR/results/$SEQ"
PARAMS_PATH="$BASE_DIR/kimera_ws/Kimera-VIO/params/TumVI"
VOCABULARY_PATH="$BASE_DIR/kimera_ws/Kimera-VIO/vocabulary/ORBvoc.yml"

# Binary path is located inside the container image
BINARY_PATH="~/Kimera-VIO/build/stereoVIOEuroc"

# ===================================================================
# Pre-flight Validation: Check for mandatory flags folder in TumVI params
# ===================================================================
if [ ! -d "$PARAMS_PATH/flags" ]; then
    echo "================================================================"
    echo "ERROR: Missing 'flags' directory inside $PARAMS_PATH!"
    echo "================================================================"
    echo "Kimera's stereoVIOEuroc binary looks for flagfiles via:"
    echo "  --flagfile=\$PARAMS_PATH/flags/*.flags"
    echo "regardless of the sensor parameter set being used."
    echo ""
    echo "To fix this, copy the flags folder from your Euroc params:"
    echo "  cp -r $BASE_DIR/kimera_ws/Kimera-VIO/params/Euroc/flags $PARAMS_PATH/"
    echo "================================================================"
    exit 1
fi

# Create output directory on host if it doesn't exist
mkdir -p "$OUTPUT_PATH"

# Allow X server connection for visualization
xhost +local:root

echo """ Launching Kimera-VIO Inference:
            Base Directory:   $BASE_DIR
            Dataset Sequence: $SEQ
            Dataset Path:     $DATASET_PATH
            Params Path:      $PARAMS_PATH
            Output Path:      $OUTPUT_PATH
 """

# Run inside Docker container, mounting base_dir so absolute paths match
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$BASE_DIR:$BASE_DIR" \
    kimera_vio \
    bash -c "$BINARY_PATH \
      --dataset_type=$DATASET_TYPE \
      --dataset_path=$DATASET_PATH \
      --initial_k=50 \
      --final_k=10000 \
      --params_folder_path=$PARAMS_PATH \
      --use_lcd=$USE_LCD \
      --vocabulary_path=$VOCABULARY_PATH \
      --flagfile=$PARAMS_PATH/flags/stereoVIOEuroc.flags \
      --flagfile=$PARAMS_PATH/flags/Mesher.flags \
      --flagfile=$PARAMS_PATH/flags/VioBackend.flags \
      --flagfile=$PARAMS_PATH/flags/RegularVioBackend.flags \
      --flagfile=$PARAMS_PATH/flags/Visualizer3D.flags \
      --logtostderr=1 \
      --colorlogtostderr=1 \
      --log_prefix=1 \
      --v=0 \
      --vmodule=Pipeline*=00 \
      --log_output=$LOG_OUTPUT \
      --log_euroc_gt_data=$LOG_OUTPUT \
      --save_frontend_images=1 \
      --visualize_frontend_images=1 \
      --output_path=$OUTPUT_PATH"

# Disallow X server connection
xhost -local:root