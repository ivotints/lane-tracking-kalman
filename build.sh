#!/bin/bash

# Simple build script

set -e

echo "Building lane_tracking_kalman..."

if [ ! -d "build" ]; then
    mkdir build
fi

cd build
cmake ..
make -j$(nproc)

echo ""
echo "Build complete! Executable: build/lane_tracking_kalman"
echo ""
echo "Usage:"
echo "  ./build/lane_tracking_kalman --video=data/your_video.webm"
echo "  ./build/lane_tracking_kalman --camera=0"
echo ""
echo "Or use the convenience script:"
echo "  ./build_and_run.sh [optional_video_path]"
