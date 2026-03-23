#!/bin/bash

# Lane Tracking - Build and Run Script
# This script builds the project and runs it with a video from the data directory

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Lane Tracking Build and Run Script ===${NC}\n"

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Build the project
echo -e "${YELLOW}[1/3] Building project...${NC}"
if [ ! -d "build" ]; then
    echo "Creating build directory..."
    mkdir build
fi

cd build
cmake .. || { echo -e "${RED}CMake failed!${NC}"; exit 1; }
make || { echo -e "${RED}Build failed!${NC}"; exit 1; }
cd ..

echo -e "${GREEN}Build successful!${NC}\n"

# Find video file in data directory
echo -e "${YELLOW}[2/3] Looking for video files...${NC}"

VIDEO_FILE=""

# Check for command line argument first
if [ $# -gt 0 ]; then
    if [ -f "$1" ]; then
        VIDEO_FILE="$1"
        echo "Using provided video: $VIDEO_FILE"
    else
        echo -e "${RED}Error: Provided file '$1' not found!${NC}"
        exit 1
    fi
else
    # Look for video files in data directory
    for ext in webm mp4 avi mov mkv; do
        VIDEO_FILE=$(find data -maxdepth 1 -name "*.$ext" -type f | head -n 1)
        if [ -n "$VIDEO_FILE" ]; then
            echo "Found video: $VIDEO_FILE"
            break
        fi
    done
fi

# Run the program
echo -e "${YELLOW}[3/3] Running lane detection...${NC}\n"

if [ -n "$VIDEO_FILE" ]; then
    # Run with video file
    echo -e "${GREEN}Processing video: $VIDEO_FILE${NC}"
    echo -e "Output will be saved to: results/output.avi\n"
    ./build/lane_tracking_kalman --video="$VIDEO_FILE" --output="results/output.avi"
else
    # No video file found, use webcam
    echo -e "${YELLOW}No video file found in data directory.${NC}"
    echo -e "${GREEN}Using webcam (camera 0)${NC}\n"
    ./build/lane_tracking_kalman --camera=0
fi

echo -e "\n${GREEN}=== Done! ===${NC}"
