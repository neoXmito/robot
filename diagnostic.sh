#!/bin/bash
################################################################################
# Camera Diagnostic and Fix Script
# Troubleshoots and fixes common camera issues
################################################################################

echo "========================================"
echo "  CAMERA DIAGNOSTIC TOOL"
echo "========================================"
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ==============================================================================
# 1. CHECK VIDEO DEVICES
# ==============================================================================
echo -e "${BLUE}[1/7] Checking video devices...${NC}"

if ls /dev/video* >/dev/null 2>&1; then
    echo -e "${GREEN}✓ Video devices found:${NC}"
    ls -l /dev/video*
else
    echo -e "${YELLOW}⚠ No /dev/video* devices found${NC}"
    echo "This could mean:"
    echo "  1. Camera is not connected"
    echo "  2. Camera drivers not loaded"
    echo "  3. USB permissions issue"
fi
echo ""

# ==============================================================================
# 2. LIST USB DEVICES
# ==============================================================================
echo -e "${BLUE}[2/7] Checking USB cameras...${NC}"

if command -v lsusb &> /dev/null; then
    CAMERAS=$(lsusb | grep -i 'camera\|webcam\|video')
    if [ -n "$CAMERAS" ]; then
        echo -e "${GREEN}✓ USB cameras found:${NC}"
        echo "$CAMERAS"
    else
        echo -e "${YELLOW}⚠ No USB cameras detected in lsusb${NC}"
        echo "All USB devices:"
        lsusb
    fi
else
    echo -e "${YELLOW}lsusb not available${NC}"
fi
echo ""

# ==============================================================================
# 3. CHECK V4L2 UTILITIES
# ==============================================================================
echo -e "${BLUE}[3/7] Checking v4l-utils...${NC}"

if ! command -v v4l2-ctl &> /dev/null; then
    echo -e "${YELLOW}⚠ v4l-utils not installed${NC}"
    echo "Installing v4l-utils..."
    sudo apt-get update -qq
    sudo apt-get install -y v4l-utils
    echo -e "${GREEN}✓ v4l-utils installed${NC}"
else
    echo -e "${GREEN}✓ v4l-utils already installed${NC}"
fi

# List all video devices
if command -v v4l2-ctl &> /dev/null; then
    echo ""
    echo "Video4Linux devices:"
    v4l2-ctl --list-devices 2>/dev/null || echo "No devices found"
fi
echo ""

# ==============================================================================
# 4. CHECK KERNEL MODULES
# ==============================================================================
echo -e "${BLUE}[4/7] Checking kernel modules...${NC}"

REQUIRED_MODULES=("uvcvideo" "videodev" "v4l2_core")
for module in "${REQUIRED_MODULES[@]}"; do
    if lsmod | grep -q "^$module"; then
        echo -e "${GREEN}✓ Module $module loaded${NC}"
    else
        echo -e "${YELLOW}⚠ Module $module not loaded${NC}"
        echo "  Attempting to load..."
        sudo modprobe $module 2>/dev/null && echo -e "${GREEN}  ✓ Loaded${NC}" || echo -e "${RED}  ✗ Failed${NC}"
    fi
done
echo ""

# ==============================================================================
# 5. CHECK PERMISSIONS
# ==============================================================================
echo -e "${BLUE}[5/7] Checking permissions...${NC}"

if ls /dev/video* >/dev/null 2>&1; then
    for device in /dev/video*; do
        if [ -c "$device" ]; then
            perms=$(ls -l "$device" | awk '{print $1}')
            owner=$(ls -l "$device" | awk '{print $3":"$4}')
            echo "$device - $perms ($owner)"
            
            # Check if current user can access
            if [ -r "$device" ] && [ -w "$device" ]; then
                echo -e "  ${GREEN}✓ Current user has access${NC}"
            else
                echo -e "  ${YELLOW}⚠ Current user lacks access${NC}"
                echo "  Adding user to 'video' group..."
                sudo usermod -a -G video $USER
                echo -e "  ${YELLOW}Note: You may need to log out and back in${NC}"
            fi
        fi
    done
else
    echo -e "${YELLOW}No video devices to check permissions${NC}"
fi
echo ""

# ==============================================================================
# 6. TEST CAMERA WITH OPENCV
# ==============================================================================
echo -e "${BLUE}[6/7] Testing camera with OpenCV...${NC}"

python3 << 'EOF'
import cv2
import sys

print("Testing camera indices 0-4...")
working_cameras = []

for i in range(5):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            height, width = frame.shape[:2]
            print(f"  ✓ Camera {i}: Working ({width}x{height})")
            working_cameras.append(i)
        else:
            print(f"  ⚠ Camera {i}: Opens but can't read frames")
        cap.release()
    else:
        pass  # Camera doesn't exist, skip silently

if working_cameras:
    print(f"\n✓ Found {len(working_cameras)} working camera(s): {working_cameras}")
    print(f"  Recommended camera_index: {working_cameras[0]}")
    sys.exit(0)
else:
    print("\n✗ No working cameras found")
    sys.exit(1)
EOF

CAMERA_TEST_RESULT=$?
echo ""

# ==============================================================================
# 7. RECOMMENDATIONS
# ==============================================================================
echo -e "${BLUE}[7/7] Recommendations:${NC}"
echo ""

if [ $CAMERA_TEST_RESULT -eq 0 ]; then
    echo -e "${GREEN}✓ Camera is working!${NC}"
    echo ""
    echo "To use with your vision node, update the camera_index parameter:"
    echo "  In optimized_robot_vision.py, set:"
    echo "  camera_index = 0  # or the index shown above"
    echo ""
    echo "Or launch with parameter:"
    echo "  ros2 run your_package vision_node --ros-args -p camera_index:=0"
else
    echo -e "${RED}✗ Camera not working. Try these fixes:${NC}"
    echo ""
    echo "1. PHYSICAL CONNECTION:"
    echo "   - Reconnect USB camera"
    echo "   - Try a different USB port"
    echo "   - Check if camera has power LED (should be on)"
    echo ""
    echo "2. DRIVER ISSUES:"
    echo "   - Reconnect camera: unplug and plug back in"
    echo "   - Check dmesg: sudo dmesg | tail -20"
    echo "   - Reload modules:"
    echo "     sudo modprobe -r uvcvideo"
    echo "     sudo modprobe uvcvideo"
    echo ""
    echo "3. PERMISSIONS:"
    echo "   - Log out and back in (if added to video group)"
    echo "   - Or run: newgrp video"
    echo ""
    echo "4. VIRTUAL MACHINE:"
    echo "   - If using VM, ensure USB passthrough is enabled"
    echo "   - Check VM settings for camera/USB device"
    echo ""
    echo "5. RASPBERRY PI CSI CAMERA:"
    echo "   - Enable camera in raspi-config"
    echo "   - Use libcamera instead of v4l2"
    echo "   - Or use: raspistill -o test.jpg"
fi

echo ""
echo "========================================"
echo "  DIAGNOSTIC COMPLETE"
echo "========================================"
echo ""
echo "For more help, check:"
echo "  - dmesg | grep -i video"
echo "  - journalctl -xe | grep -i camera"
echo "  - lsusb -v (detailed USB info)"
echo ""