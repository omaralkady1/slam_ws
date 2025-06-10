#!/bin/bash

# ESP32 Firmware Flash Script - Fixed Version
# This script helps you flash the optimized firmware to your ESP32

echo "=== ESP32 Firmware Flash Helper ==="
echo "This script will help you flash the optimized firmware to fix encoder issues"
echo ""

# Check if Arduino CLI is available
if ! command -v arduino-cli &> /dev/null; then
    echo "ERROR: arduino-cli not found. Please install Arduino CLI first:"
    echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
    echo "  sudo mv arduino-cli /usr/local/bin/"
    exit 1
fi

# Set the firmware path
FIRMWARE_PATH="/home/alkady/slam_ws/src/my_slam_pkg/firmware_reference/esp32_firmware.ino"

if [ ! -f "$FIRMWARE_PATH" ]; then
    echo "ERROR: Firmware file not found at $FIRMWARE_PATH"
    exit 1
fi

echo "Found firmware file: $FIRMWARE_PATH"
echo ""

# Detect ESP32 port
echo "Looking for ESP32 devices..."
ESP32_PORTS=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -5)

if [ -z "$ESP32_PORTS" ]; then
    echo "ERROR: No USB serial devices found. Please:"
    echo "  1. Connect your ESP32 via USB"
    echo "  2. Make sure the driver is installed"
    echo "  3. Check that the device appears in /dev/ttyUSB* or /dev/ttyACM*"
    exit 1
fi

echo "Available serial ports:"
select PORT in $ESP32_PORTS "Manual entry"; do
    if [ "$PORT" = "Manual entry" ]; then
        echo -n "Enter ESP32 port (e.g., /dev/ttyUSB0): "
        read PORT
    fi
    if [ -n "$PORT" ]; then
        ESP32_PORT="$PORT"
        break
    fi
done

echo "Selected ESP32 port: $ESP32_PORT"
echo ""

# Check if the ESP32 board is installed
echo "Checking ESP32 board configuration..."
if ! arduino-cli board listall | grep -q "esp32"; then
    echo "Installing ESP32 board support..."
    arduino-cli core update-index
    arduino-cli core install esp32:esp32
fi

# Install required libraries
echo "Installing required libraries..."
arduino-cli lib install "micro_ros_arduino"
# Note: You may need to manually install micro-ros-arduino library

echo ""
echo "=== FIRMWARE FIXES APPLIED ==="
echo "✓ Optimized encoder ISR functions for better performance"
echo "✓ Reduced control loop frequency from 100Hz to 50Hz"
echo "✓ Improved velocity filtering (less aggressive)"
echo "✓ Fixed direction correction for differential drive"
echo "✓ Increased motor deadband to reduce noise"
echo "✓ Reduced debug output frequency"
echo "✓ Increased watchdog timeout for stability"
echo ""

echo "Ready to compile and upload firmware to $ESP32_PORT"
echo -n "Proceed? (y/N): "
read PROCEED

if [[ "$PROCEED" =~ ^[Yy]$ ]]; then
    echo ""
    echo "Compiling and uploading firmware..."
    
    # Compile and upload
    arduino-cli compile --fqbn esp32:esp32:esp32 "$FIRMWARE_PATH"
    
    if [ $? -eq 0 ]; then
        echo "Compilation successful! Uploading..."
        arduino-cli upload -p "$ESP32_PORT" --fqbn esp32:esp32:esp32 "$FIRMWARE_PATH"
        
        if [ $? -eq 0 ]; then
            echo ""
            echo "✓ Firmware uploaded successfully!"
            echo ""
            echo "Next steps:"
            echo "1. Open the Arduino IDE Serial Monitor at 115200 baud"
            echo "2. Reset the ESP32 to see initialization messages"
            echo "3. Run the encoder test: ros2 run my_slam_pkg test_encoders.py"
            echo "4. Build and run your SLAM system: colcon build && ros2 launch my_slam_pkg esp32_slam_real.launch.py"
        else
            echo "ERROR: Upload failed. Check connections and try again."
        fi
    else
        echo "ERROR: Compilation failed. Check the firmware code."
    fi
else
    echo "Upload cancelled."
fi

echo ""
echo "=== TROUBLESHOOTING TIPS ==="
echo "If you still experience issues:"
echo "1. Check encoder wiring (A, B channels properly connected)"
echo "2. Verify motor power supply (adequate current)"
echo "3. Test individual motors with: ros2 run my_slam_pkg test_encoders.py"
echo "4. Monitor serial output for encoder count changes"
echo "5. Ensure micro-ROS agent is running: ros2 run micro_ros_agent micro_ros_agent serial --dev $ESP32_PORT"
