#!/bin/bash
# setup_gamepad.sh - Setup script for gamepad control

echo "Setting up Logitech F310 Gamepad for ROS2..."

# Install required system packages
sudo apt update
sudo apt install -y \
  ros-jazzy-joy \
  ros-jazzy-teleop-twist-joy \
  python3-pygame \
  joystick \
  jstest-gtk

# Set up udev rules for gamepad access without sudo
sudo tee /etc/udev/rules.d/99-gamepad.rules >/dev/null <<'EOF'
# Logitech F310 Gamepad
SUBSYSTEM=="input", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c21d", MODE="0666", GROUP="input"
# Generic joystick access
SUBSYSTEM=="input", KERNEL=="js[0-9]*", MODE="0666", GROUP="input"
EOF

# Add user to input group
sudo usermod -a -G input $USER

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Gamepad setup complete!"
echo ""
echo "Testing gamepad connection:"

# Test if gamepad is detected
if ls /dev/input/js* >/dev/null 2>&1; then
  echo "✓ Gamepad device found: $(ls /dev/input/js*)"

  # Test basic functionality
  echo "Testing gamepad input (press Ctrl+C to stop):"
  echo "Move sticks and press buttons to see output..."
  timeout 5s jstest /dev/input/js0 || echo "Test completed or no input detected"
else
  echo "✗ No gamepad detected. Please:"
  echo "  1. Connect your Logitech F310 controller"
  echo "  2. Make sure it's in DirectInput mode (switch on back set to 'D')"
  echo "  3. Log out and back in (or reboot) to apply group changes"
fi

echo ""
echo "Manual testing commands:"
echo "  jstest /dev/input/js0          # Test raw input"
echo "  jstest-gtk                     # GUI testing tool"
echo "  ros2 run joy joy_node          # Test ROS2 joy node"
echo "  ros2 topic echo /joy           # Monitor joy messages"
