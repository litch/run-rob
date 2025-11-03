#!/bin/bash
# Setup CAN interface for RobStride motor

echo "Setting up CAN interface..."

# Kill any existing slcand processes
sudo pkill slcand || true

# Bring down can0 if it exists
sudo ip link set can0 down 2>/dev/null || true

DEV=$(readlink -f /dev/serial/by-id/*CANable2*)
echo "Starting slcand on $DEV ..."
sudo slcand -o -s8 -t hw "$DEV" can0

# Give it a moment to initialize
sleep 0.5

# Bring up the interface
echo "Bringing up can0..."
sudo ip link set can0 up

# Wait for interface to be ready
sleep 0.5

# Show interface details
echo ""
echo "CAN interface status:"
ip -details link show can0

echo ""
echo "CAN interface is ready!"
echo "You can now run: cargo run --release"
