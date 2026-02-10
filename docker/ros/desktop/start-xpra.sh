#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -euo pipefail

# Xpra startup script for Docker containers
# Starts Xpra server with HTML5 web client support
# Supports GPU acceleration via VirtualGL or software rendering via Xvfb

# Default configuration
PORT=14500
DISPLAY=:100
USE_GPU=false
XVFB_PID=""

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      PORT="$2"
      shift 2
      ;;
    --display)
      DISPLAY="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [--port PORT] [--display DISPLAY]"
      exit 1
      ;;
  esac
done

# Detect GPU availability
detect_gpu() {
  # Check for NVIDIA GPU
  if command -v nvidia-smi &> /dev/null; then
    if nvidia-smi &> /dev/null; then
      echo "NVIDIA GPU detected"
      return 0
    fi
  fi
  
  # Check for DRI devices (AMD/Intel)
  if [[ -d /dev/dri ]] && [[ -n "$(ls -A /dev/dri 2>/dev/null)" ]]; then
    echo "DRI GPU device detected"
    return 0
  fi
  
  echo "No GPU detected, using software rendering"
  return 1
}

# Setup GPU rendering with VirtualGL
setup_gpu_rendering() {
  if ! command -v vglrun &> /dev/null; then
    echo "Warning: VirtualGL not found, falling back to software rendering"
    return 1
  fi
  
  # Set VGL_DISPLAY to use the GPU
  export VGL_DISPLAY=:0
  USE_GPU=true
  echo "GPU rendering enabled with VirtualGL"
  return 0
}

# Setup software rendering with Xvfb
setup_software_rendering() {
  if ! command -v Xvfb &> /dev/null; then
    echo "Error: Xvfb not found. Install xvfb package."
    exit 1
  fi
  
  echo "Starting Xvfb virtual framebuffer on $DISPLAY..."
  
  # Start Xvfb with Mesa llvmpipe software rendering
  # -screen: 1920x1080 resolution, 24-bit color
  # -ac: disable access control
  Xvfb "$DISPLAY" -screen 0 1920x1080x24 -ac &
  XVFB_PID=$!
  
  # Give Xvfb time to start
  sleep 1
  
  echo "Xvfb started with PID $XVFB_PID"
}

# Cleanup function for graceful shutdown
cleanup() {
  echo "Shutting down Xpra server..."
  
  # Stop Xpra server
  if command -v xpra &> /dev/null; then
    xpra stop "$DISPLAY" 2>/dev/null || true
  fi
  
  # Stop Xvfb if it was started
  if [[ -n "$XVFB_PID" ]] && kill -0 "$XVFB_PID" 2>/dev/null; then
    echo "Stopping Xvfb (PID $XVFB_PID)..."
    kill "$XVFB_PID" 2>/dev/null || true
    wait "$XVFB_PID" 2>/dev/null || true
  fi
  
  exit 0
}

# Setup signal handlers for graceful shutdown
trap cleanup SIGTERM SIGINT

# Main startup logic
echo "=========================================="
echo "Xpra Server Startup"
echo "=========================================="
echo "Port: $PORT"
echo "Display: $DISPLAY"

# Detect and setup rendering
if detect_gpu; then
  if ! setup_gpu_rendering; then
    setup_software_rendering
  fi
else
  setup_software_rendering
fi

# Export DISPLAY for child processes
export DISPLAY

echo ""
echo "Starting Xpra server..."
echo "HTML5 client will be available at: http://localhost:$PORT"
echo ""

# Start Xpra server
# --bind-tcp: Listen on all interfaces on specified port
# --html=on: Enable HTML5 web client
# --daemon=no: Run in foreground (important for Docker)
# --exit-with-children: Exit when all child processes exit
if [[ "$USE_GPU" == "true" ]]; then
  # Use VirtualGL wrapper for GPU rendering
  vglrun xpra start "$DISPLAY" \
    --bind-tcp=0.0.0.0:"$PORT" \
    --html=on \
    --daemon=no \
    --exit-with-children
else
  # Use standard Xpra for software rendering
  xpra start "$DISPLAY" \
    --bind-tcp=0.0.0.0:"$PORT" \
    --html=on \
    --daemon=no \
    --exit-with-children
fi

# Cleanup on exit
cleanup

