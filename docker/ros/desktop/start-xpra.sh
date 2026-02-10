#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -euo pipefail

# Xpra startup script for Docker containers
# Starts Xpra server with HTML5 web client support
# Supports GPU acceleration via VirtualGL or software rendering via Xvfb

# Default configuration
PORT=14500
DISPLAY=:100

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
    echo "Warning: VirtualGL not found, GPU acceleration will not be available"
    return 1
  fi

  # Set VGL_DISPLAY to use the GPU
  export VGL_DISPLAY=:0
  echo "GPU detected. Use 'vglrun <app>' for GPU-accelerated rendering (e.g., 'vglrun rviz2')"
  return 0
}

# Cleanup function for graceful shutdown
cleanup() {
  echo "Shutting down Xpra server..."

  # Stop Xpra server
  if command -v xpra &> /dev/null; then
    xpra stop "$DISPLAY" 2>/dev/null || true
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
  setup_gpu_rendering || true
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
# Xpra creates its own virtual X server (Xvfb or Xdummy) internally
xpra start "$DISPLAY" \
  --bind-tcp=0.0.0.0:"$PORT" \
  --html=on \
  --daemon=no

# Cleanup on exit
cleanup

