#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -euo pipefail

# Default configuration
HOST=127.0.0.1
PORT=14500
DISPLAY=:100
BACKGROUND=false

# Parse command-line arguments
print_usage() {
  cat <<'USAGE'
Usage: start-xpra.sh [--port PORT] [--display DISPLAY] [--background]

Start an Xpra server with the HTML5 web client enabled.

Options:
  --port PORT       HTTP port for the HTML5 client (default: 14500)
  --host HOST       Host/IP to bind the Xpra server (default: 127.0.0.1)
  --display DISPLAY X display (accepts :100 or 100; default: :100)
  --background      Start Xpra in the background and exit immediately
  --stop            Stop the Xpra server and clean up resources
USAGE
}


normalize_display() {
  if [[ "$DISPLAY" != :* ]]; then
    DISPLAY=":$DISPLAY"
  fi
}

xpra_session_running() {
  xpra list 2>/dev/null | grep -Fq "LIVE session at $DISPLAY"
}

port_in_use() {
  if command -v lsof &>/dev/null; then
    lsof -iTCP:"$PORT" -sTCP:LISTEN -n -P &>/dev/null
    return $?
  fi
  if command -v fuser &>/dev/null; then
    fuser "$PORT"/tcp &>/dev/null
    return $?
  fi
  return 1
}

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
    if xpra_session_running; then
      xpra stop "$DISPLAY" 2>/dev/null || true
      sleep 1
      if xpra_session_running; then
        xpra exit "$DISPLAY" 2>/dev/null || true
      fi
    else
      echo "Xpra server on $DISPLAY is not running; nothing to stop."
    fi
  fi

  if port_in_use; then
    echo "Warning: TCP port $PORT is still in use after cleanup."
  fi

  exit 0
}


while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      if [[ $# -lt 2 ]]; then
        echo "Error: --host requires a value."
        print_usage
        exit 1
      fi
      HOST="$2"
      shift 2
      ;;
    --port)
      if [[ $# -lt 2 ]]; then
        echo "Error: --port requires a value."
        print_usage
        exit 1
      fi
      PORT="$2"
      shift 2
      ;;
    --display)
      if [[ $# -lt 2 ]]; then
        echo "Error: --display requires a value."
        print_usage
        exit 1
      fi
      DISPLAY="$2"
      shift 2
      ;;
    --background)
      BACKGROUND=true
      shift
      ;;
    --stop)
      cleanup
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      print_usage
      exit 1
      ;;
  esac
done

normalize_display

# Setup signal handlers for graceful shutdown
trap cleanup SIGTERM SIGINT

# Main startup logic
echo "=========================================="
echo "Xpra Server Startup"
echo "=========================================="
echo "Port: $PORT"
echo "Display: $DISPLAY"

if ! command -v xpra &> /dev/null; then
  echo "Warning: xpra not found. Skipping startup."
  exit 0
fi

if xpra_session_running; then
  echo "Xpra already running on $DISPLAY. Skipping startup."
  exit 0
fi

# Detect and setup rendering
if detect_gpu; then
  setup_gpu_rendering || true
fi

# Export DISPLAY for child processes
export DISPLAY

echo ""
echo "Starting Xpra server..."
echo "HTML5 client will be available at: http://$HOST:$PORT"
echo ""

# Shared Xpra options (defined once for both foreground/background modes)
XPRA_ARGS=(
  "$DISPLAY"
  "--bind-tcp=$HOST:$PORT"
  "--html=on"
  "--daemon=no"
  "--mdns=no"
  "--notifications=no"
  "--global-menus=no"
  "--start-new-commands=no"
  "--dbus-proxy=no"
  "--dbus-control=no"
  "--dbus-launch="
  "--webcam=no"
  "--opengl=yes"
)

# Start Xpra server
# --bind-tcp: Listen on specified host and port
# --html=on: Enable HTML5 web client
# --daemon=no: Run in foreground (important for Docker)
# Xpra creates its own virtual X server (Xvfb or Xdummy) internally
if [[ "$BACKGROUND" == "true" ]]; then
  nohup xpra start "${XPRA_ARGS[@]}" > /tmp/xpra.log 2>&1 &
  exit 0
fi

if ! xpra start "${XPRA_ARGS[@]}"; then
  echo "Warning: Xpra failed to start. Check ~/.xpra/logs/ for details."
  exit 1
fi
