#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -euo pipefail

# Default configuration
HOST=${XPRA_HOST:-127.0.0.1}
PORT=14500
DISPLAY=:100
BACKGROUND=false
PORT_EXPLICIT=false
VIDEO_ENCODERS=${XPRA_VIDEO_ENCODERS:-x264,vpx}
XPRA_DAEMON_MODE=no
XPRA_LOG_FILE="$PWD/log/xpra.log"

# Parse command-line arguments
print_usage() {
  cat <<'USAGE'
Usage: start-xpra.sh [--host HOST] [--port PORT] [--display DISPLAY] [--background] [--stop]

Start an Xpra server with the HTML5 web client enabled.

Options:
  --port PORT       HTTP port for the HTML5 client (default: derived from DEVCONTAINER_ID in range 14500-14599, or 14500 if DEVCONTAINER_ID is unset)
  --host HOST       Host/IP to bind the Xpra server (default: XPRA_HOST or 127.0.0.1)
  --display DISPLAY X display (accepts :100 or 100; default: :100)
  --background      Start Xpra in the background and exit immediately
  --stop            Stop the Xpra server and clean up resources

Environment:
  XPRA_HOST         Override the default bind host without editing this script
USAGE
}

normalize_display() {
  if [[ "$DISPLAY" != :* ]]; then
    DISPLAY=":$DISPLAY"
  fi
}

display_number() {
  echo "${DISPLAY#:}"
}

ensure_display_lock_is_clear() {
  local number lock_file socket_file lock_pid
  number="$(display_number)"
  lock_file="/tmp/.X${number}-lock"
  socket_file="/tmp/.X11-unix/X${number}"

  if [[ ! -f "$lock_file" ]]; then
    return 0
  fi

  lock_pid="$(tr -cd '0-9' < "$lock_file" 2>/dev/null || true)"
  if [[ -n "$lock_pid" ]] && kill -0 "$lock_pid" 2>/dev/null; then
    echo "Error: display $DISPLAY is in use by PID $lock_pid."
    echo "Use --display with a different number, or stop the existing X server."
    exit 1
  fi

  echo "Removing stale X lock for display $DISPLAY: $lock_file"
  rm -f "$lock_file"
  if [[ -S "$socket_file" ]]; then
    rm -f "$socket_file"
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
      PORT_EXPLICIT=true
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
      normalize_display
      cleanup
      ;;
    *)
      echo "Unknown option: $1"
      print_usage
      exit 1
      ;;
  esac
done

# Derive unique port per devcontainer instance when DEVCONTAINER_ID is set
if [[ "${PORT_EXPLICIT}" != "true" ]] && [[ -n "${DEVCONTAINER_ID:-}" ]]; then
  port_hash=$(echo -n "${DEVCONTAINER_ID}" | cksum | awk '{print $1}')
  PORT=$((14500 + (port_hash % 100)))
fi

normalize_display

if [[ "$BACKGROUND" == "true" ]]; then
  XPRA_DAEMON_MODE=yes
fi

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

ensure_display_lock_is_clear

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
  "$DISPLAY"                             # Virtual display identifier (for example, :100)
  "--bind-tcp=$HOST:$PORT"               # Bind HTML5/websocket server to host:port
  "--html=on"                            # Enable built-in HTML5 web client
  "--daemon=$XPRA_DAEMON_MODE"           # Xpra daemon mode (yes for background lifecycle hooks)
  "--mdns=no"                            # Disable mDNS service advertisement
  "--notifications=no"                   # Disable desktop notification forwarding
  "--global-menus=no"                    # Disable global menu integration
  "--start-new-commands=no"              # Prevent clients from launching new server-side commands
  "--dbus-proxy=no"                      # Disable D-Bus proxying into sessions
  "--dbus-control=no"                    # Disable remote control over D-Bus
  "--dbus-launch="                       # Do not auto-launch a D-Bus session bus
  "--webcam=no"                          # Disable webcam forwarding
  "--opengl=yes"                         # Allow OpenGL acceleration when available
  "--video-encoders=$VIDEO_ENCODERS"     # Preferred video encoder list (env-overridable)
)

# Start Xpra server
# --bind-tcp: Listen on specified host and port
# --html=on: Enable HTML5 web client
# --daemon=no: Run in foreground (important for Docker)
# Xpra creates its own virtual X server (Xvfb or Xdummy) internally
if [[ "$BACKGROUND" == "true" ]]; then
  echo "Starting Xpra in background mode..."
  if ! xpra start "${XPRA_ARGS[@]}" "--log-file=$XPRA_LOG_FILE"; then
    echo "Warning: Xpra failed to start in background mode. Check $XPRA_LOG_FILE."
    exit 1
  fi

  for _ in {1..10}; do
    if xpra_session_running; then
      exit 0
    fi
    sleep 1
  done

  if ! xpra_session_running; then
    echo "Warning: Xpra background startup did not produce a live session on $DISPLAY."
    echo "Check $XPRA_LOG_FILE for diagnostics."
    exit 1
  fi

  exit 0
fi

if ! xpra start "${XPRA_ARGS[@]}"; then
  echo "Warning: Xpra failed to start. Check ~/.xpra/logs/ for details."
  exit 1
fi
