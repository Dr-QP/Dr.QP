#!/bin/bash
set -e

DEVICE_MAC="D0:BC:C1:F4:F9:8B"   # your controller MAC

# Start pairing agent in the background
bt-agent --capability=NoInputNoOutput &
AGENT_PID=$!
sleep 2

# Power on & make pairable
bt-adapter --set Powered 1
bt-adapter --set Pairable 1

# 1️⃣ Launch discovery as a coprocess
coproc DISCOVERY { bt-adapter --discover; }
DISC_PID=$!

FOUND=0
# 2️⃣ Read discovery output line‑by‑line
while IFS= read -r -u "${DISCOVERY[0]}" line; do
  echo "$line"        # show progress
  if [[ "$line" == *"$DEVICE_MAC"* ]]; then
    FOUND=1
    break
  fi
done

# 3️⃣ Stop discovery
kill "$DISC_PID"

if [[ $FOUND -ne 1 ]]; then
  echo "❌ Device $DEVICE_MAC not found!"
  kill "$AGENT_PID"
  exit 1
fi
echo "✅ Found $DEVICE_MAC"

# 4️⃣ Connect (auto‑pairs if needed) and trust
echo "🔗 Connecting & pairing…"
bt-device --connect "$DEVICE_MAC"
bt-device --set "$DEVICE_MAC" Trusted 1

# 5️⃣ Show final status
bt-device --info "$DEVICE_MAC"

# Cleanup
kill "$AGENT_PID"
