# HapticEffect.msg - SDL3 Haptic Effect Command
#
# Maps the SDL3 haptic API (SDL3/SDL_haptic.h) to ROS 2 messages.
# Publish to joy/set_haptic to trigger haptic effects on the gamepad.

# Effect type constants — map to SDL_HAPTIC_* types
uint8 TYPE_LEFTRIGHT = 0      # SDL_HAPTIC_LEFTRIGHT  - Dual motor rumble
uint8 TYPE_CONSTANT = 1       # SDL_HAPTIC_CONSTANT   - Constant force
uint8 TYPE_SINE = 2           # SDL_HAPTIC_SINE       - Sine wave
uint8 TYPE_TRIANGLE = 3       # SDL_HAPTIC_TRIANGLE   - Triangle wave
uint8 TYPE_SAWTOOTHUP = 4     # SDL_HAPTIC_SAWTOOTHUP
uint8 TYPE_SAWTOOTHDOWN = 5   # SDL_HAPTIC_SAWTOOTHDOWN
uint8 TYPE_RAMP = 6           # SDL_HAPTIC_RAMP       - Ramp force

# Action constants
uint8 ACTION_PLAY = 0     # Create and run the effect
uint8 ACTION_STOP = 1     # Stop the currently active effect
uint8 ACTION_STOP_ALL = 2 # Stop all active effects

uint8 action       # ACTION_* - what to do with the effect
uint8 effect_type  # TYPE_*   - type of haptic effect (used with ACTION_PLAY)

# Timing parameters (used for most effect types)
# Set duration_ms to 0 to play indefinitely (SDL_HAPTIC_INFINITY)
uint32 duration_ms    # Effect duration in ms (0 = until stopped)
uint32 delay_ms       # Delay before effect starts (ms)
uint32 attack_ms      # Fade-in duration (ms)
uint32 fade_ms        # Fade-out duration (ms)
uint32 iterations     # Play count (0 = loop forever)

# ---- TYPE_LEFTRIGHT fields ----
# Controls the left (large/heavy) and right (small/light) rumble motors
float32 large_magnitude   # Left/heavy motor strength [0.0 to 1.0]
float32 small_magnitude   # Right/light motor strength [0.0 to 1.0]

# ---- TYPE_CONSTANT fields ----
float32 level    # Constant force level [-1.0 to 1.0]

# ---- TYPE_SINE / TYPE_TRIANGLE / TYPE_SAWTOOTHUP / TYPE_SAWTOOTHDOWN fields ----
uint32 period_ms        # Wave period in ms
float32 magnitude       # Peak amplitude [0.0 to 1.0]
float32 offset          # Mean / centre value [-1.0 to 1.0]
float32 phase_degrees   # Phase offset [0.0 to 360.0]

# ---- TYPE_RAMP fields ----
float32 ramp_start  # Starting force [-1.0 to 1.0]
float32 ramp_end    # Ending force   [-1.0 to 1.0]
