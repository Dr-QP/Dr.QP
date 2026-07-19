# 01 — Constrain slow Gazebo CI and retries

## Goal

Make a deterministic behavior failure fail promptly and preserve useful logs, without allowing a
single launch-test executable to consume the complete slow-job budget.

## Dependencies

None. Ship this before changing balance behavior.

## TDD red

1. Add a collection-policy test proving an ordinary `@pytest.mark.launch` item receives no retry
   marker.
2. Prove the small set of tests with an evidenced shutdown crash keeps an explicit retry marker.
3. Add a configuration test proving the slow matrix supplies an explicit, bounded CTest parallel
   level rather than whitespace-triggered automatic parallelism.
4. Add a CMake/configuration test proving every Gazebo test executable has a finite outer wall
   timeout comfortably below the 60-minute job timeout.

## Implementation

- Remove the collection hook that adds `flaky(retries=3)` to every launch test.
- Keep retries only on individually annotated tests whose CI history shows the known shutdown
  crash. Do not retry motion, pose, reachability, or leveling assertions.
- Give the slow simulation matrix an explicit concurrency limit. Start with serial Gazebo tests;
  increase to two only after several clean CI runs show stable physics and lower total time.
- Restore a finite CTest timeout for each launch-test executable. The timeout must include normal
  low-real-time-factor operation but remain below the job budget. Pytest's per-item timeout is a
  secondary stack-dump watchdog, not the only suite deadline.
- Ensure canceled or timed-out launch tests upload their captured output and xUnit files when
  available.

## Acceptance criteria

- A deliberate motion assertion failure runs once and produces one failure report.
- No `drqp_gazebo` test executable can run until the GitHub job cancels it.
- The slow suite completes within 35 minutes in three consecutive amd64 CI runs.
- Process-exit assertions and allowlists remain unchanged.

## Validation

```text
DRQP_TEST_MODE=slow scripts/with-ros-env.sh python3 -m colcon test \
  --packages-select drqp_gazebo --return-code-on-test-failure
```
