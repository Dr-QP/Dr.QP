# T3-B · Replace erase-from-front with a position index in `SerialPlayer`

**Severity:** Low (micro-optimisation)
**Agent turns:** 2
**File:** `packages/runtime/drqp_serial/src/SerialPlayer.cpp:70`

## Background

After verifying each written byte sequence, `SerialPlayer` erases the consumed bytes from the
front of the recording buffer:

```cpp
// TODO(anton-matosov): Consider keeping a lastWritePosition instead of erasing.
//  Or use range and update it every write
record.request.bytes.erase(record.request.bytes.begin(), record.request.bytes.begin() + size);
```

`std::vector::erase` from the front is O(n) because it shifts all remaining elements.
For large recordings this degrades performance quadratically. Using an index avoids the copies.

## Goal

Replace the erase-from-front with a `size_t writeOffset` that advances past verified bytes.
The underlying `bytes` vector is never mutated; the offset is reset when a new recording is
loaded.

## Acceptance criteria

- [ ] `SerialPlayer` no longer calls `erase` on `request.bytes`.
- [ ] A `size_t writeOffset` (or equivalent) is maintained per recording entry.
- [ ] All existing `SerialPlayer` tests pass.
- [ ] The TODO comment is removed.

## Implementation approach

### Turn 1 — Implement
Locate the `Recording` (or `Record`) struct that holds `request.bytes`. Add a `size_t writeOffset = 0`
field. Replace the erase call with `writeOffset += size`. Update the comparison loop to index
from `writeOffset`.

### Turn 2 — Verify
Run all tests in `drqp_serial` and `drqp_a1_16_driver`. Ensure replay-based tests still pass.

## Files to modify

| File | Change |
|------|--------|
| `packages/runtime/drqp_serial/src/SerialPlayer.cpp` | Remove erase; add offset tracking |
| `packages/runtime/drqp_serial/include/…/SerialPlayer.h` (if struct is there) | Add `writeOffset` field |

## Priority note

Only address if `SerialPlayer` appears in a profiling hot-path. Otherwise this is opportunistic
cleanup — pick it up when touching this file for another reason.

## Dependencies

None.
