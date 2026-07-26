# Task 9 Report: Force Calibration Integration in main.cpp

**Status:** Complete

## Changes Summary

### File modified: `Touch_Client/main.cpp`

1. **Added includes** (lines 17-18):
   - `#include "force/ForceCalibration.h"`
   - `#include "force/ForceCompensation.h"`

2. **Calibration file load at startup** (lines 523-535):
   - After `initForceReader()`, loads `force_calib.json` via `ForceCalibration::loadFromFile()`
   - On success, applies params to `ForceCompensation::setCalibration()` and prints mass + residual
   - On failure, prints message telling user to press 'c' when idle

3. **Calibration state tracker in idle()** (lines 63-79):
   - After `pollForce()`, monitors `ForceCalibration::currentState()` for changes
   - Prints status text on every state transition
   - When state transitions to DONE, loads the saved file and applies results to `ForceCompensation::setCalibration()`

4. **Keyboard handler** (lines 179-216):
   - **'c' key**: Checks force calibration state first
     - If calibrating: aborts via `RelayCore::abortForceCalibration()`
     - If idle: tries `RelayCore::startForceCalibration()` (which internally checks transmission/connection/alarm)
     - Falls through to coordinate calibration toggle if force calib rejected
   - **SPACE key**: Force calibration check runs first before existing coord/FK handlers
     - If force calib active: calls `ForceCalibration::confirmPose()`

## Key Design Decisions

- **Calib file path**: Used `"force_calib.json"` (no subdirectory) to match `ForceCalibration::saveToFile()` behavior
- **Simplified keyboard guard**: `RelayCore::startForceCalibration()` already handles transmission, connection, and alarm checks internally — no need to duplicate in main.cpp
- **DONE application**: Handled in `idle()` status tracker rather than in `pollForce()` to keep user-visible output in main.cpp

## SPACE Key Priority

```
1. Force calibration confirmPose (if calibrating)
2. Coordinate calibration point recording (if collectMode)
3. FK validation point recording (if FkValidate::mode)
```

## 'c' Key Priority

```
1. Abort force calibration (if running)
2. Start force calibration (if idle + robot connected)
3. Toggle coordinate calibration mode (fallback)
```
