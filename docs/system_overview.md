# System Overview

> **Note:** The source code for this project is kept in a private repository. Script names below refer to that repository. Access is [available on request](https://github.com/dhanushanand-dev).

## Goal

Sort non-ferrous scrap automatically. A depth camera finds and localises each piece on a conveyor, and a robot arm picks it.

## Work cell

```
            Intel RealSense D435i  (fixed, looking down: eye-to-hand)
                       │
                       ▼
   ┌───────────────── conveyor (stepper motor, Arduino-controlled) ─────────────────┐
   │   parts ──►   [ ROI watched by the camera ]   ──► proximity sensor             │
   └─────────────────────────────────────────────────────────────────────────────────┘
                       ▲
                Dobot MG400 (beside the conveyor, Ethernet / TCP/IP to the PC)
```

## 1. RGB-D acquisition

- Colour and depth streams: 640 × 480, 30 fps (`rgb8`, `z16`).
- `realsense.align` aligns the depth frame to the colour frame, so pixel (u, v) means the same point in both images.
- The depth scale and the intrinsics (fx, fy, cx, cy) are read from the camera, using its factory calibration.

## 2. Detection by blob analysis

1. Crop a region of interest (ROI) over the conveyor.
2. Segment the objects. For the coloured test cubes, threshold the HSV hue, saturation and value bands (red, green, blue, yellow). For bright metal pieces, threshold the brightness (white / reflective blobs).
3. Clean the mask:
   - `imopen` with a 3-px disk
   - `imclose` with a 5-px disk
   - `bwareaopen` removes blobs under 150 px
4. `regionprops` returns each blob's centroid, bounding box, area and pixel list.

Blob analysis needs no training data, runs in real time, and is easy to tune for a new object colour or finish.

## 3. 3D localisation

For each blob:

- **Z:** the median of all valid depth pixels inside the blob. This is robust to holes and noise.
- **X, Y:** back-projected with the pinhole model:
  `X = (u − cx) · Z / fx`, `Y = (v − cy) · Z / fy`
- The coordinates are logged in millimetres and drawn on the live view. The view also shows the depth map, colourised depth and the live point cloud.

## 4. Camera-to-robot calibration (eye-to-hand)

The camera is fixed, so one rigid transform maps camera coordinates to the robot base frame.

1. Place several coloured cubes in the ROI. The camera detects them and records each cube's camera XYZ. The camera X and Y axes are swapped to line up with the robot's axis convention.
2. Jog the MG400 tool onto each cube in turn. The script reads the robot pose with `GetPose(user=1,tool=1)` on the dashboard port.
3. Solve for the rotation R and translation t that best map the camera points onto the robot points. This uses SVD (the Kabsch / Arun method):
   - centre both point sets
   - compute H = Σ (p_cam − p̄_cam)(p_rob − p̄_rob)ᵀ
   - take the SVD of H, then R = V Uᵀ (with a reflection fix if det(R) < 0)
   - t = p̄_rob − R p̄_cam
4. Save R and t to a `.mat` file for the run-time scripts.

## 5. Verification and robot motion

- Reload R and t, then detect objects live and compute `p_robot = R · p_cam + t`.
- Command the MG400 over TCP/IP:
  - `EnableRobot()`, `ClearError()`, `SetBackDistance(30)`, `SetPostCollisionMode(1)`
  - `MOVL(x, y, z, r, SPEEDJ=…, ACCJ=…)` to the target, with a Z floor for safety
- Verification was done with coloured cubes, both still on the conveyor and while the belt was moving. The robot moving onto each cube visually confirms the calibration (see the demo videos).

## 6. Conveyor integration

- An Arduino Uno, driven from MATLAB, runs the belt's stepper motor and reads a proximity sensor.
- When a part reaches the sensor, the belt stops and the MG400 runs its pick-and-place motion sequence (`MOVJ` way-points). The belt then restarts.

## MG400 TCP/IP ports

| Port | Use |
|---|---|
| 29999 | Dashboard: enable, clear errors, settings, `GetPose` |
| 30003 | Motion commands |
| 30004 | Real-time feedback |

The robot IP is configurable.
