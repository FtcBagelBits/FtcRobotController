# TEST Camera Drive

Use this separate TeleOp to move PollenBot around and inspect what its Limelight sees
at different distances and angles. Build label: **pollen-camera-v1**.

The match **PollenBot** TeleOp and autonomous logic are unchanged. This test mode does
not shoot, feed, use odometry, aim automatically, or evaluate hive UP/DOWN readiness.
It reads raw results from the camera's **currently active pipeline**. It never switches
pipelines, edits exposure, or saves calibration. The shared build label changes so the
installed APK can be identified.

## Setup and use

1. Build and install using [Desktop tools](DESKTOP_TOOLS.md). Stop the current OpMode
   before installing. Installation restarts the Robot Controller app.
2. Leave the Limelight connected to the Control Hub's **blue USB 3.0 port**. Use the
   usual PollenBot hardware configuration: `frontLeft`, `frontRight`, `backLeft`,
   `backRight`, and `limelight`. Configured `pollenLauncher`, `intake`, and `agitator`
   are explicitly set to zero. No odometry device is required.
3. On the Driver Hub select **TeleOp → TEST Camera Drive** (group **Pollen Test**),
   then press **INIT**. Verify build **pollen-camera-v1**. The camera/viewer starts
   during INIT; the wheels stay stopped.
4. Connect the laptop to this robot's Wi-Fi and open
   **http://192.168.43.1:5800/** in Chrome. This is an HTTP address, not HTTPS.
   The page shows the camera stream and has **Pause video** / **Reconnect video** buttons.
   Only one video viewer is allowed at a time. Close other video tabs before reconnecting;
   an old connection may take up to a few seconds to clear.
5. Check the **Active pipeline**, **Camera** freshness, and **AprilTags** lines on the
   Driver Hub. A fresh frame with no tags is a valid observation. Stale results are
   labeled, and their old target values are hidden. The browser picture and telemetry
   arrive independently; they are not synchronized frame-by-frame.
6. Press **START**, release L1 once, then use the controls below. Start with a short
   movement and confirm directions before exploring the field.
7. Press **STOP** when done. Motors are commanded to zero, result polling stops, and
   the viewer's listener and browser connections close. A browser may retain its last
   image; that does not mean the camera view is still live. INIT again and reconnect
   the page for another run.

| Control | Effect |
| --- | --- |
| Hold **L1** | Enables driving, capped at 30% motor power |
| Hold **L1 + R1** | Fine positioning, capped at 15% motor power |
| Release **L1** | Immediately commands zero drive power |
| Left stick forward/back | Robot-relative forward/back, same orientation as PollenBot TeleOp |
| Left stick left/right | Strafe |
| Right stick left/right | Turn |
| Triggers / face buttons / D-pad | No mechanism or pipeline action |

Holding L1 across INIT/START does not move the robot: release it once after START.
Power caps are motor-power fractions, not calibrated speed limits. Fine power may not
overcome friction on every surface. The launcher is at the robot's back, so driving
"forward" does not necessarily move toward the camera's view.

## First physical checks

- INIT: confirm live video and fresh results with the robot stationary. Move a target
  or hand through the view to verify the image is updating.
- START: confirm sticks alone do nothing; hold L1 for a short forward, strafe and turn
  test; release L1 and verify stopping. Repeat with L1 + R1.
- Compare the Driver Hub **Loop interval**, result age and control responsiveness with
  video open and paused. Streaming uses the Hub's CPU/USB/Wi-Fi; its practical impact
  must be measured on this robot. Keep the driver watching the robot, not driving
  solely from a potentially delayed image.
- Stop at several distances/angles and note which tag IDs appear or disappear. This
  mode shows raw AprilTag observations, not a calibrated hive-ready decision, and does
  not record a session log. Use the existing pipeline benchmark for recorded comparisons.
- STOP with a video tab open, then INIT again. Confirm the viewer reconnects and that
  the normal PollenBot TeleOp still works after leaving this test.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Page will not load | Confirm this mode is in INIT/running, the laptop is on the intended Hub Wi-Fi, and `http://192.168.43.1:8080` is reachable. Check **Viewer** on the Driver Hub. |
| Address already in use | Another service owns port 5800. This mode reports the bind failure; it does not stop or replace that service. Resolve the conflict before retrying. |
| Page loads but video fails | Check Limelight power/USB, close other viewers, and click Reconnect. **Viewer** reports camera HTTP errors/timeouts. The page itself does not prove the camera is reachable. |
| SDK results work but video does not | The viewer targets the default camera USB address `172.29.0.1`, port `5800`. If the camera has a custom USB index/address, update `PollenCameraViewer.CAMERA_STREAM` and rebuild. The SDK's configured address is separate. |
| No tags / wrong pipeline | Read **Active pipeline**. Choose the desired saved pipeline using the existing camera workflow before this test. No settings are changed here, and a non-AprilTag pipeline will not yield tag IDs. |
| Video failure but driving works | Intentional: the viewer is independent of manual driving. Release L1 while investigating; use Driver Hub STOP normally. |
| Camera or drive hardware missing | **INIT failed; driving disabled** is shown. Fix configuration, STOP, then INIT again. |

## Implementation and verification

- `PollenCameraDrive`: this OpMode alone maps the camera/drivetrain and owns viewer lifetime.
- `PollenCameraDriveControl`: pure Java stick mix, startup release gate, deadband and power caps.
- `PollenCameraViewer`: a bounded, viewing-only HTTP gateway. Port 5800 `/` serves the
  page; `/stream` reads the fixed Limelight MJPEG source. It forwards no browser camera
  settings requests and does not expose the full Limelight dashboard. No new dependency.
- Background workers handle video. The drive loop only reads cached SDK results and
  writes motor power. No viewer is started by the match TeleOp or autonomous modes.
- `bash tools/ftc.sh test` includes drive-gate/mix tests and real loopback-socket tests:
  binary image transfer, MJPEG boundary preservation, rejected writes/unknown routes,
  camera errors/reconnect, single-stream limit, and STOP during a stalled upstream.
- Compile/API checks and loopback tests do not validate the camera's firmware, Wi-Fi
  load, or physical robot behavior. Perform the checks above after installing.

References: [Limelight 3A hardware and streaming](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a),
[FTC Limelight API](https://javadoc.io/doc/org.firstinspires.ftc/Hardware/12.0.0/com/qualcomm/hardware/limelightvision/Limelight3A.html).
