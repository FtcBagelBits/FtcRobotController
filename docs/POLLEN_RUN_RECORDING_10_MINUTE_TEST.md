# Run recording: 10-minute confidence check

Use this instead of the full validation plan when practice time is limited.
`pollen-run-recording` at `681298a`, build **pollen-hive-v10**, is installed.
Computer tests, Android unit tests and the APK build already passed on the Mac.
No rebuild or reinstall is needed for this check.

Have an empty robot, a clear area, the Driver Hub and the Mac ready on robot Wi-Fi.
One person drives; one checks the files. Stop for unexpected movement or loss of control.

| Time | Do this | Pass check |
| --- | --- | --- |
| 0:00–1:00 | INIT a previously validated Odo shooting auto at its normal start. Check Build and Hub storage. Run it empty, then STOP. | Normal motion and finish; no new fault. This also creates one match-auto recording. |
| 1:00–2:00 | Immediately select PollenBot TeleOp. Keep the robot still through INIT; wait for Pinpoint READY, then START. | No movement during INIT, no Pinpoint error, normal transition into TeleOp. This targets the new Pinpoint initialization after autonomous. |
| 2:00–4:00 | Idle 5 s. Circle; drive roughly one tile toward the hive, strafe left, pause 1 s with intake on, then Square. Briefly check rotation, intake reverse, empty launcher/feed and release. Circle again, drive a short return, then STOP with that rehearsal open. | Normal control response and mechanism behavior. Circle gives one buzz, Square two. Recording screen shows the correct rehearsal numbers. STOP halts operation. |
| 4:00–9:00 | Download once after STOP using `bash tools/ftc.sh logs NEW_SESSION_FOLDER`. While it transfers, note whether controls felt normal. | Session folder and ZIP produced; analysis errors reviewed. Inspect only the two runs just made, rather than all historical data. |
| 9:00–10:00 | Open the new TeleOp summary/SVG, its raw CSV and the new `auto/*.csv`. | TeleOp has START/settings, two rehearsals and final STOP; second rehearsal ends by STOP. Drawing has the expected forward/left shape, pause/intake interval and plausible distances. Odo CSV has START, phases and STOP. No driver-visible lag; investigate a reported loop gap >=250 ms. |

**Decision:** if all checks pass, continue practice with recording enabled. This establishes a
basic operation-and-evidence check, not precise waypoint accuracy or loaded hive performance.
If controls or the handoff fail, stop using this build for driving until investigated. If only
recording/download fails, do not spend the remaining practice time collecting unusable rehearsals.

Downloads include all retained recordings and can exceed five minutes. At minute 10, end the
physical test and leave the transfer running; mark file verification pending until it finishes.
A successful ZIP alone is not proof of valid analysis. `auto-summary.md` covers hive traces;
check the new Odo `auto/*.csv` directly.

Defer storage pruning, deliberately induced faults, pipeline benchmarks, other auto variants,
loaded shooting and precise tape-measured waypoint validation. Do not adopt generated waypoints
as an autonomous route based only on this check.
