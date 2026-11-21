# Cycle time, double volley and parking in the match autos

Question: where do the 30 s of Primary Shooter + Garden and Secondary Shooter + Flower go on the
real robot, why does the second volley rarely happen, and what simple changes could speed the cycle
and raise the odds of two accurate volleys and a park?

Started: 2026-11-19 by Claude (cloud session). Based on branch `clear-out-v33` at `1eb1d57`
(run history from 21 downloads, builds v3-v29, 2026-11-12 to 11-18). A cloud session cannot reach
the robot or the raw downloads: every number below is read from `runs/` or the code (`[repo]`) or
reasoned from them (`[inferred]`). Nothing here has been tried on the robot yet.

## Summary

- **The second volley is the weak point.** Of 29 LIVE match-type runs, 7 came back from collection
  and **1** fired the second volley (v27 Primary). The misses: 3 left with `Hive not ready for return
  shot`, 1 with `Return pose did not settle`, 2 stopped by the driver. `[repo]` `runs/SUMMARY.md`
- **The hive read `UNKNOWN`, not `DOWN`, for the whole return window** in both v28/v29 Primary
  misses, with the camera reporting `OK, pipeline 4, 5 hive tag(s)`. So it was the classifier, not a
  tipped hive, as far as the history shows. v30-v32 moved the UP center to 50.7 in after replaying
  those frames; **no v30+ run has been downloaded**, so that fix is untested. `[repo]`
- **Six of 29 LIVE runs died at START:** 4 with no alliance chosen, 2 with the Pinpoint moved after
  INIT. That is a bigger loss than any route tuning. `[repo]`
- **Spin-up is not the bottleneck any more:** START to first feed is 0.75-0.9 s on the Primary
  (including the 0.3 s settle), not the 3 s the older plans assumed. `[repo]`
- **Slow contact creeping costs about 4.5 s per Primary cycle** (find side wall 2.2-2.8 s, inward
  pass 2.3 s), because the garden entry stops 6.5 in short of the side wall and 13.4 in out from the
  end wall. These are INIT rows. `[repo]`
- **Volleys feed through flywheel dips:** the launcher falls to 81-92 % of target and spends
  0.3-1.6 s of each volley below 95 %; the last dip came by 2.2 s in every 3 s volley. `[repo]`
- The angled shot from a spot nearer the garden saves little with today's motion, because the turns
  cost what the shorter legs save; it is worth it only if the shot is accurate there and the turns
  overlap with travel. Measure the shot first (existing Calibration Angled Primary). `[inferred]`

## What is known

### Primary, measured (v28 and v29 LIVE, `runs/2026-11-18/match-17950564…`, `…58969…`)

| Step | Seconds | Detail |
| --- | ---: | --- |
| START to first feed | 0.9-1.0 | flywheel ready + 0.3 s settle |
| First volley | 3.0 | setting; last dip 0.4-2.2 s in |
| Outbound | 3.1 | CLEAR END WALL 1.04 (13.4 in forward), ALIGN WITH GARDEN 1.97 (42 in strafe) |
| Garden | 4.7 (1 pass) / 7.3 (2 passes) | Find right wall 2.2-2.8 (7-12 in at 0.30 power), inward pass 2.3 (11.5 in at 0.25), back off and pass 2 |
| Return | 3.9-4.3 | CLEAR COLLECTION SITE 1.0, RETURN ACROSS 2.05 (49 in), RETURN SHOOT POSITION 0.8 + approach |
| Align / settle | 0.4-0.9 | v29 corrected 3.35 deg of heading after the strafe |
| Hive check before volley 2 | 1.3 (v27) | 0.3 s settle + over 1 s of steady UP |
| Second volley | 3.0-4.0 | setting |
| Park drive | 5.46 | 18.67 to 24.13 s in v29, matching the 5.46 s row |

Return position errors were 0.24-0.47 in; the tolerance is 0.5 in. In the v28 settle miss the robot
stopped 0.47 in off with 2.07 deg of heading and waited 1.5 s; since v29 the heading part is fixed
(ALIGN_SHOT), but a stop more than 0.5 in off the spot still skips the volley. `[repo]`

### Secondary, measured (v28, v29)

Waiting for UP before the first volley took 1.5-7.1 s (partner-dependent). Flower: 8.2 s with 2
shakes and 1 cleanup (v29) to 10.7 s with 3 shakes and 2 cleanups (v28): lower drainer 0.4, approach 1.8 (21 cm
at 0.20 power), 0.53-0.83 s per shake, withdraw 0.8, 1.0 s per cleanup, 1.05 s final dwell. Return
2.3 s. `[repo]`

### Launcher during volleys (`slim-trace.csv`, 15 volleys v16-v29)

Minimum 960-1080 tps against 1180 (81-92 %); 0.3-1.6 s below 95 % per volley. The feed rule starts a
volley at 96 % and then feeds without pausing ([PollenHiveAuto.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenHiveAuto.java):
"finish the volley through camera changes and flywheel dips"), so balls 2-4 leave slower than
ball 1. Battery read about 10.0-10.3 V at the first feed in the v28/v29 Primary runs (12.4-12.7 V at
START, minimum 9.7 V). Whether the slower balls miss is unknown: no `notes.md` has ball counts. `[repo]`

WPILib's flywheel tuning notes that combined feedforward and feedback recovers quickly after a ball
contacts the wheel, and that a consistent launch starts with the wheel at speed before the piece goes
in. `[doc]` [WPILib: tuning a flywheel](https://docs.wpilib.org/en/latest/docs/software/advanced-controls/introduction/tuning-flywheel.html)

### Mecanum motion

A mecanum drive strafes slower than it drives forward, and combining translation with rotation also
slows it. `[doc]` [gm0 holonomic](https://game-manual-0.readthedocs.io/en/stable/docs/drivetrains/holonomic.html),
[gm0 mecanum](https://game-manual-0.readthedocs.io/en/latest/docs/software/mecanum-drive.html).
With wheel power normalized, travel time at full power is roughly proportional to |forward| +
|sideways|, so a diagonal leg saves little distance-time over an L; what it saves is a stop. The
measured legs suggest each stop costs about 0.6-1.0 s of slowing and settling. `[inferred]`

`PollenHiveMotion` already turns while it translates on any leg whose target heading differs (turn
power capped at 0.25) and does not arrive until within 2 deg. `[repo]`

## Your angled-shot idea, estimated

Spot A = the existing Calibration Angled Primary spot: 5 in off the wall, 30 in left, turned
36.3 deg toward the hive, 50.7 in from the opening center (docs/POLLEN_FIELD_ROUTES.md). Leg times
from the measured rates above; turns about 0.8 s each if done in place. `[inferred]`

| Part | Today (S) | Angled (A), turns in place | Angled, turns during travel |
| --- | ---: | ---: | ---: |
| START to first feed | 0.9 | ~2.8 (30 in leg + turn + settle) | ~2.0 |
| Outbound to garden entry | 3.1 | ~2.2 (turn back + 15 in) | ~1.4 |
| Return to the shot spot | 3.9 + 0.9 | ~3.3 + turn | ~2.6 |
| Park from the shot spot | 5.5 | ~5.2 + turn back | ~5.0 |
| **Net per match** | | **about 0 s** | **about +2.5 s** |

So the idea pays only with the turns folded into the legs, and only if the shot from A is as
accurate as from S. Neither is known: the topspin launcher has never been tested at an angle, the
camera gate has never classified the hive from A, and the turn while translating is capped at 0.25
power. The 0.9 s spin-up is too short to hide a 30 in move behind.

## Options

| ID | Option | Status | Evidence | Next step |
| --- | --- | --- | --- | --- |
| C1 | Garden entry 8 in off the end wall (row minimum) and ~3 in short of the side wall (116 cm), so the slow contact passes are short | **Trying** in Beta Fast mode (v34) | Contact passes 4.5 s; side wall found at 49.3 in left | Run Fast mode in DRIVE, then LIVE; count balls collected |
| C2 | 2.5 s feeds instead of 3 s | **Trying** in Fast mode | Last dip by 2.2 s in every 3 s volley; v33 clear-out catches stragglers | Count balls scored per volley in `notes.md` |
| C3 | Return nudge: stopped 0.5-3 in off the spot, one short leg back (1.5 s cap) instead of skipping volley 2 | **Trying** in Fast mode | 1 of 3 recent misses; errors run 0.24-0.47 in against 0.5 | Promote to Match if it fires and lands |
| C4 | Paced feed: hold the agitator below a set % of launcher speed, restart at 96 % | **Trying** as INIT row `Pause feed below launcher %` (default 0 = off, all modes) | Volleys dip to 81-92 % | A/B in Calibration Primary Shot Only: 0 vs 92, same balls, count scores |
| C5 | `hive_reason` in the run history's slim traces | **In use** (tool) | Return misses read UNKNOWN with no reason kept | Laptop agent: re-run `tools/run-history.py` |
| C6 | Angled shot from a spot nearer the garden | Proposed | Estimate above | Calibration Angled Primary: 10 shots at A, count scores; only then a beta mode |
| C7 | Straight shot from a start shifted up to ~5 in left (still inside the 20 in cell opening) | Proposed | ~0.5 s per cycle; no turn, camera sideways gate 12.5 in | Place the robot shifted; set entry-left and park rows to match |
| C8 | Merge legs (diagonal outbound to the garden entry; return across and approach as one leg) | Proposed | Each stop ~0.6-1.0 s | After C1 data; check footprints |
| C9 | Alliance at START: use the cell the camera locked during INIT when the driver did not choose | Proposed, **mentor decision** | 4 of 29 LIVE runs lost; Ari's INIT principle (2026-11) was autodetect with override, `57d334c` (Codex, 2026-11-18) made the choice mandatory | Decide; small code change |
| C10 | Flower: approach at 0.30 instead of 0.20 (creep still seats), fewer shakes if counts allow | Proposed | Approach 1.8 s; shakes 0.6-0.8 s each | Calibration Flower Drain with ball counts |
| C11 | Park margin 0.5 s instead of 1.0 s | Proposed | Measured park 5.46 s = the 5.46 s row (v29) | Two more timed parks first |
| C12 | Fresh battery each match; launcher speed check at INIT | Proposed | 10 V at the volley in practice | Note battery in `notes.md` |

## Questions for the agent on the laptop (raw downloads)

1. For `match-1795056412358-581495091`, `match-1795058969741-1506116415` (Primary) and
   `match-1795056741649-1788934707` (Secondary): print every trace row in SECOND with
   `hive_state`, `hive_reason`, `cell` and `tags_id_xyz_m_height_in`. Are the tag heights inside
   47.7-53.7 in (v32 band), inside the band of their own build, or missing? Say which.
2. Re-run `tools/run-history.py` over the downloads so `slim-trace.csv` carries `hive_reason`.
3. If anyone remembers ball counts for the v27-v29 LIVE runs, fill those `notes.md` files.

## Not verified

- Everything in the Options column marked Trying: computer tests and simulation only.
- Whether slower balls in a volley miss; whether 2.5 s feeds leave balls; whether 8 in garden entry
  still collects all four balls.
- The angled-shot estimate (rates extrapolated from a handful of legs).

## Decision log

- 2026-11-19, Ari: asked for beta and calibration modes to try cycle-time ideas, including an angled
  shot nearer the reload point. Claude cloud session added Beta Primary Shooter - Fast (C1-C3)
  and the paced-feed row (C4); C6-C12 left as proposals.

## Sources

- `runs/SUMMARY.md`, `runs/index.csv`, `runs/2026-11-*/match-*/` (this branch)
- [gm0: holonomic drivetrains](https://game-manual-0.readthedocs.io/en/stable/docs/drivetrains/holonomic.html)
- [gm0: mecanum programming](https://game-manual-0.readthedocs.io/en/latest/docs/software/mecanum-drive.html)
- [WPILib: tuning a flywheel velocity controller](https://docs.wpilib.org/en/latest/docs/software/advanced-controls/introduction/tuning-flywheel.html)

## Can the flywheel identify the fourth ball?

2026-11-21, Codex local session on `match_2_mods` at `9ede19b`. Replayed the November 19
LIVE Hive traces on the laptop and cross-checked two examples against the desktop download.

- [repo] The run-history dip counter requires a drop below 90% of target followed by recovery
  above 95%. It reports four dips in 2 of the 13 recorded opening volleys with feed that day.
  See `tools/run-history.py` and `runs/index.csv`.
- [tested] On the laptop, a replay of those 13 opening volleys flagged four candidate events in
  10 volleys. During active feed, the replay flagged a drop of at least 60 ticks/s from a running
  high, tracked the low, and rearmed after a rise of 40 ticks/s. The fourth candidate appeared
  1.50–2.36 s after feed started in those ten volleys. In the v36 Primary run
  `match-1795141993215-1788558765`, the four candidate times were about 0.12, 0.44, 0.97 and 1.63 s
  into a 3 s feed. The v36 angled run `match-1795141682566-1515457612` dipped from 1180 to
  1060, partially recovered to 1120, then dropped again to 1000 ticks/s. That sequence stays
  below the run-history recovery threshold between the two drops.
- [repo] The November 19 run notes lack observed ball exit counts per Hive volley. The 13-volley
  replay used the existing Hive trace samples, roughly 0.11 s apart during these feeds.
- [inferred] Try a telemetry-only estimate in Calibration Primary Shot Only. Record timestamped
  loop speed during operator-observed trials with 0, 1, 2, 3 and 4 balls. Ground truth is the
  number and time of exits, ideally from video. Use the timed feed and park deadline while
  measuring missed and extra counts, especially when balls enter close together or jam.
