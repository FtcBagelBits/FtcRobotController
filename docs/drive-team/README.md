# Drive team guides

For the students who run the robot at matches. Each guide prints on one sheet (the two autos and
TeleOp front and back); the PDFs are in [print/](print/). Build `pollen-hive-v41`.

## Choose a Driver Hub mode

| Group | Autonomous | TeleOp |
| --- | --- | --- |
| 1 Match | Primary Shooter + Garden, Secondary Shooter + Flower | PollenBot |
| 2 Beta Test | Beta Primary Shooter - Hero, Beta Primary Shooter - Fast, Beta Primary Shooter - Angled Left, Beta Secondary Shooter - Out of the Way, Beta Secondary Shooter - Angled | |
| 3 Emergency | Emergency Odo Shoot + Park, Emergency Odo Supporting Shooter, Emergency Odo Park, and the three Emergency Timed modes | |
| 4 Calibration | Primary or secondary shot only, angled primary, garden pickup, garden + return, flower drain | Hive camera, hive pipeline benchmark, launcher, drainer servo, odometry, Pedro Pathing |
| 5 Legacy | | Legacy Lox Drive and Legacy Lox Mecanum use the Lox hardware configuration. |

**Starting a Match or Beta shooter:** Press INIT and use the first row. D-pad **left selects RED**.
D-pad **right selects BLUE**. Check the alliance and tag range on screen before START. Each INIT
requires a fresh choice. If you already pressed START with no color, press STOP and INIT again.
Square saves tuning. **Park margin s** defaults to 1.0 and changes in 0.5 s steps. It is the
projected time left after parking. The alliance choice applies to this run.

## Print these for an event

| Page | When | Copies |
| --- | --- | --- |
| [Partner sheet](../POLLEN_PIT_SHEET.svg) ([PDF](print/pit-sheet.pdf)) | Show our alliance partner our two autos, lanes, slots and timing. Landscape. | 3 (one to hand over) |
| [6. Primary Shooter + Garden](6-primary-garden.md) ([PDF](print/6-primary-garden.pdf)) | Match auto when we shoot first. Front and back. | 2 |
| [7. Secondary Shooter + Flower](7-secondary-flower.md) ([PDF](print/7-secondary-flower.pdf)) | Match auto when the partner shoots first. Front and back. | 2 |
| [3. PollenBot TeleOp](3-teleop.md) ([PDF](print/3-teleop.pdf)) | Every match, after autonomous. Front and back. | 2 |
| [Drive coach strategy](coach-strategy.md) ([PDF](print/coach-strategy.pdf)) | Clock calls, tip counting, endgame and SWARM decisions, legal defense, partner talk. | 1 per coach |
| [4. Hive match and practice](4-hive-practice.md) ([PDF](print/4-hive-practice.pdf)) | Camera calibration and practice modes, for the pit. | 1 |
| [8. Beta Primary Shooter - Angled Left](8-beta-primary-angled.md) ([PDF](print/8-beta-primary-angled.pdf)) | Beta: the match Primary, both volleys from an angled spot. Front and back. | 1 |
| [9. Beta Secondary Shooter - Angled](9-beta-secondary-angled.md) ([PDF](print/9-beta-secondary-angled.pdf)) | Beta: the match Secondary, both volleys from an angled spot. Front and back. | 1 |

## Keep in the binder (modes we don't expect to use)

| Page | When |
| --- | --- |
| [Emergency Timed autos](fallback-timed-autos.md) ([PDF](print/fallback-timed-autos.pdf)) | **Emergency only:** the odometry sensor isn't working. |
| [1. Emergency Odo Shoot + Park](1-shoot-and-park.md) ([PDF](print/1-shoot-and-park.pdf)) | We shoot first using the odometry fallback. |
| [2. Emergency Odo Supporting Shooter](2-supporting-shooter.md) ([PDF](print/2-supporting-shooter.pdf)) | Our partner shoots first and we use the odometry fallback. |
| [Park only](fallback-park-only.md) ([PDF](print/fallback-park-only.pdf)) | Testing, or the launcher is broken. |

**Calibration Angled Primary** (group *4 Calibration*) is in the autonomous list too. It is an
experiment for practice, with no guide yet. **Don't pick it for a match** unless the coach says so.

## Setup for Emergency modes

Use the Hive practice sheet for the Match and Beta modes.


1. **Agree with the partner first:** who shoots first, who parks in which slot.
2. **Place the robot, then INIT, then press Triangle once.** Make only the changes the coach asks for.
3. **Front flat on the wall, back (launcher) to the hive** for every auto: the robot drives its routes backward.
4. **Check the first line, ROUTE, is the route you want**, the Build line says `pollen-hive-v41`, and Status says `READY`.
5. After the match, photograph the Driver Hub screen **before** INIT on TeleOp, and tell a programmer the
   match number and what happened, especially any **Warning** on the screen.

For programmers: edit these Markdown pages, then rebuild the pictures and PDFs with
`python3 tools/draw-drive-team-figures.py && python3 tools/build-drive-team-pdfs.py`.
The full reference is the [operating manual](../POLLEN_AUTONOMOUS.md).

[Hive pipeline benchmark card](print/5-hive-benchmark.pdf): timed camera comparison and log download.
