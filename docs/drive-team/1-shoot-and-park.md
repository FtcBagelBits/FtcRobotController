# Odo ROBOT Shoot + Park

**Use this when we shoot first.** It is our main autonomous.

The robot shoots its 4 pollen, then drives to the parking slot on its own.
It measures where it is as it drives (odometry), so it ends up in the same place every time.

## What the robot does

<div class="side" markdown="1">
<div markdown="1">

![Route: start at the wall, three legs to the INNER parking slot](img/shoot-and-park-route.svg)

</div>
<div markdown="1">

| Time | What happens |
| --- | --- |
| 0 - 0.5 s | Backs gently into the wall behind it, to line itself up straight. This is normal. |
| 0.5 - 3.5 s | Launcher spins up. |
| 3.5 - 13.5 s | Shoots the pollen. |
| about 13.5 - 19 s | Drives: (1) a little off the wall, (2) left past the hive, (3) to the **INNER** parking slot. Then it stops. |

Points if it works: **LEAVE 3 + PARK 5**. Our partner parks in the **OUTER** slot.

</div>
</div>

## Before the match

**1. Agree with the partner.** We shoot first. They shoot second (or don't shoot), stay out
of our lane, and park in the OUTER slot. Show them the pit sheet.

**2. Put the robot on the field.**

<div class="side" markdown="1">
<div markdown="1">

![Placement: back flat on the wall, centered on the hive, 3.7 in from the center seam](img/shoot-and-park-placement.svg)

</div>
<div markdown="1">

- **Red:** at the **audience** wall. **Blue:** at the **scoring-end** wall.
- Back of the robot **flat against the wall**, facing the hive.
- **Center the robot on the hive.** Its edge is 3.7 in (9.5 cm) from the center seam.
- Exactly **4 pollen** loaded. Not touching a flower. Not in the loading zone.

</div>
</div>

**3. On the Driver Hub**, pick **Odo ROBOT Shoot + Park**. It's in the autonomous list, group *Pollen Odometry*.

**4. Press INIT.** Don't touch the robot for 1 second: the odometry sensor is getting ready.

<div class="page-break"></div>

**5. Press Triangle once.** That puts every setting back to the defaults. Then make only the
changes the coach asks for (usually none).

**6. Check the screen:**

| Line | Should say |
| --- | --- |
| Build | `pollen-odo-v5` |
| Mode | `ODOMETRY Shoot + Park` |
| Odometry x fwd / y left / heading | numbers near 0 (not `NOT VALID`) |
| Settings notice | `Defaults: shoot, then 3 legs to the parking slot` |
| Status | `READY - no movement until START` |

**7. Press START** when the match starts.

<div class="side buttons" markdown="1">
<div markdown="1">

![Buttons during INIT: Triangle defaults, Circle short fallback, Square does nothing](img/shoot-and-park-buttons.svg)

</div>
<div markdown="1">

- **Triangle:** back to the defaults. Always safe.
- **Square:** does nothing. (It used to load this route; now it's the default.)
- **Circle:** the short fallback. Only if the coach says. See [Leave-the-wall fallback](fallback-leave-the-wall.md).
- **D-pad:** changes settings. Only if the coach says.

</div>
</div>

## If something is wrong

| You see | What it means | Do this |
| --- | --- | --- |
| `BLOCKED: ... No goBILDA Pinpoint named "odometry"` | The odometry sensor isn't found | Use **Pollen ROBOT Shoot + Park** instead ([timed fallback](fallback-timed-autos.md)). Tell a programmer. |
| `Odometry ...: NOT VALID` | The odometry sensor isn't reading | Press STOP, then INIT again with the robot still. Still bad: use the timed fallback. |
| `READY FOR DEPARTURE ONLY - SHOOTER FAILED` | The launcher didn't start up | The robot will still drive and park, but won't shoot. Tell a programmer; check the launcher cables. |
| `NOT READY - over time budget` | Someone's settings take too long | Press Triangle. |
| Any other `BLOCKED` | Something isn't plugged in, or the wrong robot configuration is active | Tell a programmer. |

## After the match, tell the programmers

- Did it shoot all 4? Did it park in the slot? Was it touching the side wall?
- Anything under **Warning** on the screen, for example `LEG 3 TO PARK gave up 4.0 in short`.
- If it stopped early: the last **Phase** shown, for example `NO ODOMETRY POSE - STOPPED` or `TIME LIMIT - STOPPED`.
