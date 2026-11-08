# Fallback: leave the wall only (Circle)

**The coach decides when to use this.** It's for when the full route isn't safe, for example the
partner can't stay out of our lane, or we're on a field we haven't tested.

In **Odo ROBOT Shoot + Park** (or the timed **Pollen ROBOT Shoot + Park**), after INIT and Triangle,
press **Circle** once. **Do it again every match you want it:** at the next INIT the robot goes back
to the full route by itself (since `pollen-odo-v6`), so a Circle press can't sneak into a later match.

The robot shoots exactly the same way, then only drives **about 2 feet (61 cm)** forward and
to the left, away from the wall. It does **not** go to the parking slot.

- Points: **LEAVE 3** only, not PARK. Our partner can use either parking slot.
- Place the robot exactly as on the [Shoot + Park](1-shoot-and-park.md) page.
- The first line of the screen says `ROUTE: LEAVE ONLY (Circle): shoot, then a 2-ft move. WILL NOT PARK`,
  and the Settings notice says
  `LEAVE-THE-WALL FALLBACK: shoot, then a 2-foot move; does NOT park. Triangle = full route`.
- Changed your mind? Press **Triangle** to go back to the full route.
- Circle does nothing in the Supporting Shooter.
