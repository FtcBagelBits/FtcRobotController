# Drive team guides

For the students who run the robot at matches. Each guide prints on one sheet (the two autos and
TeleOp front and back); the PDFs are in [print/](print/). Build `pollen-odo-v6`.

## Print these for the pit (one copy each, plus one for the drive coach)

| Page | When |
| --- | --- |
| [1. Odo ROBOT Shoot + Park](1-shoot-and-park.md) ([PDF](print/1-shoot-and-park.pdf)) | **Our main auto.** We shoot first. |
| [2. Odo Supporting Shooter](2-supporting-shooter.md) ([PDF](print/2-supporting-shooter.pdf)) | Our partner shoots first. |
| [3. PollenBot TeleOp](3-teleop.md) ([PDF](print/3-teleop.pdf)) | Every match, after autonomous. |
| [Pit sheet for partners](../POLLEN_PIT_SHEET.svg) ([PDF](print/pit-sheet.pdf)) | Show our alliance partner (and opponents) our lanes, slots and timing. Print landscape. |

## Keep in the binder (modes we don't expect to use)

| Page | When |
| --- | --- |
| [Timed autos](fallback-timed-autos.md) ([PDF](print/fallback-timed-autos.pdf)) | The odometry sensor isn't working. |
| [Leave the wall only](fallback-leave-the-wall.md) ([PDF](print/fallback-leave-the-wall.pdf)) | The coach says the full route isn't safe this match. |
| [Park only](fallback-park-only.md) ([PDF](print/fallback-park-only.pdf)) | Testing, or the launcher is broken. |

## Rules for every match

1. **Agree with the partner first:** who shoots first, who parks in which slot.
2. **Place the robot, then INIT, then press Triangle once.** Make only the changes the coach asks for.
3. **Check the first line, ROUTE, is the route you want**, the Build line says `pollen-odo-v6`, and Status says `READY`.
4. After the match, photograph the Driver Hub screen **before** INIT on TeleOp, and tell a programmer the
   match number and what happened, especially any **Warning** on the screen.

For programmers: edit these Markdown pages, then rebuild the pictures and PDFs with
`python3 tools/draw-drive-team-figures.py && python3 tools/build-drive-team-pdfs.py`.
The full reference is the [operating manual](../POLLEN_AUTONOMOUS.md).
