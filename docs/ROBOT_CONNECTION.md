# Set up, test, install, and share your changes

For anyone programming PollenBot, on Windows or Mac. Start with Coach Richard's setup email
(install Android Studio, tick Android 16 / API 36, clone the repo). This page covers what comes
after that, plus what trips people up. Mentors: the Mac Terminal version of everything here,
including the full connection reset, is in [CLAUDE.md](../CLAUDE.md).

## 1. One-time setup: things to know

- **Android Studio version.** The project needs Android Studio **Narwhal 3 Feature Drop
  (2025.1.3) or newer**: its Android Gradle plugin is 8.13.2. A fresh download is fine; an
  old install must be updated.
- **Say no to "upgrade the Android Gradle plugin".** Newer Android Studios offer to upgrade it.
  Decline: the FTC SDK picks that version (the build file says "Do not modify this yourself").
- **Do the first build at home.** The first Gradle sync downloads the FTC SDK, Pedro Pathing
  and Panels (hundreds of MB). School Wi-Fi may block some of those sites, and the robot's Wi-Fi
  has no internet at all. Once one build has worked, the downloads are saved on your laptop.
- **Check it worked:** Build → Make Project ends with `BUILD SUCCESSFUL`.
- **If Gradle sync fails with a Java error:** Settings → Build, Execution, Deployment → Build
  Tools → Gradle → Gradle JDK → pick the one that comes with Android Studio (jbr-21).

## 2. Run the computer tests (no robot needed)

Open `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/pollen/PollenTests.java` and click the
green arrow next to `class PollenTests` (or right-click the file → Run). You get one line per
test program; all should be green. A red one's message says which check failed.

They check autonomous timing, the INIT menus, time budgets, slow loops, and the odometry autos
against a *simulated* robot. Passing them is not a robot test, but run them before every pull
request that touches `pollen/`.

## 3. Install on the robot over Wi-Fi

Our team has never got a USB-C cable to the Control Hub working, so we use Wi-Fi.

1. **Build first, while you still have internet** (Build → Make Project).
2. **Power on the robot** and wait about 30 s, until the Control Hub's light is solid green.
3. **Join the Control Hub's Wi-Fi** on your laptop. The name starts with `FTC-`; ask a mentor
   for the password. Turn off any VPN. Your laptop has no internet now; that's expected.
4. **Connect adb** (the tool Android Studio uses to install apps). Open the **Terminal** tab at
   the bottom of Android Studio and paste the lines for your computer:

   Windows (the Terminal tab is PowerShell):
   ```powershell
   $adb = "$env:LOCALAPPDATA\Android\Sdk\platform-tools\adb.exe"
   & $adb connect 192.168.43.1:5555
   & $adb devices
   ```

   Mac:
   ```bash
   adb=~/Library/Android/sdk/platform-tools/adb
   $adb connect 192.168.43.1:5555
   $adb devices
   ```

   Expect `connected to 192.168.43.1:5555`, then a line `192.168.43.1:5555   device`.
   (192.168.43.1 is always the Control Hub's address on its own Wi-Fi.)
5. **Install:** in the device menu at the top of Android Studio, pick the Control Hub
   (`REV Robotics Control Hub` or `192.168.43.1:5555`). Make sure the run setting next to it is
   **TeamCode**, then click the green Run ▶ button.
6. **Check:** the Robot Controller app restarts on the hub. On the Driver Hub, INIT any Pollen
   mode and check that the **Build** line matches `PollenConfig.BUILD` in the code.
7. Rejoin your normal Wi-Fi when you're done.

### When it won't connect

Stop at the first row that matches.

| What you see | Do this |
| --- | --- |
| `adb connect` times out or "No route to host" | Check you're on the hub's Wi-Fi (laptops drift back to home Wi-Fi), the VPN is off, and the hub has finished booting. |
| `adb devices` shows `offline` or `unauthorized` | Run `adb disconnect` (Windows: `& $adb disconnect`), power-cycle the hub, rejoin its Wi-Fi, connect again. |
| Still nothing | Quit Android Studio, then in a terminal run `adb kill-server` (Windows: `& $adb kill-server`), reopen Android Studio and connect again. An old adb left running is the most common cause. |
| The device menu shows an emulator (`emulator-5554`) | Close the emulator (Device Manager → stop) so you don't install onto it by mistake. |
| Gradle sync fails while on the hub's Wi-Fi | It wanted the internet. Rejoin normal Wi-Fi, build, then switch back and click Run. |
| `INSTALL_FAILED_UPDATE_INCOMPATIBLE` | The app on the hub was installed from another computer. Ask a mentor before uninstalling anything. |

Windows alternative: the REV Hardware Client, if installed and running, tries to connect adb to
a Control Hub on Wi-Fi by itself.

## 4. Share your change: branch and pull request

`main` is protected: nobody pushes to it directly. Every change goes through a pull request (PR)
that a mentor reviews.

1. **New branch:** Git → New Branch (bottom-right branch menu → New Branch). Name it after the
   change, like `teleop-slow-mode`.
2. **Change code** only under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` (and its
   tests under `TeamCode/src/test/`) unless a mentor says otherwise.
3. **If what the robot does changes**, bump `BUILD` in `pollen/PollenConfig.java`
   (for example `pollen-odo-v10` → `pollen-odo-v11`), so the drive team can see which code is installed.
4. **Run the tests** (section 2) and **build** (Build → Make Project).
5. **Commit:** Git → Commit. Write one line saying what changed and why.
6. **Push:** Git → Push. Android Studio asks you to log in to GitHub the first time.
7. **Open the PR:** on github.com, open the repository; GitHub shows a **Compare & pull request**
   button for your new branch. Say what you changed and whether you tested it on the robot.

## 5. After an event: save the robot's log

The Pollen shoot autos write what they did to the robot's log (search it for `PollenAuto`).
The log gets overwritten over time, so a mentor saves it after each event: on the robot's Wi-Fi,
open `http://192.168.43.1:8080` in a browser → **Manage** → **Download Logs**. The adb way is in
[CLAUDE.md](../CLAUDE.md).
