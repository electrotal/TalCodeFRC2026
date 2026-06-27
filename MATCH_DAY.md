# MATCH DAY — what to do, in order

This is the master checklist. Other docs: **ELASTIC_DASHBOARD.md** (dashboard layout),
**TUNING.md** (every value you can change), **AUTONOMOUS.md** (drawing autos).

---

## A. At home / pit — deploy the code
1. Branch is `reliability-shooting-overhaul`. In WPILib VS Code: **Deploy Robot Code**.
2. After deploy, on Elastic check `Build/Version` + `Build/DeployTime` — confirm it's the new build.
3. Confirm no errors scrolling in the Driver Station console.

## B. First power-up — fix the hood motor (the one open item)
The code is verified: it *does* command the hood motor. If it doesn't spin, it's wiring/ID/power.
1. Enable the robot (Teleop). Hold **RT**.
2. Watch `Hood/CmdOutput` on Elastic — should read ~0.1–0.2.
3. Look at the **Spark MAX status LED** on the hood motor:

| LED | Meaning | Fix |
|---|---|---|
| Orange/Magenta blink | **Sensor fault — NEO hall cable** (most likely) | Reseat the ~6-pin ribbon at the Spark MAX *and* the NEO |
| Orange/Yellow blink | CAN fault / wrong ID | REV Hardware Client → confirm device at **ID 26** |
| Orange/Blue blink | No 12 V | Check power lug + PDH fuse |
| Orange/Cyan blink | Dead controller | Swap the Spark MAX |
| Cyan solid, no spin | Signal OK, not commutating | Hall cable or phase wires |
| Green, no spin | Mechanical bind / phase wire | Check phases + mechanism |
| Dark/off | Corrupt firmware | Recover in REV Hardware Client |

4. **Fastest isolation:** USB into the Spark MAX → REV Hardware Client → **Run**. Spins on USB but
   not from code = CAN ID (tell me the real ID, I change `kHoodAngleNeo`). Won't spin on USB =
   hardware (hall/phase/power).
5. Once it spins: jog to the **closed** stop, press `Hood/ZeroNow`, confirm `Hood/Zeroed = true`.

## C. Pre-match checklist (EVERY match — no service tomorrow)
- [ ] Right code deployed (`Build/Version`).
- [ ] Hood jogs with RT/LT, then zeroed at closed (`Hood/Zeroed = true`).
- [ ] Odometry shows: `Field` widget + `Drive/PoseX/Y` have values.
- [ ] Shooter kill switches `Shooter/RightShooterOn` + `MiddleShooterOn` + `LeftShooterOn` all **TRUE**.
- [ ] Auto chooser shows your **real auto** (not "Do Nothing").
- [ ] **See an AprilTag while disabled** before the match → `Vision/Fusion/Seeded = true`
      (anchors field heading; avoids a pose jump in auto).
- [ ] `Robot/CANUtil` well under 100. `Robot/BatteryV` healthy (> 12 V at rest).
- [ ] Fresh battery, firmly strapped.

## D. Controller map (one Xbox, port 0)
| Control | Action |
|---|---|
| Left stick | Field-relative drive (alliance-aware) |
| Right stick X | Rotate; overrides auto-aim when a lock is on |
| Start + Back | Reset gyro |
| RT / LT (hold) | Hood jog open / close |
| POV Up (toggle) | Distance shot: RPM + hood from the table |
| POV Down | Stop shooter |
| POV Right (toggle) | **Up-close hub shot** (preset hood + RPM) |
| POV Left | Intake jerk (settle balls) |
| Left stick click | Shoot-on-move (face virtual goal + follow) |
| Right stick click | Face-hub rotation lock |
| X (toggle) | Transport conveyor (feed) |
| A (hold) | Transport reverse |
| Right bumper | Toggle intake open/close |
| B (toggle) | Manual shooter RPM test |

## E. Shooting, simply
- **Right next to hub:** press **POV-Right** (spins up + sets steep hood), then **X** to feed.
  Tune `Hub/PresetHoodRot` + `Hub/PresetRpm` live if it misses.
- **From distance:** press **POV-Up** — it reads `Drive/DistToHubM` and sets RPM + hood from the
  table automatically. Watch `Robot/ReadyToShoot` (rumble + green), then **X** to feed.
- **Calibrate at practice:** park at a measured distance, find the RPM/hood that scores
  (B + RT/LT), type them into `ShotCal/RPMn` + `ShotCal/Hoodn`. See TUNING.md.

## F. If something breaks mid-event
- **Robot disables / e-stops:** check `Robot/BrownedOut` + DS log for voltage dips. Shooter
  current already lowered (50 A + ramp). Swap battery.
- **Shots inconsistent:** confirm hood zeroed, `Robot/ReadyToShoot` true before feeding, and the
  `ShotCal/*` values match TUNING.md (someone may have dragged a slider).
- **Auto does nothing:** chooser showing "Do Nothing" → `deploy/pathplanner/settings.json` not
  deployed; re-deploy. See AUTONOMOUS.md.
- **Odometry gone again:** `SwerveDriveTelemetry.verbosity` must be `POSE` (it is).
