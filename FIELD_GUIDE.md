# FIELD GUIDE — competition, no programmer on site

One document. Self-contained. Branch to deploy: **`reliability-shooting-overhaul`**.
Keep e-stop reachable for every powered test.

---

# 1. THE MOMENT YOU'RE WITH THE ROBOT (do in order)

1. **Battery** in, strapped, > 12.3 V resting. Main breaker on.
2. Driver Station + Elastic connected (radio linked, DS says "Communications" green).
3. **Deploy** the branch: WPILib VS Code → "Deploy Robot Code". Wait for BUILD SUCCESSFUL + deploy done.
4. On Elastic confirm **`Build/Version`** + **`Build/DeployTime`** = the deploy you just did.
5. DS console: no red error spam.
6. Go to **Section 2** (hood motor) → then **Section 4** (pre-match checklist). Don't skip the hood test.

If you have zero time: do steps 1–5, then the **Quick pre-match** (Section 4 top 6 boxes), then play.

---

# 2. HOOD MOTOR — the one thing that needs you (full decision tree)

The code is verified to command the motor. If it doesn't spin, it's wiring/ID/power. Find which:

**Test:** Enable **Teleop**. Hold **RT**. Watch `Hood/CmdOutput` (should be ~0.1–0.2) and the
**Spark MAX status LED** on the hood motor.

### Case A — `Hood/CmdOutput` is 0 (not ~0.2)
- Controller not on port 0, or not in Teleop, or `Hood/ManualPercent` stays 0 → check the Xbox
  controller is plugged/recognized in the DS, robot is **Enabled Teleop**.
- If `Hood/CmdOutput` shows ~0.2 but motor dead → go to the LED table.

### Case B — LED table (CmdOutput is good, motor still dead)
| LED pattern | Meaning | DO THIS |
|---|---|---|
| **Orange/Magenta** blink | Sensor fault — **NEO hall cable** (MOST LIKELY) | Power off. Reseat the small ~6-pin ribbon at BOTH the Spark MAX sensor port and the NEO. Power on, retest. |
| **Orange/Yellow** blink | CAN fault / wrong ID | USB → REV Hardware Client → is there a device at **ID 26**? If different, see Case D. Check CAN wire + termination. |
| **Orange/Blue** blink | No 12 V to controller | Power off. Check the power lugs + the PDH/PDP fuse for that channel. |
| **Orange/Cyan** blink | Gate driver fault (controller damaged) | Swap the Spark MAX for a spare, set it to ID 26 + brushless, retest. |
| **Cyan solid**, no spin | Has signal, won't commutate | Hall cable (Case B row 1) or a phase wire (Case C). |
| **Green**, no spin | Commanded but stuck | Mechanical bind, or a loose phase wire (Case C). Move hood by hand — free? |
| **Dark / off** | Corrupt firmware / no power | USB → REV Hardware Client → update/recover firmware. Or no 12 V (Case Blue). |

### Case C — phase wires
If it hums/twitches but won't rotate: power off, tug-test the 3 fat motor leads + the sensor cable.
Re-seat any loose one.

### Case D — confirm / change the CAN ID
- USB into the Spark MAX → **REV Hardware Client** → read its **CAN ID**.
- If it IS 26 → ID is fine, problem is hall/power/phase above.
- If it's NOT 26 → either set the Spark MAX to **26** in Hardware Client (easiest), **or** change the
  code: `Constants.CanId.kHoodAngleNeo = <real id>`, redeploy. (Note: ID 26 is also the climber in
  code, but the climber isn't used — no conflict.)
- **Fastest motor test:** in Hardware Client, click **Run** to spin it over USB. Spins on USB but
  not from code → CAN/ID. Won't spin on USB → hardware (hall/phase/power).

### Once it spins
1. Jog **LT** gently to the **fully closed** stop.
2. Press **`Hood/ZeroNow`**. Confirm `Hood/HoodRot ≈ 0` and `Hood/Zeroed = true`.
3. Jog **RT** → `Hood/HoodRot` must **increase**. If it **decreases**, set
   `Constants.HoodConstants.kAbsEncoderInverted = true`, redeploy, redo.
4. Test PID: set `Hood/TunePercent` = 50, flip `Hood/GoToTune = true` → hood goes to mid, smoothly,
   `Hood/AtAngle` → true. Set `Hood/GoToTune = false` when done.

### If you CAN'T fix the hood at all
Fallback: you can still shoot. Use **LT/RT jog** to hand-set the hood angle per shot, and shoot with
manual RPM (B) + feed (X). Or rely on the point-blank hub shot with whatever fixed angle works.
The robot is fully drivable + can intake regardless of the hood.

---

# 3. POWER-ON HEALTH (30 seconds, disabled→enabled)
- `Robot/CANUtil` steady and **< 100** (ideally < 50).
- `Robot/BatteryV` healthy, `Robot/BrownedOut` = false.
- Move hood by hand (disabled): `Hood/RawEncoder` changes smoothly, no jumps.
- `Field` widget + `Drive/PoseX/Y` show numbers.
- Enable Teleop, hands off, ~1 min: **stays enabled**, no "Loop overrun" flood.

---

# 4. PRE-MATCH CHECKLIST (every match)
**Quick (do these 6 minimum):**
- [ ] Right code (`Build/Version`).
- [ ] Hood jogs + zeroed (`Hood/Zeroed = true`).
- [ ] Shooter switches `Shooter/RightShooterOn` + `MiddleShooterOn` + `LeftShooterOn` all **TRUE**.
- [ ] Auto chooser = your real auto (not "Do Nothing").
- [ ] See an AprilTag **while disabled** → `Vision/Fusion/Seeded = true`.
- [ ] Fresh battery strapped.

**Full:**
- [ ] `Robot/CANUtil` < 100, `Robot/BatteryV` good.
- [ ] Odometry (`Field`, `Drive/PoseX/Y`) live.
- [ ] Alliance color set correctly in the DS (drive + aim depend on it).
- [ ] Hood `OpenLimitRot` = 0.07 (or your measured value).
- [ ] `ShotCal/*` match your calibrated numbers (nobody dragged a slider).

---

# 5. SHOOTING

### Up close, next to the hub
- Press **POV-Right** (spins shooter + sets steep hood). Wait for **rumble / `Robot/ReadyToShoot`**.
- Press **X** to feed. Misses? Adjust `Hub/PresetRpm` (speed) and `Hub/PresetHoodRot` (arc) live.

### From a distance
- Press **POV-Up** (toggle). It reads `Drive/DistToHubM` → sets RPM + hood from the table.
- Wait for **rumble / green `Robot/ReadyToShoot`** → press **X** to feed. **POV-Down** stops.

### Shoot while moving
- **Left-stick click** (toggle): faces the virtual goal and follows RPM/hood. Drive, wait ready, feed (X).

### `Robot/ReadyToShoot` never turns true
- It needs BOTH shooter at speed AND hood at angle. Check `Shooter/Ready` and `Hood/AtAngle`
  separately to see which is failing. If hood, confirm it's zeroed + reaching target.

---

# 6. CALIBRATE THE SHOT TABLE (practice match)
Table now (from WCP): **1.32 m → hood 0.0166 / 2800 · 2.91 m → 0.0359 / 3275 · 4.20 m → 0.0433 / 3650.**
1. Park at a measured distance; confirm `Shot/DistanceMeters` ≈ reality (fix vision if way off).
2. Find a scoring shot: **B** (manual RPM, set `Shooter/ToggleHighRpm`) + **RT/LT** hood.
3. Read the working actual RPM + `Hood/HoodRot`. Type into `ShotCal/RPMn` + `ShotCal/Hoodn` +
   `ShotCal/DistN_M` for the nearest row.
4. Repeat at 2–3 distances (close / mid / far). Verify in between with POV-Up.
5. After: copy good values into `Constants.ShotLookup` so they survive a reboot/reimage.

---

# 7. AUTONOMOUS
- Pick the auto on the Elastic chooser. If only "Do Nothing" shows → `deploy/pathplanner/` (incl.
  `settings.json`) didn't deploy → redeploy.
- Build/edit in the **PathPlanner app**: draw start+end dots = a line the robot follows.
- Drop named commands: **`ShootAtHub`** (point-blank: spin + wait + feed, by itself),
  `PrepareFromDistance`, `Shoot`, `OpenIntake`/`CloseIntake`, `FeedShooter`, `StopAll`.
- Easiest scoring auto: start at the hub, auto = just **`ShootAtHub`**.
- See a tag while disabled first so the field pose is anchored.

---

# 8. FAILURE CASES (symptom → fix)
| Symptom | Likely cause | Do |
|---|---|---|
| Robot won't enable | DS comms / code crash | Check DS for code error; redeploy; reboot RIO |
| Disables itself / e-stops | Brownout (current spike) | Fresh battery; shooter current already capped 50 A + ramp; check `Robot/BrownedOut` + DS log |
| `Robot/CANUtil` ~100 | A device flooding / wiring | Reseat CAN; check for a duplicate CAN ID; reboot |
| Odometry gone from dash | telemetry verbosity | It's set to `POSE` (correct). If still gone, redeploy; check the `Field` widget binding |
| Pose jumps in auto | vision seeded mid-auto | See a tag **while disabled** before the match |
| Hood won't move | hall cable / ID / power | Section 2 LED tree |
| Hood won't hold angle | not zeroed | Zero at closed stop (`Hood/ZeroNow`) |
| Hood goes wrong way | inverted | flip `kAbsEncoderInverted`, redeploy |
| Shooter won't spin | kill switch / config | `Shooter/*ShooterOn` all TRUE |
| One shooter wheel dead | that wheel's switch / motor | toggle its `Shooter/*ShooterOn`; check that Kraken's LED |
| Ball won't feed | transport/intake | **X** conveyor; check intake not jammed; `POV-Left` jerk to settle |
| Drive feels mirrored | wrong alliance set | set alliance color in DS; reset gyro (Start+Back) |
| Auto does nothing | no path deployed | redeploy `deploy/pathplanner/`; reselect auto |
| Aim spins / overshoots | vision noise | nudge right stick to override; aim uses gyro heading, not raw camera |

---

# 9. CONTROLLER MAP (one Xbox, port 0)
| Control | Action |
|---|---|
| Left stick | Field drive (alliance-aware) |
| Right stick X | Rotate; overrides auto-aim when locked |
| Start + Back | Reset gyro |
| RT / LT (hold) | Hood jog open / close |
| POV Up | Distance shot (RPM + hood) · **POV Down** stop shooter |
| POV Right | Up-close hub shot · **POV Left** intake jerk |
| Left stick click | Shoot-on-move · **Right stick click** face-hub lock |
| X | Conveyor feed · **A** reverse conveyor |
| Right bumper | Intake open/close · **Left bumper** pivot-hold · **Y** clopen |
| B | Manual shooter RPM test |
| `Hood/ZeroNow` (dash) | Zero hood at closed stop |
| `Hood/TunePercent` + `Hood/GoToTune` (dash) | Drive hood to a % of range |

---

# 10. KEY DASHBOARD VALUES (glance during a match)
- **`Robot/ReadyToShoot`** (rumble+green) — can I shoot now.
- **`Robot/CANUtil`**, **`Robot/BatteryV`**, **`Robot/BrownedOut`** — robot health.
- **`Hood/Zeroed`**, **`Hood/HoodRot`** — hood sane.
- **`Drive/DistToHubM`** — shot distance.
- **`Shooter/Ready`**, **`Hood/AtAngle`** — which half of "ready" is missing.

Other docs in the repo: MATCH_DAY · ELASTIC_DASHBOARD · TUNING · AUTONOMOUS. This guide is the superset.
