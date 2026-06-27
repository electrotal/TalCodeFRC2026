# Deploy-day commissioning checklist

Robot **on blocks or in clear space**, Driver Station + Elastic connected, **e-stop within reach**.
Do the steps in order — each gates the next. If a **STOP** condition hits, don't push past it.

> Note: hood **PID gains, MaxOut, OpenLimitRot** live in the **Preferences** NetworkTable (persistent),
> not under `SmartDashboard/`. Everything else (`Hood/ZeroNow`, `Hood/GoToTune`, etc.) is `SmartDashboard/`.

## STEP 0 — Deploy
Deploy from VS Code (WPILib: Deploy Robot Code) or `./gradlew deploy`. Confirm the DS console prints
`>>> CODE VERSION: Tal-Main ...`.

## STEP 1 — Reliability first (the whole point)
1. Enable **Teleop**, hands off, let it sit **3–5 minutes**.
2. `Robot/CANUtil` should be **well under 100%** (ideally <50%) and steady.
3. It must **stay enabled** — no auto-disable, no flood of "Loop time overrun" in the DS console.
- Pass -> continue. **STOP** if CAN is ~100% or it disables itself.

## STEP 2 — Hood encoder reliability (highest risk)
**Disabled.** Move the hood by hand through its range, watch `Hood/RawEncoder`.
- Should change **smoothly and repeatably**, no wild jumps/freezes.
- Smooth -> continue. **STOP** if jumpy/frozen — the absolute through-bore is still flaky; don't run hood PID.

## STEP 3 — Zero + direction + limits
1. **Enabled**, jog with **LT** to the **fully closed** hard stop (gentle).
2. Press **`Hood/ZeroNow`**. Confirm `Hood/HoodRot ~= 0`, `Hood/Zeroed = true`.
3. Jog **open** (**RT**). `Hood/HoodRot` must **increase**. If it decreases -> flip `kHoodInverted`
   (or `kAbsEncoderInverted`) in `Constants`, redeploy, redo.
4. Jog to **full open**, read `Hood/HoodRot`, type it into **`Hood/OpenLimitRot`** (Preferences).

## STEP 4 — Hood PID, gently
1. Start with `Hood/MaxOut` low (~**0.2**).
2. Type a small target in `Hood/TuneTargetRot` (e.g. **0.1**), flip **`Hood/GoToTune = true`**.
3. Hood moves slowly/smoothly to it; `Hood/AtAngle` -> true. Watch `Hood/HoodRot` vs `Hood/TargetRot`.
4. Raise `Hood/kP` gradually, add a little `Hood/kD` if it oscillates. Sweep the target across the range.
   Then `Hood/GoToTune = false`. Keep a hand near e-stop the first time.

## STEP 5 — Shooter
1. **B** spins to test RPM; press **B** again -> wheels **coast** down freely.
2. Watch `Shooter/TopRPM/MidRPM/BottomRPM` reach target, `Shooter/Ready` true. Tune `Shooter/PID/kV` then `kP`.

## STEP 6 — Ready indicator
Shooter spun up **and** hood at a commanded angle -> controller **rumble** + `Robot/ReadyToShoot` true.
Only when **both** hold. Drop either -> it stops.

## STEP 7 — Drive / gyro / alliance
1. Set **alliance color** in the DS (for real — the lock-on flip depends on it).
2. **Start+Back** resets gyro. Left stick **forward** -> robot drives **away from you**. Confirm on **both** Red and Blue.
3. Hold **right-stick-click** (face hub) and drive: translation stays field-oriented (not robot-centric)
   while it rotates to the hub. Nudge **right stick X** -> your rotation **overrides** auto-aim.

## STEP 8 — Vision (heading stays gyro)
Point the limelight at a tag. `Vision/Fusion/RejectReason` -> `SEEDED` then `ACCEPTED`; `Drive/PoseX/PoseY`
snap onto the field. **`Drive/HeadingDeg` must NOT jump** when a tag appears. Reset gyro, then show a tag:
forward should **not** drift.

## STEP 9 — Distance shot calibration (needs the field)
1. Park at a tape-measured distance; confirm `Shot/DistanceM` matches reality (fix vision first if not).
2. Manual RPM (**B**) + hood (`Hood/GoToTune`) until it scores. Read the working RPM and `Hood/HoodRot`.
3. Enter `(distance, hood, RPM)` into `ShotCal/Dist*_M`, `ShotCal/RPM*`, `ShotCal/Hood*`.
4. Repeat at **4 spread distances** (~1.3 / 2.5 / 3.5 / 4.2 m), ascending. **POV-Up** then auto-tracks;
   verify at an in-between distance and nudge the nearest points.

## STEP 10 — Intake + transport
**POV-Left** -> intake jerks fwd/back (tune `FeedConstants`). **X** conveyor, **A** reverse, **RB** intake toggle.

## STEP 11 — Auto sanity
In PathPlanner, drop named commands (`OpenIntake`, `Shoot`, `PrepareFromDistance`, `PrepareAtHub`, `StopAll`)
on a short test path. Run in **Autonomous**, watch it sequence.

## STEP 12 — Lock in your numbers
Preferences (hood gains/MaxOut/OpenLimitRot) and `ShotCal/*` persist on the roboRIO. To survive a **reimage**,
copy the dialed-in values into `Constants.java` and commit.

---

### Button map (one controller, port 0)
| Control | Action |
|---|---|
| Left stick | Drive (field, alliance-aware) |
| Right stick X | Rotate; overrides auto-aim when locked |
| Start+Back | Reset gyro |
| X | Toggle conveyor · **A** reverse conveyor |
| Right bumper | Toggle intake open/close · **Left bumper** pivot-hold · **Y** clopen |
| B | Toggle shooter test RPM |
| LT / RT | Hood jog down / up |
| Left stick click | Shoot-on-move (virtual goal + RPM/hood) |
| Right stick click | Face-hub rotation lock |
| POV Up | Distance follow (RPM + hood) |
| POV Down | Stop shooter |
| POV Right | Hub preset (min angle + set RPM) |
| POV Left | Intake jerk |
| `Hood/ZeroNow` (dash) | Zero hood at closed stop |
| `Hood/GoToTune` + `Hood/TuneTargetRot` (dash) | Drive hood to a typed angle |
