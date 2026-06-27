# Deploy checklist — SEE THE NEW DOCS

This file is superseded. The old version had stale keys (Preferences, `Hood/TuneTargetRot`) that no
longer match the code. Use these instead:

- **MATCH_DAY.md** — deploy steps, hood-motor LED fix, pre-match checklist, controller map, shooting.
- **ELASTIC_DASHBOARD.md** — exactly which dashboard widgets to add.
- **TUNING.md** — every value you can change (correct keys: `Hood/kP`, `Hood/TunePercent`,
  `Hood/OpenLimitRot`, `ShotCal/*`, `Hub/Preset*` — all under `SmartDashboard/`, not Preferences).
- **AUTONOMOUS.md** — draw a path, drop `ShootAtHub` / `PrepareFromDistance` / etc.

Bring-up order (details in MATCH_DAY.md):
1. Deploy, confirm `Build/Version`.
2. Reliability: enable Teleop 3–5 min, `Robot/CANUtil` < 100, stays enabled.
3. Hood by hand: `Hood/RawEncoder` (DIO 6) moves smoothly.
4. Jog LT to closed stop → `Hood/ZeroNow` → `Hood/Zeroed = true`. RT must make `Hood/HoodRot` increase.
5. Hood PID: `Hood/TunePercent` + `Hood/GoToTune`.
6. Shooter (B), ready indicator, drive/gyro/alliance, vision seed, distance calibration, auto.
