# TUNING — every value you can change, in one place

Two ways to change a value:
- **Dashboard (live):** type it on Elastic under the key shown. Takes effect immediately. **Resets
  on reboot** unless you also copy it into the constant.
- **Constant (permanent):** edit `src/main/java/frc/robot/Constants.java` and re-deploy.

After the practice match, copy your good dashboard values into the constants so they survive.

---

## HOOD  (Constants.HoodConstants)
| What | Dashboard key | Constant | Default | Notes |
|---|---|---|---|---|
| PID P | `Hood/kP` | `kP` | 6.0 | Raise for snappier; lower if it oscillates |
| PID I | `Hood/kI` | `kI` | 0.0 | Usually leave 0 |
| PID D | `Hood/kD` | `kD` | 0.2 | Add if it oscillates around target |
| Max PID output | `Hood/MaxOut` | `kMaxOut` | 0.30 | Gentleness cap (duty) |
| Jog strength | `Hood/JogPercent` | — | 0.20 | LT/RT power; raise if hood is too weak |
| Open limit | `Hood/OpenLimitRot` | `kOpenHoodRot` | 0.07 | Max hood travel (your measured range) |
| Tolerance | — | `kToleranceHoodRot` | 0.005 | "At angle" window |
| Slew per loop | — | `kSlewPerLoop` | 0.04 | PID smoothness |
| Encoder invert | — | `kAbsEncoderInverted` | false | Flip if Hood/HoodRot goes DOWN when opening |
| Test target | `Hood/TunePercent` + `Hood/GoToTune` | — | — | Drive to % of range to test |

## SHOOTER  (Constants.ShooterConstants)
| What | Dashboard key | Constant | Default | Notes |
|---|---|---|---|---|
| Velocity P | `Shooter/PID/kP` | `kVelocityP` | 1.0 | |
| Velocity kV | `Shooter/PID/kV` | `kVelocityV` | 0.12 | Feedforward — tune first |
| Velocity kS | `Shooter/PID/kS` | `kVelocityS` | 0.0 | |
| Test RPM (B button) | `Shooter/ToggleHighRpm` | `kToggleTestHighRpm` | 5000 | |
| Default RPM | — | `kTopRpm/kMidRpm/kBottomRpm` | 4500 | Used by SpinUpShooterDefault |
| Per-wheel ON/OFF | `Shooter/RightShooterOn` etc. | — | true | **Keep all TRUE for matches** |

## UP-CLOSE / HUB SHOT  (POV-Right, and ShootAtHub in auto)
| What | Dashboard key | Constant | Default | Notes |
|---|---|---|---|---|
| Hub hood angle | `Hub/PresetHoodRot` | `kHubPresetHoodRot` | 0.0166 | Steep, point-blank |
| Hub RPM | `Hub/PresetRpm` | `kHubPresetRpm` | 2800 | |

## DISTANCE SHOT TABLE  (Constants.ShotLookup) — the calibration
Each row = one distance point. Interpolated between, clamped outside. From WCP, scaled to our hood.
| Row | Distance key | Hood key | RPM key | Default dist (m) | Default hood | Default RPM |
|---|---|---|---|---|---|---|
| 1 (close) | `ShotCal/Dist1_M` | `ShotCal/Hood1` | `ShotCal/RPM1` | 1.32 | 0.0166 | 2800 |
| 2 (mid) | `ShotCal/Dist2_M` | `ShotCal/Hood2` | `ShotCal/RPM2` | 2.91 | 0.0359 | 3275 |
| 3 (far) | `ShotCal/Dist3_M` | `ShotCal/Hood3` | `ShotCal/RPM3` | 4.20 | 0.0433 | 3650 |
*(To add a 4th row, add a value to each array in `Constants.ShotLookup` — the code handles any count.)*

## INTAKE  (Constants.IntakeConstants)
| What | Dashboard key | Constant | Default |
|---|---|---|---|
| Pivot P/I/D | `Intake/P` `Intake/I` `Intake/D` | `kPivotP/I/D` | 1.2 / 0 / 0.03 |
| Open / Closed / Clopen | `Intake/OpenRot` etc. | `kOpenPivotRot` etc. | 0.36 / 0.78 / 0.55 |
| Roller % | `Intake/RollerPercent` | `kRollerPercent` | 0.80 |
| Jerk half-clopen | — | `kJerkHalfClopenRot` | 0.45 |
| Jerk dwell (s) | — | `kJerkDwellSeconds` | 0.2 |

## TRANSPORT  (Constants.TransportConstants)
| What | Dashboard key | Default |
|---|---|---|
| Conveyor % | `Transport/TransportPercent` | 0.70 |
| Feeder % | `Transport/FeederPercent` | 0.85 |

## DRIVE  (Constants)
| What | Constant | Default | Notes |
|---|---|---|---|
| Max speed | `maxSpeed` | 13.05 ft/s | Was 14.5; change the number to restore |
| Hold-heading PID | `SwerveConstants.kHoldHeadingP/D` | 4.0 / 0.15 | Auto-aim tightness |
