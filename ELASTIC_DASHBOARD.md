# Elastic dashboard — what to add (and what to ignore)

All keys live under **SmartDashboard/** unless noted. Add a widget by dragging the key from the
NetworkTables tree in Elastic. Build your layout once, then save it.

## Tab 1 — DRIVE / MATCH (always visible)
| Widget | Key | Type | Why |
|---|---|---|---|
| Field / odometry | `SmartDashboard/Field` (the YAGSL Field2d) | Field2d | See robot position on the field |
| Pose X / Y | `Drive/PoseX`, `Drive/PoseY` | number | Backup numeric odometry |
| Heading | `Drive/HeadingDeg` | number | Robot angle (gyro) |
| Distance to hub | `Drive/DistToHubM` | number | For distance shots |
| Battery | `Robot/BatteryV` | number/voltage | Watch for brownout |
| CAN util | `Robot/CANUtil` | number | Must stay well under 100 |
| Browned out | `Robot/BrownedOut` | boolean | Red = power problem |
| **READY TO SHOOT** | `Robot/ReadyToShoot` | boolean (big green) | Shooter RPM AND hood both ready |
| Build version | `Build/Version`, `Build/DeployTime` | string | Confirm the right code is deployed |

## Tab 2 — HOOD (bring-up + tuning)
| Widget | Key | Type | Why |
|---|---|---|---|
| Hood position | `Hood/HoodRot` | number | Your 0.00–0.07 reading |
| Hood % open | `Hood/PercentOpen` | number | 0–100 of the range |
| Hood target | `Hood/TargetRot` | number | Where the PID is aiming |
| At angle | `Hood/AtAngle` | boolean | True when on target |
| Zeroed | `Hood/Zeroed` | boolean | Must be true for accurate angles |
| **ZERO NOW** | `Hood/ZeroNow` | boolean (button) | Press at the closed stop to zero |
| **GO TO TUNE** | `Hood/GoToTune` | boolean (button) | Drive hood to TunePercent |
| **Tune percent** | `Hood/TunePercent` | number (slider 0–100) | Sweep the angle to test PID |
| DEBUG: motor cmd | `Hood/CmdOutput` | number | What we send the Spark MAX |
| DEBUG: motor applied | `Hood/MotorApplied` | number | What the Spark MAX actually does |
| DEBUG: manual | `Hood/ManualPercent` | number | -1 LT / 0 / +1 RT |

## Tab 3 — SHOOTER
| Widget | Key | Type | Why |
|---|---|---|---|
| Top / Mid / Bottom RPM | `Shooter/TopRPM`, `Shooter/MidRPM`, `Shooter/BottomRPM` | number | Actual wheel speeds |
| Shooter ready | `Shooter/Ready` | boolean | At target RPM |
| Kill switches | `Shooter/RightShooterOn`, `MiddleShooterOn`, `LeftShooterOn` | boolean (button) | **All must be TRUE** |
| Shot distance | `Shot/DistanceMeters` | number | Distance used for the auto shot |
| Shot target hood / rpm | `Shot/TargetHoodRot`, `Shot/TargetRpm` | number | What the table picked |

## Tab 4 — VISION
| Widget | Key | Type | Why |
|---|---|---|---|
| Limelight connected | `Vision/LimelightConnected` | boolean | Camera alive |
| Has target | `Vision/HasTarget` | boolean | Sees a tag now |
| Tag count | `Vision/TagCount` | number | How many tags |
| Fusion seeded | `Vision/Fusion/Seeded` | boolean | Field heading anchored (see a tag while disabled!) |
| Reject reason | `Vision/Fusion/RejectReason` | string | Why vision was/ wasn't used |

## Ignore / don't bother adding
- All the per-tunable PID keys go on the **TUNING** tab (see TUNING.md), not here.
- `Intake/*` detail beyond `Intake/State` — only needed if debugging the intake.
- Anything not listed above is internal; it won't help during a match.

> Tip: make the **READY TO SHOOT** and **CAN util** widgets large on the driver tab. Those two tell
> the driver "can I shoot" and "is the robot healthy."
