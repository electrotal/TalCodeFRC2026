# KEYBINDS — one Xbox controller (port 0)

Verified against `RobotContainer.java`. **toggle** = press once on, press again off. **hold** = only
while held. **press** = one-shot.

## Driving (always active)
| Control | Action |
|---|---|
| Left stick | Drive — field-relative, alliance-aware |
| Right stick X | Rotate; **overrides auto-aim** when a lock is on |
| Start **+** Back (together) | Reset gyro |

## Shooting
| Control | Type | Action |
|---|---|---|
| POV Up | **toggle** | Distance shot — RPM **+** hood from the table (holds until off) |
| POV Right | **toggle** | Up-close hub shot — fixed hood + RPM (point-blank) |
| POV Down | press | Stop shooter (ends distance/hub) |
| Left stick click | **toggle** | Shoot-on-move — face virtual goal + RPM/hood follow |
| Right stick click | **toggle** | Face-hub rotation lock (aim only; you feed) |
| B | **toggle** | Manual shooter test RPM (`Shooter/ToggleHighRpm`) |
| X | **toggle** | Transport conveyor = **feed** (press again to stop) |
| A | hold | Transport **reverse** |

## Hood
| Control | Type | Action |
|---|---|---|
| RT | hold | Jog **open** (+) |
| LT | hold | Jog **close** (−) |

## Intake
| Control | Type | Action |
|---|---|---|
| Right bumper | press | Toggle intake open/close |
| Left bumper | press | Toggle pivot-hold |
| Y | press | Clopen (half position) |
| POV Left | hold | Intake jerk — wiggle pivot to shake balls in (wheels off) |

## Dashboard buttons (Elastic, not the controller)
| Key | Action |
|---|---|
| `Hood/ZeroNow` | Zero hood at the closed stop |
| `Hood/GoToTune` + `Hood/TunePercent` | Drive hood to a % of its range |

## Feedback
Controller **rumbles** and `Robot/ReadyToShoot` goes **green** only when BOTH shooter RPM and hood
angle are on target.

## Basic shooting flow
1. **POV-Right** (close) or **POV-Up** (distance) → spins up + sets hood.
2. Wait for **rumble / green**.
3. **X** to feed. Press **X** again or **POV-Down** to stop.

> Remember: X, POV-Up, POV-Right, B, and the stick-clicks are **toggles** — they stay on until you
> press them again.
