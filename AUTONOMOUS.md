# AUTONOMOUS — draw a line, robot follows (+ shooting)

We use **PathPlanner**. You draw the path in the PathPlanner app, drop in commands (like "shoot"),
save, pick it on the dashboard. No coding needed for a new auto.

---

## One-time check
- The robot is already set up for PathPlanner (`AutoBuilder` + named commands are registered).
- Make sure the **PathPlanner** desktop app is installed and pointed at this project folder.
- File `deploy/pathplanner/settings.json` must deploy with the code (it does by default).

## Make a "drive a line" auto (5 steps)
1. Open the **PathPlanner** app → it loads this project.
2. **New Path** → name it (e.g. `LeaveLine`). On the field, drag the **start** dot and **end** dot
   to draw the line. Curve it by dragging the control handles if you want.
3. Set the **start pose rotation** (which way the robot faces) and the end rotation.
4. **New Auto** → name it (e.g. `Auto_LeaveLine`) → add your path to it. Done — that auto drives the line.
5. Deploy code (or just the deploy folder). On Elastic, the **auto chooser** now lists
   `Auto_LeaveLine`. Select it. It runs in autonomous.

## Add shooting to an auto
In the **Auto** editor, between/after paths you can drop **named commands** (or put them as
**event markers** on a path to fire mid-drive). Type the name EXACTLY:

| Command name | What it does |
|---|---|
| **`ShootAtHub`** | **Point-blank shot: holds preset hood + RPM, waits to spin up, then feeds. Use this right next to the hub.** |
| `PrepareFromDistance` | Continuously set hood + RPM from live distance to hub (aim while driving) |
| `PrepareAtHub` | Set the point-blank hood + RPM, but don't feed (you feed later) |
| `Shoot` | Spin up default RPM, wait, then feed |
| `OpenIntake` / `CloseIntake` / `ToggleIntake` | Intake pivot |
| `FeedShooter` | Run transport + intake jerk (the feed action) |
| `SpinUpShooterDefault` | Just spin the shooter |
| `StopShooter` / `StopAll` | Stop |
| `AimUnderHub` | Rotate to a fixed heading under the hub |

### Easiest scoring auto (next to the hub)
1. Start the robot touching/near the hub (set the auto's start pose there).
2. New Auto → add a **named command `ShootAtHub`** (no path needed if you don't move).
3. Optional: add a path after it to leave, plus `OpenIntake` to grab another, then another `ShootAtHub`.

That's it — `ShootAtHub` is the "just shoot from next to the hub" command you asked for. It spins
up, waits until at speed, and feeds, all on its own. Tune its values live with
`Hub/PresetHoodRot` + `Hub/PresetRpm` (see TUNING.md).

## Notes
- Auto distances/positions are blue-origin; the code mirrors for red automatically.
- If the chooser only shows **"Do Nothing"**, the PathPlanner files didn't deploy — re-deploy and
  confirm `deploy/pathplanner/` has your path + auto + `settings.json`.
- See an AprilTag while disabled before auto so the field pose is anchored (avoids a jump).
