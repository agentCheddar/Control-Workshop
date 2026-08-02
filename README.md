# FRC Control Workshop

Teaching code for a two-mechanism control-and-tuning workshop. Built for **WPILib 2026**,
**Phoenix 6 (26.3.0)** Kraken X60 motors on a **CANivore**, tuned live in **Elastic** and logged with
**AdvantageKit** for review in **AdvantageScope**.

> **Demo 1 (this repo): Bang-Bang control of the linear extension.** The pivot is scaffolded and
> ready for a later demo.

---

## 1. Hardware this code assumes

| Item | Detail |
| --- | --- |
| Power | 12 V battery → breaker → **REV PDH** → roboRIO 2.0, both Krakens |
| Controller | roboRIO 2.0, VH-109 radio over Ethernet |
| CAN | **CTRE CANivore** bus (named `canivore`) carrying both motors |
| Mechanism A | Linear extension — 1 Kraken X60, **30:12 (2.5:1)**, rack & **2.54 cm (1") pitch-dia pinion**, travel **0–29.21 cm** (11.5") |
| Mechanism B | Pivot — 1 Kraken X60, **72/14·5·5 (~128.57:1)**, zero at horizontal, range **0–100° CCW** |

### CAN map — set these in Phoenix Tuner X to match, or edit `Constants.java`

| Device | CAN ID | Bus |
| --- | --- | --- |
| Linear extension Kraken | **20** | `canivore` |
| Pivot Kraken | **21** | `canivore` |

If your CANivore has a different name, change `Constants.kCANBus`. Use `new CANBus("*")` to grab the
first CANivore found.

---

## 2. One-time setup

1. **Install the WPILib 2026 VS Code** bundle (it brings its own JDK 17 + Gradle — you do **not**
   need a system Java or Gradle). Open this folder in it.
2. **Set your team number.** Edit [`.wpilib/wpilib_preferences.json`](.wpilib/wpilib_preferences.json)
   and change `"teamNumber": -1` to your number. Deploys will fail with a clear error until you do.
3. Vendor libraries (Phoenix 6, AdvantageKit, WPILib New Commands) are **already vendored** in
   [`vendordeps/`](vendordeps) — nothing to install.
4. **(For `.wpilog` logging)** plug a **FAT32-formatted USB stick** into the roboRIO. Live logging over
   NetworkTables works without it; the USB stick just adds an on-robot file to replay later.

### Deploy (from this Mac)

```bash
./gradlew deploy
```

Or use the WPILib command palette → **"WPILib: Deploy Robot Code"**. Build without deploying:

```bash
./gradlew build
```

---

## 3. Demos — three controllers on the linear extension

The linear extension carries **three controllers** the students compare, plus an OFF mode. Exactly
one runs at a time; you switch between them with buttons on Elastic. **All drive to the same target**
(the `Target_Cm` tunable). All the logic lives in
[`LinearExtension.java`](src/main/java/frc/robot/subsystems/LinearExtension.java): `periodic()` picks
the active mode, and `bangBangVolts()` / `pidVolts()` / `profileVolts()` are the algorithms side by
side.

### Demo 1 — Bang-Bang

Each 20 ms loop it looks at whether the mechanism is below or above the target and applies a **fixed
voltage** in that direction — nothing in between.

```
error = target - position
|error| ≤ deadband   →   0 V        (close enough)
error > 0            →   +Kv V      (below target → extend)
error < 0            →   −Kv V      (above target → retract)
```

### Demo 2 — PID

A proportional-integral-derivative controller eases in as the error shrinks, instead of slamming a
fixed voltage:

```
output = kP·error + kI·∫error dt + kD·d(error)/dt      (then clamped to ±PID_MaxVolts)
```

Students tune **kP, kI, kD** and the **max voltage** live. Start P-only (kI = kD = 0), raise kP until
it's responsive, then add a little kD to tame overshoot. The output is clamped to the tunable
`PID_MaxVolts` (never above the 0–6 V hardware ceiling).

### Demo 3 — Motion profile + feedforward + path-following PID

A **trapezoidal motion profile** generates a smooth, moving setpoint — position ramps up to a max
velocity, cruises, then decelerates into the goal (bounded by a max acceleration). A **feedforward**
predicts the voltage for that motion, and a **PID corrects the error against the profile setpoint**
(the moving point on the path — *not* the final goal), so it tracks the whole trajectory:

```
volts = [ kS·sign(v) + kV·v + kA·a ]  +  PID(profileSetpoint − position)   [+ kG added globally]
        └──────── feedforward ───────┘    └──── path-following feedback ────┘
        (profile part clamped to ±Profile_MaxVolts; then kG is added and the total clamped to ±6 V)
        kG is applied in EVERY enabled mode, so it's shown separate from the profile feedforward.
```

This is the best-tracking controller of the three: the feedforward does the bulk of the work and the
PID cleans up whatever it misses. Because the reference is the *profile* setpoint (typically a
a few mm away), the PID stays small and gentle — it corrects path error, not the whole
distance to the goal. Students tune **kS, kG, kV, kA**, the **max velocity / acceleration**, the **max
voltage**, and the path-following **kP / kI / kD** live. Tune the feedforward first (kG for gravity —
0 if horizontal — then kV, a small kS, kA last) so `Profile_FeedbackVolts` stays near zero, then add
kP to pull the actual position onto the profile line.

### Choosing a controller (Elastic buttons)

On boot the controller is **OFF** — nothing moves until you pick one. Four command buttons switch
modes (selecting one disables the others):

- **`LinearExtension/Use Bang-Bang`**
- **`LinearExtension/Use PID`** (resets the PID integral each time you enter PID mode)
- **`LinearExtension/Use Motion Profile`** (restarts the profile from the current position)
- **`LinearExtension/Disable Controller`** (back to OFF / 0 V)

The current choice is shown as text at `LinearExtension/ActiveController`.

### Running it

1. Deploy, open **Elastic**, connect to the robot.
2. Open the Driver Station and **Enable** (Teleop). *(Motors only move while enabled — that's the FRC
   safety interlock, not a bug.)*
3. Click **Use Bang-Bang**, **Use PID**, or **Use Motion Profile**, then change `Target_Cm` and
   tune.

### What students tune in Elastic

These live under the NetworkTables `Tuning/LinearExtension/` table (drag each onto the layout as a
text/number widget or slider; editing takes effect immediately, no redeploy):

| NT key | Meaning | Default |
| --- | --- | --- |
| `Tuning/LinearExtension/Target_Cm` | **Shared** setpoint both controllers drive to (clamped 0–29.21 cm) | 10.16 |
| `Tuning/LinearExtension/Kv_Volts` | Demo 1: fixed drive voltage magnitude. **Clamped to 0–6 V.** | 2.0 |
| `Tuning/LinearExtension/Deadband_Cm` | Demo 1: how close to the target counts as "there" | 0.635 |
| `Tuning/LinearExtension/kP` | Demo 2: proportional gain (volts per cm of error) | 0.3937 |
| `Tuning/LinearExtension/kI` | Demo 2: integral gain | 0.0 |
| `Tuning/LinearExtension/kD` | Demo 2: derivative gain | 0.0 |
| `Tuning/LinearExtension/PID_MaxVolts` | Demo 2: output voltage cap. **Clamped to 0–6 V.** | 4.0 |
| `Tuning/LinearExtension/kS` | Demo 3: static-friction feedforward (V) | 0.1 |
| `Tuning/LinearExtension/kG` | Gravity feedforward (V) — applied in **every enabled mode**, not just Demo 3 (0 if horizontal) | 0.0 |
| `Tuning/LinearExtension/kV` | Demo 3: velocity feedforward (V per cm/s) | 0.19685 |
| `Tuning/LinearExtension/kA` | Demo 3: acceleration feedforward (V per cm/s²) | 0.0 |
| `Tuning/LinearExtension/Profile_MaxVel_CmPerSec` | Demo 3: profile top speed (cm/s) | 25.4 |
| `Tuning/LinearExtension/Profile_MaxAccel_CmPerSec2` | Demo 3: profile acceleration (cm/s²) | 50.8 |
| `Tuning/LinearExtension/Profile_MaxVolts` | Demo 3: output voltage cap. **Clamped to 0–6 V.** | 6.0 |
| `Tuning/LinearExtension/Profile_kP` | Demo 3: path-following gain (V per cm of *path* error) | 0.3937 |
| `Tuning/LinearExtension/Profile_kI` | Demo 3: path-following integral gain | 0.0 |
| `Tuning/LinearExtension/Profile_kD` | Demo 3: path-following derivative gain | 0.0 |
| `Tuning/LinearExtension/Settle_Tolerance` | Settle timer: "arrived" band for both position (cm) and speed (cm/s) | 0.635 |

> `Kv_Volts` is labeled **Kv** per the workshop plan. Note that "Kv" normally means a velocity
> feedforward gain — here it is simply the fixed bang-bang voltage. The code comments call this out
> so students aren't misled.

### The buttons

Elastic shows these command buttons (published from `RobotContainer`):

- **`LinearExtension/Use Bang-Bang`** / **`Use PID`** / **`Use Motion Profile`** / **`Disable
  Controller`** — pick the active controller (works while **disabled**, so you can pre-select before
  enabling).
- **`LinearExtension/Zero`** — sets the current position as 0". Retract the mechanism by hand first,
  then click (works while **disabled**).
- **`Pivot/Zero`** — same idea, defines the current pose as horizontal (0°).

### Timing the controllers (how fast is each one?)

A settle stopwatch runs for whichever controller is active:

- **Starts** the moment you change `Target_Cm` (a new commanded move).
- **Stops** once the mechanism is within `Settle_Tolerance` of the target **and** its speed is within
  that same tolerance of zero — i.e. it actually arrived and stopped, not just blew past.
- **`Timer_LastSettleSec`** is that move's time; **`Timer_AverageSettleSec`** averages all moves since
  the last disable (disabling the robot resets the average).

To compare controllers fairly: pick a controller, change the target, read the time; switch to the
next controller, change the target again, compare. Watch `Timer_ElapsedSec` count up live and freeze
at the settle time. *(Bang-bang may never satisfy the velocity part of the tolerance because it
chatters — that's a real result: it doesn't truly settle. Loosen `Settle_Tolerance` if you want it to
register.)*

Only target **changes** start the timer (per the workshop design), and only while a controller is
active (not OFF) — so a controller switch alone won't restart it. Nudge the target to time a fresh run.

### The tuning lesson

**Bang-bang:**
- **Kv too low** → can't overcome friction/gravity, never reaches target.
- **Kv too high** → overshoots, then chatters/buzzes rapidly across the target (bang-bang's signature).
- **Deadband too small** → constant buzzing at the target. **Too large** → stops noticeably short.

**PID:**
- **kP too low** → sluggish, stops short. **Too high** → overshoot and oscillation.
- **kD** → damps overshoot; too much makes it jittery/noisy.
- **kI** → erases steady-state error (the last sliver), but too much causes slow
  oscillation / windup.

**Motion profile + feedforward + PID:**
- **kV too low** → lags behind the profile (actual trails `Profile_SetpointCm`). **Too high** →
  runs ahead / overshoots.
- **kG** → if it drifts down at rest, raise kG; if it creeps up, lower it.
- **kS** → just enough to break static friction; too much makes it lurch off the start.
- **Max velocity / acceleration** → shape of the ramp. Lower = gentler and easier to follow.
- **Path-following kP/kI/kD** → tune the feedforward FIRST (so `Profile_FeedbackVolts` sits near 0),
  then raise kP to snap the actual position onto the profile line; kD damps, kI erases the last bit
  of steady-state error. Since it corrects against the *moving* setpoint, a little goes a long way.

**The payoff:** switch between all three on the same target and watch `PositionCm` vs.
`TargetCm` (and `Profile_SetpointCm` for Demo 3). Bang-bang chatters; a tuned PID glides in
reactively; the motion profile follows a planned trajectory (feedforward predicting the motion, a
small PID correcting the path). That progression is the whole point of the workshop.

---

## 4. Logging → AdvantageScope

AdvantageKit records everything below to a `.wpilog` (USB) **and** streams it live over NetworkTables.

**View live:** open AdvantageScope → connect to the robot → set live source to
**"NetworkTables 4 (AdvantageKit)"**. Tunables appear under `Tuning/`; logged outputs under
`RealOutputs/LinearExtension/` (and `.../Pivot/`).

**View after a match/run:** copy the `.wpilog` off the USB stick and open it in AdvantageScope.

### Logged signals

| Linear extension (`RealOutputs/LinearExtension/`) | |
| --- | --- |
| `ControlMode` | which controller is active (`OFF` / `BANG_BANG` / `PID` / `MOTION_PROFILE`) |
| `PositionCm`, `TargetCm`, `ErrorCm` | plot these three together |
| `MotorRotations` | raw rotor turns (before the 30:12) vs. `PositionCm` shows the gearing |
| `CommandedVolts`, `GravityVolts`, `AppliedVolts` | total command (incl. gravity), the always-on kG term, and what the motor did |
| `ClampedKv`, `DeadbandCm` | Demo 1 (bang-bang) values in effect |
| `PID_RawVolts`, `PID_MaxVolts` | Demo 2: PID output before clamping, and the cap |
| `Profile_SetpointCm`, `Profile_SetpointVelCmPerSec`, `Profile_GoalCm`, `Profile_MaxVolts` | Demo 3: the moving profile target (overlay on `PositionCm`) and the output cap |
| `Profile_PathErrorCm`, `Profile_FeedforwardVolts`, `Profile_FeedbackVolts` | Demo 3: path error the PID sees, and the feedforward-vs-feedback voltage split |
| `Timer_Running`, `Timer_ElapsedSec` | settle stopwatch: whether it's timing, and the live/last elapsed |
| `Timer_LastSettleSec`, `Timer_AverageSettleSec`, `Timer_SettleCount` | most-recent settle time, running average (resets on disable), and sample count |
| `SettleTolerance` | the "arrived" band in effect |
| `VelocityCmPerSec`, `StatorCurrentAmps`, `SupplyCurrentAmps` | motor-side vs. battery-side current |

The **pivot** logs the same telemetry in degrees (`PositionDegrees`, `MotorRotations`,
`VelocityDegreesPerSec`, `AppliedVolts`, `StatorCurrentAmps`, `SupplyCurrentAmps`) plus its controller
signals (`ControlMode`, `TargetDegrees`, `ErrorDegrees`, `CommandedVolts`, `PID_RawVolts`, and the
`Profile_*` family incl. `Profile_SetpointDegrees` / `Profile_FeedforwardVolts` / `Profile_FeedbackVolts`).

Plot `PositionCm` against `TargetCm` (and `Profile_SetpointCm` for Demo 3), then cycle
through **Use Bang-Bang**, **Use PID**, and **Use Motion Profile** on the same target — chatter vs. a
smooth reactive glide-in vs. following a planned profile is the money shot for the workshop.

### The pivot (Mechanism B) — PID + motion profile

The pivot carries two of the same controllers, adapted for an arm (buttons `Pivot/Use PID`,
`Pivot/Use Motion Profile`, `Pivot/Disable Controller`; shared target `Tuning/Pivot/Target_Degrees`,
0–100°, boots OFF):

- **PID** — pure feedback to the target angle. It has **no gravity term**, so it will sag by a
  gravity-dependent amount that changes with angle — that's the intended lesson (why arms need
  feedforward). Tunables: `Tuning/Pivot/kP`, `kI`, `kD`, `PID_MaxVolts`.
- **Motion profile + feedforward + PID** — a trapezoidal profile (deg/s) plus an **`ArmFeedforward`**
  whose gravity term is **kG·cos(angle)**, so it holds correctly at *every* angle (not just one, as a
  constant kG would). Then a path-following PID cleans up. Tunables: `kS`, `kG`, `kV`, `kA`,
  `Profile_MaxVel_DegPerSec`, `Profile_MaxAccel_DegPerSec2`, `Profile_MaxVolts`, `Profile_kP/kI/kD`.

Because zero is horizontal, the feedforward gets the angle directly (it's measured from horizontal).
Tune **kG first** — the voltage that holds the arm level — then kV, kS, and the path-following kP. The
gains are guesses; characterize them (SysId or the bang-bang trick) for real numbers. Units note: kV
is **V per deg/s** and the angle is converted to radians only for the gravity cos term.

The **settle timer** runs for the pivot too (`Tuning/Pivot/Settle_Tolerance`, default 1° / 1°/s; logs
`Pivot/Timer_LastSettleSec` and `Pivot/Timer_AverageSettleSec`, the average resetting on disable), so
you can time PID vs. motion-profile the same way as the extension.

---

## 5. Safety & gotchas (read before powering the mechanism)

- **Check the drive direction first.** With a low `Kv` (≈1 V) and a target above current position, the
  mechanism should **extend**. If it retracts, flip `LinearExtension.kInvert` in `Constants.java` to
  `Clockwise_Positive` (same for the pivot).
- **Soft limits** at 0 / 29.21 cm (and 0° / 100°) are enforced by the TalonFX as a backstop — but they
  are only correct **after** you've zeroed. Zero at a known pose before enabling.
- **Current limits** are set to 40 A (stator & supply). Lower them in `Constants.java` for a gentler
  demo.
- **All voltages are clamped 0–6 V in software** — bang-bang `Kv` and `PID_MaxVolts` both — so typing
  50 into Elastic still can't exceed 6 V. Raise `kMaxDriveVolts` in `Constants.java` for more.
- **Controller is OFF on boot** — nothing drives until you click a controller button.
- **Controllers don't run while disabled** — output holds 0 V and the PID integrator / motion-profile
  setpoint reset every disabled loop, so nothing winds up or lurches the moment you enable.
- **`kG` (gravity) is applied in every enabled mode** on the extension — even OFF — so it holds against
  gravity whenever enabled, not just under the profile. On enable the mechanism will apply `kG` V
  immediately; if `kG` is set too high it can creep upward, so tune it (the voltage that just holds it
  still). Leave `kG` at 0 if the extension is horizontal. *(This is the extension only; the pivot's
  gravity term still applies only under its profile controller.)*
- **Demo 3 has feedback** — the path-following PID corrects deviations, so it's far more forgiving of
  feedforward mistuning than pure feedforward would be (still capped at `Profile_MaxVolts` ≤ 6 V).
  Tune the feedforward first, then add kP; watch `Profile_SetpointCm` vs. `PositionCm`.
- **Coast is the default** neutral mode: the motor freewheels when a controller commands 0 V (or in
  OFF / while disabled), so students can move the mechanism by hand to zero it. It will **not** hold
  position on its own — set `kNeutralMode` to `Brake` per mechanism in `Constants.java` if you want it
  to hold (worth considering for the pivot so a heavy arm can't sag under gravity).

---

## 6. Project layout

```
src/main/java/frc/robot/
  Main.java              entry point
  Robot.java             LoggedRobot; AdvantageKit logging setup
  RobotContainer.java    creates subsystems, publishes controller + zero buttons
  Constants.java         CAN IDs, gear ratios, travel & safety limits, controller defaults
  subsystems/
    LinearExtension.java  ← Demos 1–3: bang-bang + PID + motion profile, mode switching, tunables, logging
    Pivot.java            PID + motion profile (ArmFeedforward, gravity = kG·cos θ), mode switching, logging
```

## 7. Next steps

- **Characterize the feedforward:** replace the guessed kS/kG/kV/kA (both mechanisms) with measured
  values via WPILib SysId (or the bang-bang trick in §3) — the tracking is only as good as those gains.
  The pivot's **kG** especially is a placeholder; measure the voltage that holds the arm level.
- **Optional parity:** add bang-bang and/or the settle timer to the pivot to fully match the extension.

---

*Versions: WPILib/GradleRIO 2026.2.1 · Phoenix 6 26.3.0 · AdvantageKit 26.0.2 · Gradle 8.11 · Java 17.*
