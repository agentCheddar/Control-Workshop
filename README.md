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
| Mechanism A | Linear extension — 1 Kraken X60, **2.5:1**, **1"** spool, travel **0–11.5"** |
| Mechanism B | Pivot — 1 Kraken X60, **150:1**, zero at horizontal, range **0–100° CCW** |

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

## 3. Demos — two controllers on the linear extension

The linear extension carries **two controllers** the students compare, plus an OFF mode. Exactly one
runs at a time; you switch between them with buttons on Elastic. **Both drive to the same target**
(the `Target_Inches` tunable). All the logic lives in
[`LinearExtension.java`](src/main/java/frc/robot/subsystems/LinearExtension.java): `periodic()` picks
the active mode, and `bangBangVolts()` / `pidVolts()` are the two algorithms side by side.

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

### Choosing a controller (Elastic buttons)

On boot the controller is **OFF** — nothing moves until you pick one. Three command buttons switch
modes (selecting one disables the others):

- **`LinearExtension/Use Bang-Bang`**
- **`LinearExtension/Use PID`** (resets the PID integral each time you enter PID mode)
- **`LinearExtension/Disable Controller`** (back to OFF / 0 V)

The current choice is shown as text at `LinearExtension/ActiveController`.

### Running it

1. Deploy, open **Elastic**, connect to the robot.
2. Open the Driver Station and **Enable** (Teleop). *(Motors only move while enabled — that's the FRC
   safety interlock, not a bug.)*
3. Click **Use Bang-Bang** or **Use PID**, then change `Target_Inches` and tune.

### What students tune in Elastic

These live under the NetworkTables `Tuning/LinearExtension/` table (drag each onto the layout as a
text/number widget or slider; editing takes effect immediately, no redeploy):

| NT key | Meaning | Default |
| --- | --- | --- |
| `Tuning/LinearExtension/Target_Inches` | **Shared** setpoint both controllers drive to (clamped 0–11.5") | 4.0 |
| `Tuning/LinearExtension/Kv_Volts` | Demo 1: fixed drive voltage magnitude. **Clamped to 0–6 V.** | 2.0 |
| `Tuning/LinearExtension/Deadband_Inches` | Demo 1: how close to the target counts as "there" | 0.25 |
| `Tuning/LinearExtension/kP` | Demo 2: proportional gain (volts per inch of error) | 1.0 |
| `Tuning/LinearExtension/kI` | Demo 2: integral gain | 0.0 |
| `Tuning/LinearExtension/kD` | Demo 2: derivative gain | 0.0 |
| `Tuning/LinearExtension/PID_MaxVolts` | Demo 2: output voltage cap. **Clamped to 0–6 V.** | 4.0 |

> `Kv_Volts` is labeled **Kv** per the workshop plan. Note that "Kv" normally means a velocity
> feedforward gain — here it is simply the fixed bang-bang voltage. The code comments call this out
> so students aren't misled.

### The buttons

Elastic shows these command buttons (published from `RobotContainer`):

- **`LinearExtension/Use Bang-Bang`** / **`Use PID`** / **`Disable Controller`** — pick the active
  controller (works while **disabled**, so you can pre-select before enabling).
- **`LinearExtension/Zero`** — sets the current position as 0". Retract the mechanism by hand first,
  then click (works while **disabled**).
- **`Pivot/Zero`** — same idea, defines the current pose as horizontal (0°).

### The tuning lesson

**Bang-bang:**
- **Kv too low** → can't overcome friction/gravity, never reaches target.
- **Kv too high** → overshoots, then chatters/buzzes rapidly across the target (bang-bang's signature).
- **Deadband too small** → constant buzzing at the target. **Too large** → stops noticeably short.

**PID:**
- **kP too low** → sluggish, stops short. **Too high** → overshoot and oscillation.
- **kD** → damps overshoot; too much makes it jittery/noisy.
- **kI** → erases steady-state error (the last fraction of an inch), but too much causes slow
  oscillation / windup.

**The payoff:** switch between the two on the same target and watch `PositionInches` vs.
`TargetInches`. Bang-bang can't ease in, so it always trades speed for chatter; a tuned PID glides in
and holds. That contrast is the whole point of the workshop.

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
| `ControlMode` | which controller is active (`OFF` / `BANG_BANG` / `PID`) |
| `PositionInches`, `TargetInches`, `ErrorInches` | plot these three together |
| `MotorRotations` | raw rotor turns (before the 2.5:1) vs. `PositionInches` shows the gearing |
| `CommandedVolts`, `AppliedVolts` | what the controller asked for vs. what the motor did |
| `ClampedKv`, `DeadbandInches` | Demo 1 (bang-bang) values in effect |
| `PID_RawVolts`, `PID_MaxVolts` | Demo 2: PID output before clamping, and the cap |
| `VelocityInchesPerSec`, `StatorCurrentAmps` | |

Pivot logs `PositionDegrees` (mechanism), `MotorRotations` (raw rotor), `VelocityDegreesPerSec`, `AppliedVolts`, `StatorCurrentAmps`.

Plot `PositionInches` against `TargetInches`, then switch between **Use Bang-Bang** and **Use PID** on
the same target — the difference in the curve (chatter vs. a smooth glide-in) is the money shot for
the workshop.

---

## 5. Safety & gotchas (read before powering the mechanism)

- **Check the drive direction first.** With a low `Kv` (≈1 V) and a target above current position, the
  mechanism should **extend**. If it retracts, flip `LinearExtension.kInvert` in `Constants.java` to
  `Clockwise_Positive` (same for the pivot).
- **Soft limits** at 0" / 11.5" (and 0° / 100°) are enforced by the TalonFX as a backstop — but they
  are only correct **after** you've zeroed. Zero at a known pose before enabling.
- **Current limits** are set to 40 A (stator & supply). Lower them in `Constants.java` for a gentler
  demo.
- **All voltages are clamped 0–6 V in software** — bang-bang `Kv` and `PID_MaxVolts` both — so typing
  50 into Elastic still can't exceed 6 V. Raise `kMaxDriveVolts` in `Constants.java` for more.
- **Controller is OFF on boot** — nothing drives until you click **Use Bang-Bang** or **Use PID**.
- Brake mode holds position when the controller commands 0 V.

---

## 6. Project layout

```
src/main/java/frc/robot/
  Main.java              entry point
  Robot.java             LoggedRobot; AdvantageKit logging setup
  RobotContainer.java    creates subsystems, publishes controller + zero buttons
  Constants.java         CAN IDs, gear ratios, travel & safety limits, controller defaults
  subsystems/
    LinearExtension.java  ← Demos 1 & 2: bang-bang + PID, mode switching, tunables, logging
    Pivot.java            scaffold: config + telemetry + zero (controller = later demo)
```

## 7. Next steps

- **Pivot demo:** add a controller in `Pivot.java` (e.g. PID + gravity feedforward — the pivot fights
  gravity, unlike the extension). The hardware config, unit conversion, zeroing, and logging are
  already in place, and you can copy the mode-switching pattern from `LinearExtension`.
- Consider adding kS/kG/kV **feedforward** alongside the PID on the extension as a third controller.

---

*Versions: WPILib/GradleRIO 2026.2.1 · Phoenix 6 26.3.0 · AdvantageKit 26.0.2 · Gradle 8.11 · Java 17.*
