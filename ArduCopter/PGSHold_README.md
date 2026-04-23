# PGSHold Flight Mode

Mode number: **88**

AltHold with gentle automatic position correction driven by external navigation (odometry).
The altitude axis is identical to AltHold. On the horizontal axes, when the pilot centers the
sticks the mode holds the last known XY position using a PD lean-angle controller fed by the
EKF position and velocity estimates.

## Requirements

### EKF source parameters

| Parameter | Value | Meaning |
|---|---|---|
| `EK3_SRC1_POSXY` | 6 | EXTNAV as XY position source |
| `EK3_SRC1_VELXY` | 6 | EXTNAV as XY velocity source |
| `EK3_SRC1_VELZ` | 6 | EXTNAV as Z velocity source |

### MAVLink ODOMETRY message

The external nav system must send `ODOMETRY` (MAVLink message ID 331) with:

| Field | Required value |
|---|---|
| `frame_id` | `MAV_FRAME_LOCAL_FRD` (13) |
| `child_frame_id` | `MAV_FRAME_BODY_FRD` (12) |

Any other frame combination is silently dropped by ArduPilot.

---

## Mission Planner note

Mission Planner does not automatically know the display name of this custom
flight mode. To make `PGSHOLD` appear in the Mission Planner flight mode
dropdowns and parameter value lists, you must manually edit the Mission
Planner parameter definition file `ArduCopter.apm.pdef.xml` and add the
`PGSHOLD` mode entry there.

Typical Windows cache location:

`C:\ProgramData\Mission Planner\ArduCopter.apm.pdef.xml`

Typical Linux/Mono cache location:

`~/.local/share/Mission Planner/ArduCopter.apm.pdef.xml`

Add `PGSHOLD` to the Copter flight mode values, for example:

```xml
<value code="88">PGSHold</value>
```

After editing the file, restart Mission Planner. If Mission Planner rewrites
the cache from downloaded metadata, you may need to re-apply the change.

---

## Tunable parameters

All parameters are visible and writable via MAVLink / Mission Planner / QGroundControl.
They persist across reboots in EEPROM.

### `PGSH_P` — Position P gain
| | |
|---|---|
| **Units** | cdeg / cm |
| **Default** | 0.30 |
| **Range** | 0.05 – 2.0 |
| **Increment** | 0.05 |

Controls how aggressively the drone leans toward the hold position.
Higher = faster correction, but increases risk of oscillation.

> **0.30** → 3° lean angle per 10 m of position error  
> **0.10** → 1° lean angle per 10 m of position error (conservative, high latency)

---

### `PGSH_D` — Velocity D gain
| | |
|---|---|
| **Units** | cdeg / (cm/s) |
| **Default** | 3.00 |
| **Range** | 0.0 – 20.0 |
| **Increment** | 0.5 |

Damps oscillation by countering current velocity. The D term uses the velocity
estimate from `ahrs.get_velocity_NED()` — if that estimate is unavailable or
heavily delayed, set this to **0** to disable and use P-only control.

> **Rule of thumb**: if the drone oscillates, halve `PGSH_D` first before reducing `PGSH_P`.

---

### `PGSH_MAX` — Maximum correction angle
| | |
|---|---|
| **Units** | cdeg (centidegrees) |
| **Default** | 300 (= 3°) |
| **Range** | 50 – 1000 |
| **Increment** | 50 |

Hard cap on the lean-angle correction that the position hold algorithm can apply.
The pilot always retains full authority — this only limits the *automatic* correction.

> **300 cdeg = 3°** is a gentle nudge; increase to 500–600 for faster recovery from larger errors.

---

### `PGSH_LAT` — Sensor latency (dead-reckoning predictor)
| | |
|---|---|
| **Units** | seconds |
| **Default** | 0.00 (disabled) |
| **Range** | 0.0 – 1.0 |
| **Increment** | 0.02 |

End-to-end latency of the external nav pipeline (USB/serial transport +
vision processing time + EKF fusion lag). When non-zero, both position and
velocity are predicted forward by this amount using IMU acceleration before
the PD correction is computed, compensating for the delay.

**How to measure latency:** compare the timestamp when your vision system
sends the ODOMETRY message against when the EKF position output visibly changes
in a dataflash log (NKF1 message, PE/PN fields).

> **Example:** 200 ms camera + 50 ms serial = set `PGSH_LAT = 0.25`

> **Note:** Only set this if position *and* velocity are both delayed.
> If velocity is low-latency (e.g. from IMU dead-reckoning on the companion
> computer), leave at 0 and rely on the D term alone for damping.

---

## Tuning procedure

1. **Start conservative** — especially with high latency:
   - `PGSH_P = 0.10`, `PGSH_D = 0`, `PGSH_MAX = 200`

2. **Enable latency compensation** if you know your delay:
   - Set `PGSH_LAT` to measured latency in seconds

3. **Increase P** slowly until you see a clear return-to-hold tendency.
   Stop before oscillation begins.

4. **Add D damping** — increase `PGSH_D` from 0 in steps of 1.0 until
   overshoot is reduced without causing oscillation.

5. **Adjust MAX** — increase `PGSH_MAX` if the correction feels too weak
   when released from 5+ m away. Decrease if corrections feel jerky.

---

## Failure behaviour

| Condition | What happens |
|---|---|
| EXTNAV drops mid-flight | Position correction silently disabled; hold target invalidated; mode keeps flying as plain AltHold |
| EXTNAV recovers | Hold target re-latches at current position on next stick-release |
| Velocity estimate unavailable | D term zeroed automatically; P-only correction continues |
| EKF position garbage spike | `PGSH_MAX` limits worst-case lean; EKF health gate filters persistent loss |
