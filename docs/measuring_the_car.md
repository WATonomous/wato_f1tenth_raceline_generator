# Measuring the car for the raceline

The raceline is only as good as the numbers in `params/wato_car.yaml`. These are the measurements
that matter, cheapest first. Write each result into the profile with the date and the surface, and
re-run `make_raceline.py`.

| Value in `wato_car.yaml` | How | Matters for |
| --- | --- | --- |
| `car.mass_kg` | weigh it | everything (forces to accelerations) |
| `grip.mue` | pull test, then circle test | corner speeds, braking |
| `resistance.drag_coeff`, `rolling_coeff` | coast-down | top speed on straights, lap time |
| `limits.*` | copy from the stack (`vesc.yaml`) | `make_raceline.py` warns if they differ |

## 1. Mass (2 minutes)

Hang the car from a luggage or spring scale in a loop of strap, battery and everything fitted.
Expect about 3.7-4.2 kg.

## 2. Grip: pull test (10 minutes, no stack needed)

A digital luggage scale (about 10 g resolution) is enough: the pull is about 2-3.5 kg on a 4 kg car.

1. Tie a string to the side of the chassis, low and near the middle of the car.
2. Pull **sideways**, slowly and level with the floor. Cornering grip is sideways, and the wheels
   cannot roll that way, so no brake is needed.
3. Note the **peak** just before it starts to slide (static) and the **steady reading** while it
   slides (sliding). Repeat 3-5 times; use the average.
4. `mue = pull / weight` (same units, e.g. 2.4 kg / 3.9 kg = 0.62). Use the sliding value; it is
   the lower, safer one.
5. Do it on every surface the car races on. Set `grip.mue` to the measured value and
   `grip.safety_factor` to about 0.8 (the line then plans for 80% of the grip).

Until this is done the profile keeps `mue: 0.3`, which is cautious: corners come out slow.

## 3. Grip: circle test (needs the car and the stack, about 20 minutes)

The pull test is quasi-static; tyres on a moving car can differ. To check it:

1. Tape a circle of known radius (1.5-2 m) on the floor.
2. Record a bag (`./watod bag record -d`) and drive the circle under teleop, speeding up slowly
   until the car starts to slide wide. Stop the bag (`./watod bag stop`).
3. Sideways grip at the slide: `mue = v^2 / (r * 9.81)`, with `v` from `/hal/odom` (or the IMU's
   sideways acceleration directly: `mue = a_y / 9.81`).

## 4. Braking (optional)

The car's software brakes at 4.0 m/s^2 (`max_decel`), which is below the grip on most floors, so
braking is limited by the software, not the tyres. Only worth measuring if `mue * 9.81` comes out
below 4.0 m/s^2 (mue under 0.4): then lower `limits.max_decel_mps2` to match.

## 5. Coast-down (drag and rolling resistance)

1. The car must roll freely, but the stack's speed limiter brakes whenever the command drops (and when
   commands stop). Either set `max_decel: 0.1` in the stack's `vesc.yaml` for the test (put it back after),
   or use a VESC test script that ends with zero current, like the braking baseline in the VESC tuning
   (`setCurrent(0)` coasts). Then bring the car to about 8 m/s on a long straight and let it roll.
2. From the bag, fit the slowdown `m * dv/dt = -(c_roll * m * g + c_drag * v^2)`:
   `c_drag` -> `resistance.drag_coeff`, `c_roll` -> `resistance.rolling_coeff`.

The current `drag_coeff: 0.075` matches the ~80 W the drivetrain used spinning at 10 m/s on the
stand (2026-11-10), so it is a reasonable stand-in.

## Is the friction map worth it?

The generator can use a friction map (`frictionmap/`, `tpadata`, `var_friction` in
`main_globaltraj_f110.py`): grip that **changes around the track**, for example a wet patch or a
change from tile to carpet. On a single indoor floor grip is about the same everywhere, so the map
adds work for nothing. Use it only if the pull test gives values more than about 15% apart on
different parts of the same track; then measure each area and build a map with
`main_gen_frictionmap.py`.
