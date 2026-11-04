# finger_sim

Three-link, two-joint tendon-driven finger with a motorized base reel in the stock MuJoCo viewer. No browser, web server or extra GUI framework is used.

## Launch

```bash
cd ~/Desktop/finger_sim
./launch.sh
```

The MuJoCo window opens with both original left and right panels visible. Custom measurements are overlaid in the simulation viewport; the native panels remain available. In the native right-hand panel, expand **Control** and adjust **reel_torque_Nm** from 0–0.06 N·m. A motor rotates a 10 mm radius, single-groove reel at the finger base. One central tendon runs along the underside and crosses both finger joints; one motor drives the underactuated finger through an ideal taut-cable coupling.

## Native Control panel

Both original MuJoCo side panels remain visible. Expand **Control** on the right:

| Slider | Meaning | Range | Default |
| --- | --- | --- | --- |
| reel_torque_Nm | Motor torque | 0–0.06 N·m | 0 |
| spring_1_N_per_m | Extension spring 1 stiffness | 100–2000 N/m | 1000 |
| spring_2_N_per_m | Extension spring 2 stiffness | 100–2000 N/m | 700 |
| pulley_1_radius_mm | Joint 1 cable guide radius | 12–16 mm | 14 |
| pulley_2_radius_mm | Joint 2 cable guide radius | 12–16 mm | 14 |

Values update the physical parameters while the simulation runs, including while paused. There are no custom text overlays. **Home** releases the motor and resets the pose while keeping settings; **End** restores defaults; **Space** pauses/resumes. On Mac: Fn+Left = Home, Fn+Right = End. Native Clear all/Reset releases torque but preserves mechanical parameter settings when it zeros the control values.

These four parameter entries are force-free UI channels (zero gain and transmission gear). They add no joints or physical actuation. There is still one physical motor, two finger joints and one reel shaft. Use run.py/launch.sh: a generic XML viewer alone will not map the slider values to parameters. Measurements remain available through FingerExperiment.measure() for analysis, without covering the viewport.

## Model and parameter limits

- Fixed link centerline lengths: 50, 35, 25 mm; illustrative masses: 25, 18, 12 g.
- Two hinge joints: 0–75 degrees, damping 0.002 N·m·s/rad.
- One central flexor tendon across both joints; the same cable carries the entire transmitted tension.
- Reel: fixed 10 mm effective winding radius, shaft travel 0–300 degrees, illustrative 10 g core mass. Motor torque: 0–0.06 N·m.
- The model now has three hinge coordinates: two finger joints plus one reel shaft. One cable-length equality couples them, and there is still only one motor.
- Relation: external cable shortening = reel radius × reel rotation in radians. Steady tendon tension is approximately motor torque / reel radius.
- Two extension springs directly above the hinges, attachment height 14 mm.
- Stiffness ranges: 100–2000 N/m; defaults 1000 and 700 N/m.
- Spring natural/slack length: 24 mm; damping 1 N·s/m.
- Pulley radii: 12–16 mm, default 14 mm. Cable attachment sites remain fixed.

Guide radius changes affect the real wrap geometry and moment arms when the cable contacts the guide. Smaller guides that the cable clears have little effect. Radius limits keep guide sites outside the pulley. Changing geometry/stiffness while loaded can introduce a transient; reset before comparing trials.

The finger guides and cable paths are ideal massless/frictionless elements. The reel has a rotating shaft, inertia, damping and an actual joint motor. A tendon equality represents constant-radius, taut cable transmission. The single-groove reel and winding arcs are rendered, but individual cable layers, changing effective radius, slack, friction and stretch are not modeled. The equality is bilateral: transient signed reactions can be negative, which a real slack cable could not transmit. This is an ideal taut-cable approximation, not a tension-only cable model. Spring coils are visual-only; physical extension springs are spatial tendons. The reference diagram's third spring/hood and actual hardware brackets are not included. The model is not calibrated to hardware.

## Files

- models/finger.xml: links, cable paths, guides, physical extension springs and actuator.
- scenes/starter.xml: environment and model include.
- experiment.py: physical parameters and measurements.
- viewer_controls.py: native-slider synchronization and reset/pause shortcuts.
- visuals.py: coil visualization.
- run.py: stock viewer and headless runner.

## Headless check

```bash
./launch.sh --headless --steps 1000
```

## Recreate environment

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

MuJoCo is installed locally with pip; no model repository checkout is required. On macOS the graphical runner automatically uses mjpython.

## Example reel response

At default spring stiffnesses/routing, 0.02 N·m produces about 12°/15° finger angles and 6.6 mm take-up. At 0.06 N·m, the finger reaches approximately 75°/75° and 40 mm take-up. Setting torque to zero allows spring-driven unwinding, settling with slight gravity sag. Take-up and external shortening differ slightly because the equality is a soft numerical constraint.

The model has one pulling spatial tendon named flexor. The two return_spring spatial tendons represent passive extension springs; reel_take_up is an algebraic shaft transmission. These are not extra pulling cables.
