# Build notes (PETG frame)

The dimensions below are the ones the physics model and the URDF use. If
your printed parts come out different, change
`ros2_ws/src/rover_description/config/rover_geometry.yaml` and re-run the
climb analysis before you trust the rover on a step.

## Key dimensions

All measured from the rocker pivot axis, x forward, z up, wheels on flat
ground.

| Point | x (mm) | z (mm) | Notes |
|---|---|---|---|
| Rocker pivot | 0 | 0 | 107.5 mm above the floor |
| Front axle | +130 | −75 | on the rocker, steered by a servo |
| Bogie pin | −45 | −45 | 30 mm above the axles, 20 mm forward of the middle/rear midpoint |
| Middle axle | 0 | −75 | on the bogie |
| Rear axle | −130 | −75 | on the bogie |
| Body box | 220 × 140 × 70 | centre at +10 | bottom of the body is 82 mm off the floor |

- Track (left wheel centre to right wheel centre): 230 mm
- Rocker pivots sit 14 mm outboard of the body sides
- Bogie travel: ±40° before it should hit a printed stop
- Rocker travel relative to the body: ±29°
- Steering: ±60° on both front corners

Wheelbase is 260 mm with 130 mm between neighbouring axles, which leaves a
65 mm gap between tyres. Don't shrink it; the middle wheel needs that room
to swing up when the front one is on a step.

## Bearings

Put the bearings where the suspension pivots, because that's where friction
hurts. A sticky pivot makes the rocker-bogie stop sharing load evenly.

- rocker pivots: two bearings per side, spaced apart on the shaft so the
  rocker can't wobble
- bogie pins: one or two per side
- differential: one at the centre pivot of the bar on top of the body

608 bearings (8 mm bore) are common and fine. If yours are smaller, that's
fine too; just keep the shafts snug.

## The differential bar

Each rocker turns freely on its own pivot, so something has to stop the body
from flopping forward or back. A bar across the top of the body, pivoting at
its centre, with a short link from each end down to its rocker, does it. If
the left rocker tips nose-up, the bar pushes the right one nose-down, and the
body settles at the average. Use small ball links or loose M3 bolts for the
link ends so they can swing a little sideways.

## Motors and steering

- Mount the TT motors with the can pointing **up**. Lying flat, the 63 mm
  gearbox sticks out and catches the edge of the step before the wheel does.
- Front corners: the servo sits above the wheel and turns a knuckle that
  carries the motor. Put the steering axis straight down through the middle
  of the tyre so the wheel turns on the spot instead of swinging around.
  Take the side load with a bearing or a printed bushing; the servo spline
  shouldn't carry the rover's weight.
- Fit the servo horns only after the firmware has centred the servos (power
  up in HOLD), then fine-tune `kServoCentreUs`.

## Centre of mass

This matters more than we expected. The model says the 65 mm climb only
works if the centre of mass is within about 1 cm of the rocker pivot line
(a hair forward is best). Two centimetres either way and either the middle
wheel lifts or the rear one slips.

To check: lift the finished rover by a rod through both rocker pivots. It
should hang roughly level. Slide the battery packs until it does.

## Sonar mount

About 130 mm off the floor on the nose, tilted 10° up, facing straight
ahead. The tilt keeps climbable rocks under the beam (see physics.md).

## Printing

Reasonable starting settings for parts like these in PETG:

- 4 perimeters on the rocker and bogie arms, 30-40 % gyroid infill
- print the arms lying flat so layers run along the length of the arm
- 0.2 mm layers; slow the first layer down, PETG likes to string
- design holes for bearings 0.1-0.2 mm under size and press them in
- if you can, make the bogie arm arch up between the pin and the rear axle.
  The model checks a straight arm, which clears a 65 mm step edge by only
  6 mm.
