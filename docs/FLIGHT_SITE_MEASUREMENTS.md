# Physical mission measurements

## Operator information recorded on October 18

The operator reported 5-inch props, a 205 mm wheelbase, a highest point about
25 cm above the floor, FC height 8.5 cm, and approximately 0.5 m from the FC to
the rear boundary. Lateral takeoff placement can vary and has not been measured.

Assuming a symmetric true-X frame, a diagonal motor-to-motor 205 mm wheelbase
and an FC at the geometric centre, the nominal swept length/width is
`205 / sqrt(2) + 127 = 271.96 mm`: about 13.6 cm from the FC in each horizontal
direction. The vertical extents would be 16.5 cm above and 8.5 cm below the FC.
These horizontal values are calculated estimates; stretched frames, FC offsets,
and protruding sensors/batteries can change them. Confirm the actual envelope
before writing measured geometry. The existing 0.2 m planner collision radius
has not been replaced with this estimate.

At a 0.5 m rear offset, a nominal 4.5 m forward destination in the 5.8 m box
leaves about 66.4 cm from the front propeller edge to the front boundary. This is
static geometric clearance, not a tracking or obstacle-clearance measurement.
Record the actual left offset for each takeoff placement; no centre placement
has been assumed or written into the physical mission template.

## Fields to measure

Use metres and measure from the **flight-controller centre**, not the LiDAR or
the aircraft's visual centre. Forward is the initial nose direction along the
5.8 m edge; left is the aircraft's left while looking forward. These measurements
place the outer operating box; they do not prescribe a straight obstacle route.

Save a separate copy of `~/Downloads/open_day_mission.json` as
`~/Downloads/open_day_mission_measured.json`. Fill its existing fields:

| Measure | Mission JSON field | Method |
|---|---|---|
| Front extent | `vehicle.centre_to_front_m` | FC centre to the foremost propeller-inclusive edge |
| Rear extent | `vehicle.centre_to_rear_m` | FC centre to the rearmost propeller-inclusive edge |
| Left extent | `vehicle.centre_to_left_m` | FC centre to the leftmost propeller-inclusive edge |
| Right extent | `vehicle.centre_to_right_m` | FC centre to the rightmost propeller-inclusive edge |
| Top extent | `vehicle.centre_to_top_m` | FC centre to the highest aircraft point |
| Bottom extent | `vehicle.centre_to_bottom_m` | FC centre to the lowest aircraft point |
| Takeoff rear offset | `takeoff.forward_from_rear_m` | Grounded FC centre to the rear boundary, along the length |
| Takeoff left offset | `takeoff.distance_from_left_m` | Grounded FC centre to the left boundary, across the width |

Also measure the **grounded FC centre's height above the takeoff floor**. The
current conversion assumes 0.085 m. This value is a separate floor calibration;
if the measurement differs, resolve that calibration before using physical
height targets. Do not compensate by changing an aircraft extent or wall offset.

Confirm the outer operating box is 5.8 m long, 2.8 m wide and 2.8 m high, with
the nose along its length. Measure the usable clearance rather than the overall
laboratory dimensions, including protrusions, overhead nets and fixtures.
The aircraft envelope must fit at takeoff and at the 4.5 m destination. A takeoff
point directly on a boundary cannot fit an aircraft with a nonzero extent.

For each of the two pillars, record its position from the rear and left
boundaries, width/diameter and height. These values help compare the occupied
map with the real scene. SUPER still plans around the actual sensed obstacles;
the measurements do not hard-code a left/right detour. Thin wires or nets that
are absent from a scan remain physical obstacles.

Once measurements are supplied, the disarmed physical preparation command in
[the reboot quick start](REBOOT_QUICK_START.md#prepare-physical-execution-separately)
captures one live FC ground position and heading. The planner and controller
share that fixed frame and the same six vehicle-centre boundary planes. A fresh
session is required after moving/restarting localization; the preparation does
not reuse the rehearsal's synthetic geometry or frozen map.
