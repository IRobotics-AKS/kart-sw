# models/dreamer

`kart.xml` is the Top Kart Dreamer (MSTEM3 E1) model for MuJoCo. It is hand-edited.

- **Visuals:** CAD meshes in `assets/` (frame, rims, rear tyre, rear hubs, sprocket hub,
  brake rotor, caliper, engine mount), converted from the team's Fusion STEP exports.
  The front tyre mesh is generated from the 10x4.50-5 size (its STEP export was empty).
  Motor, battery, electronics, steering column and bodywork are simple shapes (no seat: autonomous).
- **Collisions:** frame = capsules along the CAD tube centre lines; hubs, rotor,
  sprocket hub, caliper and engine mount = their meshes; tyres = cylinders. Kart parts
  collide with the world, not with each other. They are in geom group 3 (press 3 in the
  viewer to show them).
- **Mass:** from the CAD and the BoM, with ballast to the 125 kg minimum: 40 % on the
  front axle, centre of mass 0.107 m above the ground, yaw inertia ~21.8 kg m^2.
- **Steering:** kingpin inclination 10.2 deg (Top Kart), caster 12 deg (placeholder),
  tie rods as a joint coupling.
- **Actuators:** `steer` (front wheel angle, rad), `drive` (rear axle torque, N m,
  +-70 = +-500 N at the tyres; negative = regen), `brake` (0..1, damper on the rear axle,
  up to 210 N m). Motor speed/power limits are left to the control code.

The header comment in `kart.xml` lists every source and every placeholder that still
needs measuring on the chassis (caster, spindle length, track widths, ride height,
component positions, gear ratio).
