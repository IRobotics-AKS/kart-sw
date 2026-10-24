# Kart hardware inventory

> Superseded on 2026-10-24 by the monorepo design. See the bundle README.

Updated: 2026-10-24.

Source: the user's parts-list image and follow-up correction excluding the
Logitech camera. These are identified project parts, not confirmation that each
device is installed, configured, or tested. Quantities below come from the image.

| Component | Quantity | Role / status |
| --- | --- | --- |
| NVIDIA Jetson Orin Nano Super Developer Kit | 1 | Onboard autonomy computer; exact installed JetPack and storage unknown |
| Luxonis OAK-D S2 | 1 | Front camera |
| Luxonis OAK-1 Lite | 2 | Side cameras; individual mounting transforms and device identifiers pending |
| BNO055 Absolute Orientation IMU Fusion Breakout | 1 | IMU; exact breakout manufacturer/revision and connection pending |
| ArduSimple GNSS RTK Starter Kit, with u-blox ZED-F9P named in the notes | 1 | GNSS; exact kit SKU, correction source, and configuration pending |
| Google Coral | 1 listed | Optional / undecided; user has not identified a need or workload; exact variant unknown |

The GNSS row lists a Budget Survey antenna and an America-region 2G/3G/4G
internet option. These are parts-list selections, not verified installed
capabilities. In particular, the antenna's listed L1/L2/L5 support does not
establish receiver-band support, and internet access does not by itself establish
an RTK correction service or an RTK fix.

## Explicit exclusion

The Logitech BRIO appears in the image but the user explicitly stated it will
not be used. The planned camera set is exactly three Luxonis devices: one front
OAK-D S2 and two side OAK-1 Lite units. Do not add a BRIO driver or simulated
BRIO camera based on the original image.

## Previously discussed components

The user clarified on 2026-10-24 that the Coral may not be needed. It is not a
required system component. Recommendation: begin without a Coral dependency and
reconsider only for a specific compatible inference workload after profiling
the Jetson/OAK pipeline. This is not a decision to permanently exclude it.

- Top Kart Dreamer chassis.
- TRAMPA VESC 75/300 MkIV controllers; quantity and actuator roles remain unknown.
- MCU supervisor is proposed; its model and transport are not selected.
- Motors, steering actuation, brake actuation, encoders, battery/BMS, power
  distribution, and independent emergency-stop hardware remain unspecified.

## Integration details still needed

- Exact JetPack release, storage, power/cooling, and any carrier-board changes.
- Camera focus variants, serial/MX identifiers, selected streams, resolution,
  frame rates, synchronization, calibration, and mounting transforms.
- BNO055 breakout wiring, operating mode, timestamping, and calibration.
- GNSS kit revision, host interface, antenna location, correction provider/base
  station, and delivery path for corrections.
- Whether the Coral is actually needed, its exact variant, and its workload.

This inventory does not change the provisional software version baseline or
constitute a validated ROS/Isaac ROS integration.
