# Multi-track validation

The crossbar layout (all cameras at x 0.05 m, z 0.75 m; sides at y +-0.30 m, 60 deg out, 25 deg down; front 6 deg down) scored on Purdue and 5 tracks from kart-sw's generator. "% of best" compares with the best set found on that track (the named sets and one coarse re-optimisation of each camera with the other two at the crossbar). It holds when it is >= 95 % of best with no unseen edge stretch over 1 m within 8 m ahead.

| Track | Length (m) | Width (m) | Tightest radius (m) | Crossbar % of best | 65 deg sides % of best | Nose % of best | Worst gap 0-8 m (m) | Beside the kart (avg / worst 10 %) | Holds |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| purdue | 429 | 5.1-6.7 | 5.9 | 98.1 | 99.8 | 89.1 | 0.75 | 79 / 76 % | yes |
| kart_s101 | 552 | 8.0-8.7 | 7.6 | 98.1 | 100.0 | 91.5 | 0.00 | 85 / 83 % | yes |
| kart_s102 | 465 | 5.8-6.6 | 5.7 | 97.3 | 99.3 | 90.4 | 8.00 | 84 / 80 % | NO |
| kart_s103 | 438 | 6.2-6.7 | 6.1 | 97.4 | 99.4 | 90.6 | 0.50 | 82 / 79 % | yes |
| kart_s104 | 368 | 6.3-6.9 | 6.1 | 97.5 | 99.9 | 91.7 | 0.00 | 81 / 76 % | yes |
| flowing_s201 | 381 | 6.4-6.5 | 6.2 | 97.8 | 99.8 | 91.0 | 0.00 | 84 / 83 % | yes |

Best position per camera when re-optimised on each track (others at the crossbar layout):

| Track | Front (x, y, z, yaw, pitch) | In box | Left | In box | Right | In box |
| --- | --- | --- | --- | --- | --- | --- |
| purdue | 0, -0.1, 0.7, 0, 0 | yes | 0, 0.3, 0.7, 65, 20 | yes | 0, -0.3, 0.7, -65, 20 | yes |
| kart_s101 | 0.6, 0.1, 0.8, 0, 0 | no | 0, 0.3, 0.8, 65, 10 | no | 0, -0.3, 0.8, -65, 10 | no |
| kart_s102 | 0.5, -0.1, 0.8, 0, 15 | no | 0, 0.3, 0.8, 65, 20 | no | 0, -0.3, 0.8, -65, 20 | no |
| kart_s103 | 0.6, 0, 0.8, 0, 2.5 | no | 0, 0.3, 0.8, 65, 10 | no | 0, -0.3, 0.8, -65, 20 | no |
| kart_s104 | 0.4, -0.1, 0.8, 0, 0 | no | 0, 0.3, 0.8, 65, 20 | no | 0, -0.3, 0.8, -65, 20 | no |
| flowing_s201 | 0.6, 0, 0.8, 0, 2.5 | no | 0, 0.3, 0.8, 65, 20 | no | 0, -0.3, 0.8, -65, 20 | no |
