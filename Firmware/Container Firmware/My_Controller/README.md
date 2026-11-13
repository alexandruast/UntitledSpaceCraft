# My controller

`Hub_3_1/` here is the generated hub sketch for this controller. Don't edit it: change
`../Hub_3_1` or `../Hub_3_1/Modules`, then regenerate with:

    python "Firmware/Container Firmware/assemble_hub.py" --compile

| Panel | Snippet |
|---|---|
| Action Groups | h1 |
| Telemetry | c1 (custom 20-field protocol) |
| Control Systems | g1 |
| Time | b1 |
| Rotation (Gear, Light, Trim, PLN/RKT/RVR) | f3 |
| Translation | f5 |
| Throttle | f6 |
| Executive Actions (Arm, Abort, Stage) | a3 |

Hub board: Arduino Micro (`arduino:avr:micro`). Libraries: KerbalSimpit, Keyboard.

The Telemetry module must run `Firmware/Module Firmware/Telemetry/Telemetry.ino` (U8g2):
the stock Telemetry firmware doesn't understand the 20-field protocol.

## Telemetry pages

The screen blanks while not in flight. Distances and speeds are whole metres / m/s,
shortened with k/M/G/T past 5 digits; times are mm:ss, hh:mm, then days (d) or years (y).

| Button | Left column | Right column |
|---|---|---|
| AL/VL | APO, PER, TTA, AOA, PIT, RLL, HDG, THR | ALT, VRT, HRZ, G, dVS, dVT, MCH, Q |
| MNVR | NDV, TTN, TTB, BRN, ADV, NHD, NPT | APO, PER, TTA, TTP, dVS, dVT, MRG |
| ORBT | SOI, SMA, ECC, INC, PRD, LAN, AOP, TA | APO, PER, TTA, TTP, ALT, VOB |
| FUEL | gauges: LF, OX, SF, EC, XE, MP (no bar = none aboard) | |

- APO / PER: apoapsis / periapsis; TTA / TTP: time to them
- ALT: altitude above sea level; VRT / HRZ: vertical / horizontal surface speed; VOB: orbital speed
- AOA: angle between nose and surface velocity (--- when stationary); Q: dynamic pressure, kPa
- PIT / RLL / HDG: attitude; THR: throttle %; MCH: Mach; G: g-force
- dVS / dVT: stage / total delta-V
- NDV: next node dV; TTN: time to node; TTB: time to burn start (node minus half the burn, NOW once started);
  BRN: burn duration; ADV: dV of all nodes; NHD / NPT: node heading / pitch (KSP1 only);
  MRG: ship dV minus all nodes' dV (negative = not enough dV)
- SOI: body orbited; SMA: semi-major axis; ECC: eccentricity; INC: inclination; PRD: period;
  LAN: longitude of ascending node; AOP: argument of periapsis; TA: true anomaly
