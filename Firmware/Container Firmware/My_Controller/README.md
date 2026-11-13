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
