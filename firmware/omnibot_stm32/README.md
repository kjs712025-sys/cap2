# OmniBot STM32 firmware

Arduino sketch for the **NUCLEO-F411RE** that runs the motor side of the
`robot/stm32.py` protocol: it receives body-velocity setpoints over the
ST-Link USB serial port and drives four brushed DC motors through two
**AM-DC2-2D** dual drivers (4-wheel omni base).

## Flash it

1. Arduino IDE → Boards Manager → install **STM32 MCU based boards**
   (STMicroelectronics).
2. Tools:
   - Board → **Nucleo-64**
   - Board part number → **Nucleo F411RE**
   - U(S)ART support → **Enabled (generic 'Serial')**
   - Upload method → **STM32CubeProgrammer (SWD)** (or Mass Storage)
3. Open `omnibot_stm32/omnibot_stm32.ino`, select the ST-Link port, **Upload**.

The board then appears on the Raspberry Pi as `/dev/ttyACM0` at 115200 baud —
already the value in the backend `.env` (`ROBOT_UART_PORT`).

## Wiring

| motor | PWM | DIR | EN | AM-DC2-2D |
|---|---|---|---|---|
| M0 FL | D3 | D4 | D5 | #1 ch 1 |
| M1 FR | D6 | D7 | D8 | #1 ch 2 |
| M2 RL | D9 | D10 | D11 | #2 ch 1 |
| M3 RR | D12 | D13 | D2 | #2 ch 2 |

Common ground between the Nucleo, both drivers and the motor battery. Do **not**
power the motors from the Nucleo. `D13` is also the on-board LED, so `LD2`
blinks with M3's direction bit.

## Calibrate (wheels off the ground)

Serial Monitor @ 115200, line ending **Newline** — checksums are optional when
typing by hand:

| send | expect | if wrong |
|---|---|---|
| `A,1` | `S,armed` back | — |
| `V,0.2,0,0` | all wheels drive forward | flip the wrong wheel's `MOTOR_SIGN[]` |
| `V,0,0,0.5` | rotates left (CCW) in place | physical wheel order ≠ M0..M3 → reorder the pin arrays / `WHEEL_MIX` rows |
| `V,0,0.2,0` | strafes left | same as above |
| `A,0` | `S,idle`, everything stops | — |

## Options (top of the .ino)

- `MOTOR_SIGN[]`, `WHEEL_MIX[]` — direction / kinematics calibration.
- `EN_ACTIVE`, `DIR_POS` — driver logic polarity.
- `PWM_MIN`, `SLEW_STEP`, `PWM_FREQ_HZ` — motor feel / deadband / soft-start.
- `ENABLE_BATTERY` (+ `BATT_PIN`, `BATT_DIVIDER`) — send `B,<volts>` telemetry.
- `USE_ENCODERS` — only if quadrature encoders are wired; then fill in
  `updateOdometry()` so the firmware sends real `O,<x>,<y>,<yaw>` frames.
  Leave it `0` otherwise (the backend dead-reckons on its own).

## Protocol

```
frame = "<payload>*<HH>\n"     HH = (sum of payload bytes) & 0xFF, upper-hex
Pi  -> STM32 :  A,1 | A,0 | V,<vx>,<vy>,<wz> | H
STM32 -> Pi  :  S,<state> | B,<volts> | E,<msg> | O,<x>,<y>,<yaw>
```

If no `V`/`H` arrives for 500 ms the wheels coast to a stop (the board stays
armed and resumes when frames return); the next `S,` heartbeat reports
`armed_nolink`.
