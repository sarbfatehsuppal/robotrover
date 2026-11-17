# Wiring

Everything here matches `firmware/stm32_body/include/board.hpp` and
`firmware/esp32_brain/include/config.hpp`. If you move a wire, move it in
the code too.

## Power

Two battery packs, two buck converters, one shared ground. The motors get
their own pack so that six TT motors starting at once can't brown out the
ESP32 (which otherwise reboots, drops WiFi and makes you think the code is
broken).

```
 8x AA pack A ──► MP2482 buck A (5 V) ──► VM on all three MX1616 boards
                                          (motors only)

 8x AA pack B ──► MP2482 buck B (5 V) ──┬─► servo red wires (x2)
                                        ├─► Black Pill "5V" pin
                                        ├─► ESP32 "VIN" / "5V" pin
                                        └─► HC-SR04 VCC

 GND of everything tied together: both packs, both bucks, all MX1616s,
 Black Pill, ESP32, servos, HC-SR04.
```

Notes:

- Use NiMH AAs if you can. Eight NiMH cells sit around 9.6 V, comfortably
  above the buck's 7 V minimum, and they handle the current spikes from
  climbing much better than alkalines.
- Measure the buck outputs **before** connecting anything. Some of these
  modules ship set to a different voltage.
- When you flash a board over USB, switch pack B off first. Some Black Pill
  and ESP32 clones don't have a diode between USB 5 V and the 5V pin, and
  you don't want the buck and your laptop's USB port fighting.
- A 470 µF capacitor across each MX1616's VM/GND is cheap insurance.

## STM32 Black Pill (STM32F411CEU6)

| Function                 | Pin  | Timer / peripheral | Connects to                         |
|--------------------------|------|--------------------|-------------------------------------|
| Front-left motor IN1     | PA6  | TIM3 CH1           | MX1616 #1 IN1                       |
| Front-left motor IN2     | PA7  | TIM3 CH2           | MX1616 #1 IN2                       |
| Middle-left motor IN1    | PB0  | TIM3 CH3           | MX1616 #1 IN3                       |
| Middle-left motor IN2    | PB1  | TIM3 CH4           | MX1616 #1 IN4                       |
| Rear-left motor IN1      | PB6  | TIM4 CH1           | MX1616 #2 IN1                       |
| Rear-left motor IN2      | PB7  | TIM4 CH2           | MX1616 #2 IN2                       |
| Front-right motor IN1    | PB8  | TIM4 CH3           | MX1616 #2 IN3                       |
| Front-right motor IN2    | PB9  | TIM4 CH4           | MX1616 #2 IN4                       |
| Middle-right motor IN1   | PA5  | TIM2 CH1           | MX1616 #3 IN1                       |
| Middle-right motor IN2   | PA1  | TIM2 CH2           | MX1616 #3 IN2                       |
| Rear-right motor IN1     | PB10 | TIM2 CH3           | MX1616 #3 IN3                       |
| Rear-right motor IN2     | PA3  | TIM2 CH4           | MX1616 #3 IN4                       |
| Left steering servo      | PA8  | TIM1 CH1 (50 Hz)   | servo signal (orange/yellow)        |
| Right steering servo     | PA2  | TIM9 CH1 (50 Hz)   | servo signal (orange/yellow)        |
| HC-SR04 TRIG             | PB12 | GPIO               | TRIG                                |
| HC-SR04 ECHO             | PB13 | EXTI13             | ECHO **through a 1 kΩ / 2 kΩ divider** |
| UART TX to ESP32         | PA9  | USART1 TX          | ESP32 GPIO16 (RX2)                  |
| UART RX from ESP32       | PA10 | USART1 RX          | ESP32 GPIO17 (TX2)                  |
| Status LED               | PC13 | GPIO               | on-board LED                        |
| Power                    | 5V, GND | –               | buck B, common ground               |

Left free on purpose: PA0 (the KEY button), PA4, PA11/PA12 (USB), PA13/PA14
(SWD), PA15, PB3, PB4, PB5, PB14, PB15. PA4 is an ADC pin if you want to add
a battery voltage divider later.

The ECHO divider: ECHO → 1 kΩ → PB13, and PB13 → 2 kΩ → GND. That turns the
HC-SR04's 5 V pulse into 3.3 V. PB13 is 5 V tolerant on paper, but the
divider costs two resistors and you never have to wonder.

## MX1616 boards

You need **three** boards for six motors (two motors each). Each board:

- VM / + : buck A 5 V
- GND    : common ground
- IN1..IN4 : from the table above
- MOTOR-A : the first motor listed for that board, MOTOR-B : the second

The firmware drives them in "slow decay": one input high, the other PWM'd at
20 kHz. Stopped means both inputs high, which brakes the motor. That's what
keeps the rover from rolling back off a step when you let go.

If a wheel spins the wrong way, don't swap wires; flip its entry in
`kMotorInvert` in `board.hpp`.

## ESP32 DevKit

| Function             | Pin    | Connects to            |
|----------------------|--------|------------------------|
| UART RX2             | GPIO16 | Black Pill PA9 (TX)    |
| UART TX2             | GPIO17 | Black Pill PA10 (RX)   |
| Start/stop mission   | GPIO0  | the on-board BOOT button |
| Status LED           | GPIO2  | on-board LED           |
| Power                | VIN/5V, GND | buck B, common ground |

Both chips are 3.3 V logic, so the UART connects directly. Remember it
crosses over: TX goes to RX.

## Servos

Brown/black = GND, red = buck B 5 V, orange/yellow = signal. Before you fix
the steering arms on, power everything up with the rover in HOLD mode. The
firmware centres both servos at boot, so that's the moment to put the arms
on pointing straight ahead. Fine-tune with `kServoCentreUs` in `board.hpp`.
If positive angles turn a wheel right instead of left, flip `kServoSign`.

## HC-SR04

VCC to buck B 5 V, GND to ground, TRIG to PB12, ECHO through the divider to
PB13. Mount it on the nose of the body, about 13 cm off the ground, tilted
**10° upward**. The tilt matters: it keeps rocks the rover can climb under
the beam, so it drives over them instead of steering around them. See
[physics.md](physics.md#why-the-sonar-is-tilted-up).
