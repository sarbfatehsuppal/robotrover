# Four-motor rover carrier schematic - Draft A

Prepared for Sarbfateh Suppal, November 18, 2026.

## Open the design

Extract the whole folder, then open **Rover_Carrier.PrjPcb** in Altium Designer. Keep the three SchDoc files beside it. Double-click a sheet in the Projects panel to view it. The symbols and electrical pins are embedded; no external symbol library is needed to view or edit these sheets.

This is an electrical schematic draft for a rover with four powered corner wheels, two passive middle wheels, one eight-AA battery pack, two buck converters, a Black Pill, an ESP32, two dual motor-driver modules, two steering servos and an HC-SR04.

It is not a released PCB design or a verified wiring instruction for unidentified modules. Physical module header pad numbers and all PCB footprints remain unassigned. Module pins deliberately use their functional labels, such as PA6, IN1 and OUT+, rather than invented physical pad numbers. For connectors J1-J9, the numbers are the intended carrier connector order, not a guarantee about an existing cable.

## What each sheet does

| Sheet | Contents |
| --- | --- |
| 01 Power | One battery input; fuse, master switch and reverse-polarity diode positions; two buck modules; motor-only power cut; bulk and bypass capacitors; rail LEDs; nine test points. |
| 02 Control and Sensors | STM32 and ESP32 module interfaces, eight motor-input pulldowns, crossed UART, steering connectors, SWD voltage reference, removable controller power links. |
| 03 Drivers and Sonar | Two MX1616 modules, four motor connectors, driver capacitors, sonar trigger level buffer and echo divider. |

Named nets replace long wires. **Matching green net names are electrically connected.** The project must use **Project > Project Options > Options > Net Identifier Scope > Global (Netlabels and ports global)**. The saved project uses this setting. Opening a sheet alone outside the project does not establish connections to the other sheets.

## Preserved firmware pins, revised module wiring

The current public firmware was read from `firmware/stm32_body/include/board.hpp`. Its blob SHA was `a6f5cd72b93e55d63336526db130abfad8c0ee1c`. The existing four corner-motor pins were preserved. The four-motor schematic reorganizes the driver modules by side:

| Wheel | STM32 IN1 / IN2 | Driver and channel | Motor connector |
| --- | --- | --- | --- |
| Front left | PA6 / PA7 | U3, A (IN1 / IN2) | J5 |
| Rear left | PB6 / PB7 | U3, B (IN3 / IN4) | J6 |
| Front right | PB8 / PB9 | U4, A (IN1 / IN2) | J7 |
| Rear right | PB10 / PA3 | U4, B (IN3 / IN4) | J8 |

The repository still describes six driven motors and two packs. This design does not modify the repository. Its wiring documentation and four-wheel behavior need updating and testing. Unused middle-wheel outputs PB0, PB1, PA5 and PA1 have no connection on this draft. Do not accidentally wire a rear wheel to a middle-wheel firmware output.

Other preserved pins: left servo PA8, right servo PA2, TRIG PB12, ECHO PB13, UART TX PA9 to ESP32 GPIO16/RX, UART RX PA10 from GPIO17/TX. PA13 and PA14 remain SWD. The ESP32 variant must expose usable GPIO16/17; a WROVER/PSRAM variant may conflict.

## Design choices and arithmetic

- **Single pack, two converters:** the motor regulator and system regulator share the battery and ground. Separate regulators do not eliminate battery sag, shared-ground noise or reset risk. The eight-AA chemistry, usable voltage range and holder current capacity are not yet confirmed.
- **Motor current estimate:** the selected X2 SK51S motor listing reports 0.8 A stalled at 4.5 V and 1.0 A at 6 V. Four times 1 A gives a conservative 4 A preliminary motor rail target near 5 V, not a measurement of these four motors. The listing's 5 A buck limit is a maximum claim, not proof of continuous performance in the rover.
- **System estimate:** using the earlier brief's unverified 0.8 A per servo, 0.5 A ESP32, 0.05 A STM32 and 0.015 A sonar gives 2.165 A. This is not an established worst-case servo current. Measure or obtain credible stall-current specifications before finalizing protection.
- **Battery estimate:** 5 x (4 + 2.165) = 30.825 W output. At an assumed 85% efficiency and nominal 9.6 V NiMH pack, input is 30.825 / (0.85 x 9.6) = 3.78 A, before reverse-diode losses. At 7 V it would be about 5.18 A. Pack, holder, leads, diode and switches must support the real conditions. F1 and the power protection parts deliberately remain TBD; no fuse rating was guessed.
- **Echo divider:** R1 = 2.2 kohm, R2 = 3.3 kohm, both 1%. At 5 V, PB13 sees 5 x 3.3 / (2.2 + 3.3) = 3.00 V. At 5.25 V and worst resistor ratio, 5.25 x 3333 / (2178 + 3333) = 3.18 V. This replaces the brief's 1 kohm / 2 kohm divider to give more upper-voltage margin. Verify the real sensor's HIGH voltage and PB13 HIGH threshold.
- **Trigger buffer:** U7 is an SN74AHCT1G125 powered by 5V_SYS. Its verified TI pin functions are 1 = active-low OE, 2 = A, 3 = GND, 4 = Y, 5 = VCC. OE is grounded. It provides a 5 V trigger from the 3.3 V STM32 output. A compatible breakout may be used for a wired prototype; a carrier footprint is not assigned.
- **Reset behavior:** R5-R12 are 10 kohm pulldowns on the eight driver inputs. Each loads a 3.3 V HIGH by about 0.33 mA and gives a defined low state when the STM32 outputs are high impedance. Verify the actual module truth table and its internal input circuitry. These resistors do not implement an independent safety controller.
- **Power LEDs:** R3/R4 are 1 kohm. For an assumed 2 V LED drop, current is (5 - 2) / 1000 = 3 mA and resistor dissipation is 9 mW. Select the actual LED and verify its voltage/current rating. D2/D3 are LEDs; D1 is reverse-polarity protection.
- **Capacitors:** bulk values are starting values, not a demonstrated fix for resets or motor transients. Polarized capacitors must have their positive terminal on the indicated positive rail. Capacitor pin 1 is the intended positive contact if a polarized part is selected. Voltage rating, ripple rating, ESR and actual footprint still need selecting. Optional motor suppression capacitors should be fitted at the motor terminals, not only far away on a carrier.

## USB programming

JP1 and JP2 are removable two-pin power links, not automatic USB power selectors. Battery power must be off before programming. Remove the relevant 5 V link before connecting that controller's USB cable. Do not connect external 5 V and USB together until the exact module's power schematic has been checked. Avoid signal back-powering through UART, sonar or debug lines when a module is unpowered; disconnect those signal cables if required. SWD's 3V3 contact is target voltage reference, not permission to inject a second 3.3 V supply.

## Items that prevent release

1. Identify the actual Black Pill revision, ESP32 variant, MX1616/TC1508A chips and buck modules. Match their silk labels to the functional pins here. Confirm 3.3 V logic input compatibility and PWM/brake behavior.
2. Obtain module dimensions and header positions. Map the logical module pins to actual carrier pads, then assign verified footprints. No nominal module dimensions have been substituted.
3. Confirm AA chemistry, loaded voltage, stall currents, regulator temperature and current capability, holder and wire ratings. Select F1, D1, SW1 and SW2 from those results. Check motor braking transients and any needed clamp/snubber against actual measurements and driver limits. SW2 is a motor supply cut, not a certified emergency stop.
4. Confirm servo connector order and actual servo current. J2/J3's intended order is GND, 5 V, signal; an existing servo cable may need a different physical arrangement.
5. Update the repo's six-wheel/two-pack assumptions and test four-motor control with wheels raised. Confirm direction independently for every motor. Encoder-free wheel motion is not verified closed-loop speed or position control.
6. Complete an electrical review and Altium validation with meaningful pin types, then assign footprints and perform PCB layout/DRC if a PCB is wanted. A clean connectivity check alone does not establish electrical safety or thermal performance.

Battery sensing, an IMU interface and extra expansion headers are not implemented in this draft; the controller has no validated pack undervoltage shutdown here. They require an agreed specification and verified voltage ranges. No PCB layout, Gerbers, drill or manufacturing release is included.

## How to learn from it

Start on sheet 01. Pick net `5V_SYS` and find it on the other sheets: it feeds the servos, sonar buffer and power links. Then trace `M_FL_IN1` from STM32 PA6 through its pulldown to driver U3 IN1. Finally follow `SONAR_ECHO_5V` through R1/R2 to PB13. That shows power distribution, a motor control signal and a voltage interface without needing to understand every symbol at once.

## Checks completed

- All three schematic sheets were opened in Altium Designer 26.11.1 and saved by Altium in native SCH Binary Version 5.0 format.
- Project net scope was set to Global and saved in the project.
- The source geometry and named-net audit checked all 149 pins and 39 nets, including the four motor pairs and UART crossover. It found no unlabeled pins, unintended joined net names, or mismatches in the checked connection groups. `Connectivity_Audit.json` records the source audit; it is not an Altium ERC report.
- Altium project validation at 5:11 PM Toronto time on November 18, 2026 reported **Compile successful, no errors found**. It reported **57 warnings**: 55 unassigned component footprints, plus no driving source on the passive echo-divider output and externally driven SWCLK. Those two driver warnings reflect what the logical symbols model: a passive divider and an external debugger. They were reviewed and left visible; no broad No-ERC suppression was added. This is not an ERC-clean manufacturing release.
- `Rover_Carrier.pdf` was exported directly from Altium Smart PDF and contains the three schematic sheets. `Rover_Carrier.OutJob` stores its export settings; check the destination path before rerunning it from a different folder.

No physical assembly, current testing, sensor threshold measurement, thermal testing or PCB DRC was performed.

## Sources checked

- Firmware pin map: https://github.com/sarbfatehsuppal/robotrover/blob/main/firmware/stm32_body/include/board.hpp
- Existing wiring (six-motor/two-pack version): https://github.com/sarbfatehsuppal/robotrover/blob/main/docs/wiring.md
- WeAct product documentation: https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1
- WeAct timer mapping: https://blog.weactstudio.com/en/posts/stm32f411ce-micropython-tutorial/
- STM32 product/datasheet entry: https://www.st.com/en/microcontrollers-microprocessors/stm32f411ce.html
- ESP32 power-source restrictions: https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html
- TI buffer datasheet: https://www.ti.com/lit/ds/symlink/sn74ahct1g125.pdf
- Motor supplier's measurements: https://x2robotics.ca/tt-motor-all-metal-gearbox-1-90-gear-ratio
- Buck supplier's listing: https://x2robotics.ca/dc-dc-mp2482-sy8205-5a-step-down-buck-module
- Motor-driver supplier's variant notes (not a substitute for identifying the installed IC): https://www.universal-solder.ca/product/h-bridge-dc-motor-driver-mx1616-or-tc1508a-2-7v-output-1-3-1-8a-continuous/
- Altium net-label scope: https://www.altium.com/documentation/altium-designer/schematic/creating-circuit-connectivity
