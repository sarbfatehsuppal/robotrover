# robotrover

A six-wheeled rocker-bogie rover, printed in PETG, driven by six TT gear
motors and two microcontrollers. It's a small, slow, very determined cousin
of Curiosity.

We wanted three things from it:

1. Drive it from a laptop.
2. Have it climb over things about as tall as its wheels (65 mm) without
   any clever control, just the suspension doing its job.
3. Let it follow a planned route on its own, and get out of the way of
   anything that turns up in front of it.

This repo is everything except the plastic: firmware for both chips, the
ROS 2 side for the laptop, a simulator, and a physics model we used to
check the design before printing it.

![The rover climbing a 65 mm step, from the physics model](docs/img/climb_65mm_filmstrip.svg)

## How it fits together

```
   LAPTOP  (ROS 2 Jazzy)                     ROVER
  ┌────────────────────────┐  WiFi    ┌──────────────────────────┐  UART   ┌──────────────────────────┐
  │ keyboard teleop        │ micro-ROS│ ESP32  "the brain"       │ 20 Hz   │ STM32  "the body"        │
  │ mission uploader       │◄────────►│ a ROS 2 node over WiFi   │◄───────►│ 6 motors (3x MX1616)     │
  │ RViz                   │  UDP     │ runs the planned route   │ 115200  │ 2 steering servos        │
  │ micro-ROS agent        │  8888    │ steers round obstacles   │         │ HC-SR04 + safety reflex  │
  └────────────────────────┘          └──────────────────────────┘         └──────────────────────────┘
```

The split is on purpose. The STM32 is the spinal cord: it does the fast,
boring, safety-critical stuff, and it will stop the wheels if the sonar sees
something close or if the ESP32 goes quiet for 300 ms, no matter what anyone
asked for. The ESP32 does the thinking and the WiFi. The laptop is mission
control. If the WiFi drops in the middle of an autonomous run, the rover
keeps going and finishes the route, the way a real Mars rover keeps working
when Earth is out of contact.

The rover logic itself (kinematics, the safety reflex, the UART protocol,
odometry, path following and obstacle avoidance) lives in one place,
[`ros2_ws/src/rover_core`](ros2_ws/src/rover_core/include/rover_core). It's
plain C++17 headers with no ROS in them, compiled into the STM32 firmware,
the ESP32 firmware *and* the simulator. So the code you test on the laptop
is exactly the code that runs on the robot.

## What's in here

```
firmware/
  stm32_body/        STM32F411 Black Pill: motors, servos, sonar, reflex (PlatformIO, STM32Cube HAL)
  esp32_brain/       ESP32: micro-ROS node, mission, avoidance (PlatformIO, Arduino + micro_ros_platformio)
ros2_ws/src/
  rover_core/        the shared C++ brain + body logic
  rover_sil/         runs rover_core on the laptop for the simulator
  rover_sim/         physics: climb analysis tool + live simulated rover (pure Python)
  rover_description/ URDF, geometry, RViz config
  rover_station/     laptop side: keyboard teleop, mission uploader, odom -> TF
  rover_bringup/     launch files, worlds, missions
docs/
  wiring.md          every wire, both power rails
  physics.md         can it actually climb? (yes, just)
  build_notes.md     dimensions, bearings, printing, balancing
```

## Try it in the simulator first

You need Ubuntu 24.04 with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html).
Humble on 22.04 should also work; if you use it on the real rover, set
`board_microros_distro = humble` in the ESP32's `platformio.ini` so the
firmware matches your agent.

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 launch rover_bringup sim.launch.py
```

In a second terminal:

```bash
source ros2_ws/install/setup.bash
ros2 run rover_station teleop
```

RViz opens on a little "Mars yard": a 65 mm curb, a rock, a book, and a few
things too tall to climb. In the simulator the rover starts in manual mode,
so you can drive straight away:

| Key | Does |
|---|---|
| `w` / `s` | faster / slower (and reverse) |
| `a` / `d` | turn left / right |
| space | stop |
| `c` | climb mode on/off: full torque, needed for steps |
| `g` | go: run the planned route on its own |
| `h` | hold: stop everything |
| `r` | restart the route from where it is now |
| `m` | back to manual |

Try driving at the curb slowly without climb mode, then with it. The motors
genuinely don't have the torque at low duty, and the simulator knows that.

Press `g` and it'll drive the route in
[`yard_loop.yaml`](ros2_ws/src/rover_bringup/config/missions/yard_loop.yaml),
climb the curb, go around the crate, roll over whatever low stuff is in
its way, and come home. You can also click **2D Goal Pose** in RViz to send
it to a single point.

![A simulated run of the yard route](docs/img/sim_yard_run.svg)

By default the simulated brain only gets dead-reckoning odometry, like the
real one, so it drifts. `ground_truth:=true` gives it perfect position if
you want to test the logic without the drift.

## Running the real rover

**1. Wire it up.** Everything is in [docs/wiring.md](docs/wiring.md): two
battery packs, two 5 V bucks (one just for the motors), three MX1616 boards,
and a voltage divider on the sonar's ECHO pin.

**2. Flash the STM32.** Install [PlatformIO](https://platformio.org/install/cli)
with its official installer script (not `pip install platformio`; the micro-ROS
library looks for the environment that installer creates), then:

```bash
cd firmware/stm32_body
pio run -t upload            # with an ST-Link
pio run -e dfu -t upload     # or over USB-C: hold BOOT0, tap NRST, let go of BOOT0
```

The blue LED blinks slowly when it's alive and waiting for the ESP32.

**3. Flash the ESP32.**

```bash
cd firmware/esp32_brain
cp include/wifi_secrets.example.h include/wifi_secrets.h    # then edit it
pio run -t upload && pio device monitor
```

`wifi_secrets.h` holds your WiFi name and password and the laptop's IP, and
git ignores it. The first build takes several minutes because PlatformIO
compiles the micro-ROS library from source. It needs `git`, `cmake` and
`python3-venv` installed. The ESP32 only does 2.4 GHz WiFi.

**4. Start the laptop side.**

```bash
ros2 launch rover_bringup rover.launch.py
```

That starts the micro-ROS agent the ESP32 connects to. If you don't have
the `micro_ros_agent` package, run it in Docker and pass `agent:=false`:

```bash
docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888
```

When the ESP32 connects, its LED goes solid and `ros2 topic list` shows the
rover's topics. Then `ros2 run rover_station teleop` and you're driving.

**First power-up, wheels off the ground:**

- It boots in HOLD. The servos centre themselves; now's the time to fit the
  steering arms pointing straight.
- Press `m`, then tap `w` once. All six wheels should turn forward. If one
  turns backward, flip its entry in `kMotorInvert` in
  [`board.hpp`](firmware/stm32_body/include/board.hpp) instead of swapping
  wires.
- Hold your hand 10 cm in front of the sonar and press `w` again. The
  wheels should refuse to go forward. That's the reflex, and it's working.

No laptop handy? The **BOOT button on the ESP32** starts and stops the
stored route. If no mission has been sent, it drives a 1.2 m × 0.8 m
rectangle.

## Topics

The real rover (via the ESP32) and the simulator (via `rover_sil`) use the
same names, so everything on the laptop works with either one.

| Topic | Type | Direction | What |
|---|---|---|---|
| `cmd_vel` | geometry_msgs/Twist | to rover | teleop, used in manual mode |
| `rover/climb` | std_msgs/Bool | to rover | climb mode for teleop |
| `rover/mode` | std_msgs/String | to rover | `hold`, `manual`, `auto`, `restart` |
| `rover/waypoints` | std_msgs/Float32MultiArray | to rover | `[mission_id, loop, x0, y0, x1, y1, ...]` |
| `odom` | nav_msgs/Odometry | from rover | dead reckoning |
| `rover/range` | sensor_msgs/Range | from rover | the sonar |
| `rover/status` | std_msgs/String | from rover | one readable line: mode, waypoint, avoidance state, flags |
| `rover/path` | nav_msgs/Path | laptop | the route, for RViz |

Waypoints go as a flat float array rather than a `nav_msgs/Path` because a
32-point Path is about 2.5 KB, more than micro-ROS's default buffers take
over UDP. The array is 264 bytes.

## Planned routes

A mission is a short YAML file, in metres, measured from wherever the rover
was switched on (x forward, y to the left):

```yaml
name: hallway and back
loop: false
waypoints:
  - [1.5, 0.0]
  - [1.5, 1.0]
  - [0.0, 1.0]
  - [0.0, 0.0]
```

```bash
ros2 launch rover_bringup rover.launch.py mission:=/path/to/hallway.yaml
```

The ESP32 follows it with pure pursuit. When the sonar sees something
within 45 cm, the rover stops, pauses, turns on the spot toward the side
the next waypoint is on until the echo goes long, turns another 30° so the
wheels clear the edge, drives on 55 cm, and then heads straight for the
waypoint again. It remembers for 20 seconds where it saw things up close, so
it doesn't turn straight back into a corner it can no longer see. If it
can't find a way round after four tries, it skips that waypoint and carries
on with the next.

## The physics, short version

The full story with figures is in [docs/physics.md](docs/physics.md). The
headline numbers for a 1.5 kg rover on 5 V TT motors:

- A 65 mm step needs **84 % of each motor's stall torque** at the worst
  moment (the rear wheel going up) and **tyre grip of μ ≥ 0.75**. Dry rubber
  on wood or concrete is about 0.8, so it makes it, without much to spare.
- It needs **at least 85 % PWM duty**. That's why there's a climb mode, and
  why autonomous mode always uses it.
- The model moved the bogie pin 10 mm down and 20 mm forward from where we
  first drew it; otherwise the middle wheel lifts off at the start of a
  climb.
- The **centre of mass has to be within about 1 cm of the rocker pivots**.
  Balance the batteries.
- Dropping one battery pack, or running the motors at 6 V, would each buy a
  good chunk of margin.

## Things that will probably bite you

- **Slow means weak.** Without encoders, a slow speed is just a low duty,
  and a low duty has no torque. Use climb mode for anything bumpy.
- **It doesn't know where it is very well.** Odometry is worked out from
  what the motors were told to do, so slips and climbs make it drift.
  In the simulator it's about 35 cm off after a 6 m loop.
- **The sonar is a narrow cone.** It can miss a box corner right in front of
  a wheel. The rover remembers recent close echoes, and a per-leg timer
  notices when it has been stuck for too long and makes it back off and look
  again.
- **It can't tell it's stuck.** Same reason: no encoders, no IMU.
- **Brown-outs look like bugs.** If the ESP32 keeps rebooting when the
  motors start, check that the motors are on their own pack and buck.

## What we'd add next

- Encoders on the back shafts of two TT motors (they're dual-shaft for a
  reason). Real speed control would fix most of the list above.
- A cheap IMU, so the rover knows when it's climbing or tipping.
- Two more sonars angled ±30°.
- A battery voltage divider on PA4.
- A Gazebo world, for full 3-D physics instead of our side-view model.

## Parts

- WeAct STM32F411CEU6 "Black Pill" (100 MHz, 512 KB flash)
- ESP32 dev board
- 3 × MX1616 dual H-bridge motor driver (2-10 V, ~1.5 A per channel)
- 6 × TT gear motor, 1:90 all-metal gearbox, 3-6 V, ~110 rpm at 6 V
- 6 × 65 mm rubber TT wheels
- 2 × 8×AA battery holders, 16 AA cells (NiMH recommended)
- 2 × MP2482 5 A step-down buck (7-24 V in, 5 V out)
- 2 × micro servos for front steering (SG90 / MG90S size)
- HC-SR04 ultrasonic sensor, plus a 1 kΩ and a 2 kΩ resistor
- Ball bearings for the pivots, PETG for everything else

---

If you build one, or find something we got wrong (especially in the
physics), please open an issue. We'd genuinely love to see it climb
something.

MIT licensed, see [LICENSE](LICENSE).
