# industrieroboter-roboterhand

Software of the robot hand (group project *Industrieroboter*, TU Graz):
firmware for an **Arduino Mega 2560** or an **ESP32** that drives the five
finger servos (and later reads strain gauges), and a **ROS 2 Humble** workspace
for the hardware interface, the URDF model with TF, a control/visualization GUI
including the **Denavit-Hartenberg** kinematics, and a bringup launch file.

```
 ┌────────────── PC (Ubuntu 22.04 / WSL 2, ROS 2 Humble) ────────────────┐
 │                                                                        │
 │  robothand_gui ──closure/joint_command──► robothand_interface ◄─USB──┼──► Arduino Mega / ESP32
 │      ▲   ▲                                   │  │  │                   │      5 servos, strain gauges
 │      │   └──────── robothand/state, strain_gauges                       │
 │      │                                       ▼                          │
 │      └── TF ◄── robot_state_publisher ◄── /joint_states                │
 │                        (URDF)                │                          │
 │                          └──────────► RViz2 ◄┘                          │
 └────────────────────────────────────────────────────────────────────────┘
```

## Repository layout

| Path | Content |
|---|---|
| `embedded_software/arduino_mega_2560/` | Firmware for the Arduino Mega 2560 (`config.h` = pins, pulse widths, limits) |
| `embedded_software/esp32/` | Same firmware for the ESP32 (library *ESP32Servo*) |
| `ros2_ws/src/robothand_msgs` | Messages `HandState`, `StrainGauges` |
| `ros2_ws/src/robothand_interface` | Node `robothand_interface`: ROS 2 ⇄ serial; firmware emulator for tests |
| `ros2_ws/src/robothand_description` | URDF/xacro, STL meshes, RViz config, DH parameters, `robot_state_publisher` launch |
| `ros2_ws/src/robothand_gui` | Qt GUI: finger control, presets, motion sequence, live plots, DH kinematics |
| `ros2_ws/src/robothand_bringup` | `robothand.launch.py` – starts everything |
| `ros2_ws/config/` | DDS configuration for WSL 2 (see [WSL notes](#wsl-2-notes)) |
| `tools/serial_tcp_bridge.py` | Serial ⇄ TCP bridge (use a Windows COM port from WSL without usbipd) |

## Quick start

```bash
# once: dependencies (ROS 2 Humble desktop is assumed)
sudo apt install python3-serial ros-humble-joint-state-publisher-gui ros-humble-xacro \
                 ros-humble-rmw-cyclonedds-cpp

cd ros2_ws
colcon build --symlink-install
source install/setup.bash
source config/dds_localhost.sh          # only needed in WSL 2, see below

# without hardware (emulated microcontroller)
ros2 launch robothand_bringup robothand.launch.py simulate:=true

# with the real hand (Arduino Mega with CH340 / ESP32: /dev/ttyUSB0, original Arduino: /dev/ttyACM0)
ros2 launch robothand_bringup robothand.launch.py port:=/dev/ttyUSB0

# only the model in RViz with joint sliders (no interface, no GUI)
ros2 launch robothand_description display.launch.py
```

Launch arguments of `robothand.launch.py`: `simulate`, `port`, `baudrate`,
`params_file` (calibration, default `robothand_interface/config/robothand_interface.yaml`),
`gui`, `rviz`, `rviz_config`.

Your user needs access to the serial port: `sudo usermod -aG dialout $USER` (log in again).

## Firmware

Flash with the Arduino IDE (open `embedded_software/arduino_mega_2560/arduino_mega_2560.ino`
or `embedded_software/esp32/esp32.ino`) or with `arduino-cli`:

```bash
arduino-cli compile --fqbn arduino:avr:mega embedded_software/arduino_mega_2560
arduino-cli upload  --fqbn arduino:avr:mega -p COM5 embedded_software/arduino_mega_2560
arduino-cli compile --fqbn esp32:esp32:esp32 embedded_software/esp32   # needs library "ESP32Servo"
```

### Wiring (defaults in `config.h`)

| Servo | Finger | Mega pin | ESP32 GPIO | Strain gauge (analog) Mega / ESP32 | HX711 DOUT Mega / ESP32 |
|---|---|---|---|---|---|
| 0 | thumb  | 2 | 18 | A0 / 36 | 22 / 36 |
| 1 | index  | 3 | 19 | A1 / 39 | 24 / 39 |
| 2 | middle | 4 | 23 | A2 / 34 | 26 / 34 |
| 3 | ring   | 5 | 25 | A3 / 35 | 28 / 35 |
| 4 | little | 6 | 26 | A4 / 32 | 30 / 32 |
| | | | | | HX711 SCK: Mega 23 / ESP32 33 |

* Power the servos from a **separate 5–6 V supply** (5 × MG996R can draw several A);
  connect its GND with the board GND. Never power them from the USB 5 V pin.
* The ESP32 has 3.3 V logic (most hobby servos accept it); analog inputs max. 3.3 V.
* Strain gauge source: `STRAIN_GAUGE_SOURCE` in `config.h` = `SG_SOURCE_NONE`,
  `SG_SOURCE_ANALOG` (amplified bridge on ADC pins, default) or `SG_SOURCE_HX711`
  (one HX711 per gauge, shared clock line).
* Safety: hard servo limits (`SERVO_LIMIT_*_DEG`), speed ramp (`V` command),
  optional command watchdog (`COMMAND_TIMEOUT_MS`). The servos are limp after
  power-up until the first command arrives, and are detached when the ROS node stops.

### Serial protocol (115200 baud, ASCII lines)

| PC → board | Meaning |
|---|---|
| `S a0 a1 a2 a3 a4` | target angles of all servos [deg] (attaches the servos) |
| `P i a` | target angle of servo *i* |
| `E 0` / `E 1` | detach (limp) / attach servos |
| `V deg_per_s` | max. speed, `0` = no ramp |
| `R hz` | telemetry rate (0 = off, default 50 Hz) |
| `H`, `T`, `Q`, `I` | home position, tare gauges, one telemetry frame, board info |

| Board → PC | Meaning |
|---|---|
| `I ROBOTHAND <board> <fw> <n_servos> <n_gauges> <sg_source>` | info (also sent at boot) |
| `A <t_ms> <enabled> c0 … c4` | current (ramped) servo angles [deg] |
| `G <t_ms> g0 … g4` | strain gauge values (raw, tared) |
| `OK <cmd>` / `ERR <text>` / `# <text>` | acknowledge / error / log |

The protocol can be tried with any serial monitor (newline line ending).

## ROS 2 interface

| Name | Type | Direction | Description |
|---|---|---|---|
| `robothand/closure_command` | `std_msgs/Float64MultiArray` | in | 5 values 0 … 1 (thumb, index, middle, ring, little), `NaN` = keep |
| `robothand/joint_command` | `sensor_msgs/JointState` | in | target angles [rad] of `*_mcp_joint` and `thumb_rotation_joint` (partial messages ok) |
| `robothand/servo_command` | `std_msgs/Float64MultiArray` | in | raw servo angles [deg], for calibration |
| `joint_states` | `sensor_msgs/JointState` | out | all 15 URDF joints (from the servo feedback, coupled joints computed via `<mimic>`) |
| `robothand/state` | `robothand_msgs/HandState` | out | connection, firmware, servo angles/targets, closure |
| `robothand/strain_gauges` | `robothand_msgs/StrainGauges` | out | raw and calibrated values (`value = raw * gauge_scale + gauge_offset`) |
| `robothand/enable` | `std_srvs/SetBool` | service | attach / detach servos (while disabled no targets are sent) |
| `robothand/home` | `std_srvs/Trigger` | service | open all fingers |
| `robothand/tare` | `std_srvs/Trigger` | service | zero the strain gauges |

Examples:

```bash
ros2 topic pub --once /robothand/closure_command std_msgs/msg/Float64MultiArray "{data: [0.5, 1.0, 1.0, 0.0, 0.0]}"
ros2 service call /robothand/enable std_srvs/srv/SetBool "{data: false}"
ros2 topic echo /robothand/strain_gauges
```

### Calibration

Each finger has one servo with a tendon. In `robothand_interface/config/robothand_interface.yaml`:

* `servo_open_deg` / `servo_closed_deg`: servo angle of the open / fully closed finger
  (swap the two values if a servo turns the other way). Find them with
  `robothand/servo_command` or the `P` command in a serial monitor.
* `joint_closed_rad`: URDF angle of the actuated joint at "closed"
  (= upper joint limit in the URDF, thumb 90°, fingers 85°).
* `passive_joints` / `passive_joint_positions`: the thumb rotation has no servo;
  set its real (manually adjusted) angle here or from the GUI.
* `gauge_scale`, `gauge_offset`, `gauge_unit`: calibration of the strain gauges.

## Kinematic model

The URDF (`robothand_description/urdf/robothand.urdf.xacro`) was derived from the
STEP assembly `Hand.stp`: the joint axes and pivots are the pin bores of the
parts (endteil: pivots at y = −7 / 37 mm, mittelteil: −7 / 27 mm, Fingerkuppe: −2 mm),
the MCP pivots are the knuckle bores of the palm. With the CAD joint angles the
URDF reproduces every CAD part pose exactly. Frames: x across the palm (thumb at +x),
y along the fingers, palm side = −z.

* Long fingers: `*_mcp_joint` (servo), `*_pip_joint` and `*_dip_joint` coupled by
  `<mimic>` (PIP = 1.0 × MCP, DIP = 0.7 × MCP, adjustable at the top of the xacro).
* Thumb: `thumb_rotation_joint` (rotation of *Daumen_Grundgelenk* about an axis
  parallel to y, passive), `thumb_mcp_joint` (servo), `thumb_ip_joint` (mimic).
* The little finger uses a *mittelteil* as proximal phalanx (34 mm instead of 44 mm).

### Denavit-Hartenberg parameters

Standard DH convention, `A_i = Rot_z(θ_i) · Trans_z(d_i) · Trans_x(a_i) · Rot_x(α_i)`,
`θ_i = q_i + offset_i`, `T_palm_tip = T_palm_0 · A_1 · A_2 · A_3`
(full table incl. the base transforms `T_palm_0`: `robothand_description/config/dh_parameters.yaml`).

| Chain | i | joint | θ offset | d [mm] | a [mm] | α |
|---|---|---|---|---|---|---|
| index / middle / ring | 1 | MCP | 0 | 0 | 44.0 | 0 |
| | 2 | PIP | 0 | 0 | 34.0 | 0 |
| | 3 | DIP | 0 | 0 | 36.7 | 0 |
| little | 1–3 | MCP, PIP, DIP | 0 | 0 | 34.0, 34.0, 36.7 | 0 |
| thumb | 1 | rotation | 0 | 30.0 | 36.21 | −90° |
| | 2 | MCP | −90° | 0 | 44.0 | 0 |
| | 3 | IP | 0 | 0 | 36.7 | 0 |

The GUI tab *Kinematics (DH)* shows the table with the current joint angles, all
`A_i` matrices, `T_palm_tip` and compares the DH tip position with the TF
transform from `robot_state_publisher` (difference 0.000 mm). The unit test
`robothand_gui/test/test_kinematics.py` checks the DH table against the URDF for
random joint angles (tip position, joint axes, frame origins).

## GUI

`ros2 run robothand_gui robothand_gui` (started by the bringup launch):

* **Control** – sliders per finger (closure 0–100 %), thumb rotation, grasp presets
  (open, fist, pinch, tripod, cylinder, point, …), demo motion sequence with smooth
  interpolation, enable/disable servos, connection status.
* **Joint angles** – live plot of the MCP angles and table of all joints.
* **Strain gauges** – live plot and bar chart, tare button.
* **Kinematics (DH)** – see above, 3D view of all DH chains and frames.

## Tests

```bash
cd ros2_ws && colcon test && colcon test-result --verbose
```

28 tests: protocol parser, servo/joint mapping and mimic expansion, firmware
emulator, URDF consistency, DH vs. URDF. Without hardware, the complete serial
path can be tested with a virtual serial device:

```bash
ros2 run robothand_interface fake_mcu            # creates /tmp/ttyROBOTHAND
ros2 launch robothand_bringup robothand.launch.py port:=/tmp/ttyROBOTHAND
```

## WSL 2 notes

* **DDS:** with `networkingMode=mirrored`, the loopback interface is not multicast
  capable and UDP datagrams larger than ~1.4 kB are dropped. Symptom: nodes do not
  see each other, or `/tf` (15 transforms) never arrives, so RViz shows no
  robot pose. Fix: use CycloneDDS with the provided localhost configuration in
  **every** terminal: `source ros2_ws/config/dds_localhost.sh`
  (e.g. add it to `~/.bashrc` after sourcing ROS).
* **USB serial:** WSL does not see USB devices by default. Either
  * attach the device with [usbipd-win](https://github.com/dorssel/usbipd-win):
    `usbipd list`, `usbipd bind --busid <id>` (admin, once),
    `usbipd attach --wsl --busid <id>` → `/dev/ttyUSB0`, or
  * run the bridge on Windows: `py -3 tools\serial_tcp_bridge.py COM5`
    (needs `pip install pyserial`) and launch with `port:=socket://localhost:5555`.
* GUI windows (RViz, robothand_gui) work with WSLg out of the box.
