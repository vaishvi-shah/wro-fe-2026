# Electrocute — WRO Future Engineers 2026

## World Robot Olympiad — Future Engineers

**Team:** Vaishvi Shah, Avani Devalankar, Jaitra Bhatt
**Robot:** Batmobile
**Category:** WRO Future Engineers 2026

---

## Introduction

Hello! We are a team of three high school students participating in the **World Robot Olympiad (WRO) Future Engineers 2026** category.

Our team combines software, hardware, mechanical design, and documentation to develop an autonomous robot capable of navigating the competition field and completing its challenges.

### Team Roles

| Team Member      | Role                 |
| ---------------- | -------------------- |
| Vaishvi Shah     | Software & Logic     |
| Avani Devalankar | 3D Design & Hardware |
| Jaitra Bhatt     | Documentation        |

Although we each have individual responsibilities, our development process is collaborative. We work together to test ideas, identify problems, refine our design, and make engineering decisions based on the robot's actual performance.

Our goal through WRO Future Engineers is not only to compete, but also to strengthen our skills in robotics, programming, engineering, problem-solving, and teamwork.

This documentation describes our development process from our initial concepts and prototypes through the final robot, including the problems we encountered, the engineering decisions we made, and the improvements we implemented.

---

# Meet Batmobile

Batmobile is our autonomous car designed and 3D-printed for **WRO Future Engineers 2026**.

At the heart of the robot is a **Raspberry Pi 5**, which processes the camera feed using OpenCV and handles navigation, obstacle detection, and sensor logic.

A **Raspberry Pi Pico 2W** communicates with the Pi through USB and acts as the real-time controller for the motors, steering servo, and IMU.

### Main Sensors

Batmobile uses three primary sensing systems:

* **Pi Camera 3 Wide** — visual navigation, wall detection, lane detection, and pillar detection
* **BNO055 IMU** — heading control, straight-line correction, and parking turns
* **TOF sensor** — close-range detection and parking assistance

Together, these systems allow Batmobile to perceive its surroundings, make navigation decisions, and control its movement autonomously.

> **Robot photos:** Add final robot photos to `docs/team_photos/` and link them here.

---

# Preliminary Work

The public WRO repositories were especially useful when we started developing Batmobile. Looking at documentation from previous teams gave us a reference for how to approach the competition without having to develop everything from scratch.

We have tried to follow the same philosophy with our documentation by making it practical and detailed enough that another team can use the repository as a starting point for building their own robot.

---

# 1. Mobility & Mechanical Design

## 1.1 Chassis Overview

The chassis uses a layered structure.

The first layer contains heavier and higher-current components, while the second elevated layer contains more sensitive sensing and control electronics.

This two-layer architecture was not part of the original plan. It developed from physical space and interference problems discovered during assembly and testing.

---

## 1.2 Chassis Iterations

### Iteration 1 — LEGO Chassis

Our first chassis was built using LEGO components.

#### Starting Design

* Built the initial chassis using LEGO components.
* Used a LEGO-based drivetrain and differential.
* Used the first version to test basic movement and steering.

#### Problems Identified

Several mechanical problems appeared during testing:

* Wheels frequently came off the chassis.
* Steering was difficult because the LEGO geometry limited the available Ackermann steering angle.
* The limited steering angle made parking difficult.
* The differential repeatedly came off the axle.
* The LEGO drivetrain introduced mechanical limitations that made precise steering difficult.

The limited Ackermann geometry meant that the robot could not make sufficiently tight turns for reliable parking.

#### Decision

The LEGO chassis was useful for initial testing, but its mechanical limitations made it unsuitable for reliable steering, parking, and drivetrain operation.

---

## 1.3 Iteration 2 — 3D-Printed Chassis

We replaced the LEGO chassis with a custom 3D-printed design.

### Two-Layer Layout

| Layer   | Components                                              |
| ------- | ------------------------------------------------------- |
| Layer 1 | Battery, motor driver, servo, Ackermann steering system |
| Layer 2 | Raspberry Pi Pico 2W, Raspberry Pi 5                    |

The two-layer structure provided dedicated mounting locations for both electronics and drivetrain components.

### Problems Identified

The battery was initially positioned toward the rear of the chassis.

This created an uneven weight distribution and shifted the centre of mass toward the rear.

As a result:

* Less normal force was available at the front wheels.
* Front-wheel traction decreased.
* At higher speeds and during turns, the front wheels could lose traction and lift slightly.
* Steering became less stable and predictable.

The relevant relationship is:

**F = μN**

where:

* **F** = maximum frictional force
* **μ** = coefficient of friction
* **N** = normal force on the front wheels

Because the battery position affected the distribution of normal force, its placement directly affected steering and traction.

---

# 1.4 Iteration 3 — Final Physical Fit

After 3D-printing the parts, we discovered several physical mismatches that were not apparent from the previous CAD iteration.

Each issue was addressed and validated against the real hardware.

### Chassis & Motor Fit

#### Motor Wire Opening

The motor wire opening was reoriented after the original opening caused the wiring to bind against the chassis.

#### Motor Length

The chassis was extended after we confirmed that the physical DC motor was longer than the CAD model.

#### Motor Mounting Access

An access hole was added for the DC motor mounting screws.

#### Motor Clamp

The motor clamp geometry was adjusted to provide a tighter fit, including reducing its diameter by **0.25 mm**.

---

## Chassis & Weight Distribution

The front of the chassis was extended to create additional space for parking and maneuvering.

### Engineering Trade-Off

We chose to extend the chassis to match the physical DC motor rather than resize the CAD assumption to fit the motor.

The actual component in hand was treated as the more reliable design reference than the original CAD estimate.

### Other Changes

* Relocated the battery to the centre of the chassis.
* Mounted the battery flat to improve balance.
* Added dedicated space for the updated DC motor dimensions.
* Replaced the previous motor-driver configuration.
* Adjusted the chassis layout around the new electronics.

---

## Mounting & Fastening

### LEGO Peg Holes

The LEGO peg holes were increased by **0.1 mm** after the original holes were found to be slightly undersized.

### Servo Mounting

Dedicated mounting holes were added to secure the servo to the chassis.

### Clamp Validation

Every clamp was test-fitted against the physical components before finalizing the design.

This addressed a previous gap where component fit had not been physically verified before printing.

---

# 1.5 Mechanical Connections

Two custom mechanical connections were designed and finalized.

### Motor → Drive Gear

A custom connector mechanically couples the DC motor to the drive gear.

### Differential → Wheel Axle

A second custom connector couples the differential to the wheel axle.

With these connections mechanically finalized, the wiring harness could follow a fixed and repeatable route instead of being improvised for every build.

---

# 1.6 Motor & Differential Iterations

## Motor Iteration 1 — LEGO Differential

Our first drivetrain used a LEGO differential to drive the rear wheels instead of a fixed axle.

The differential allowed the inside wheel to rotate at a different speed from the outside wheel during turns, reducing wheel scrub and drag.

### Problems

During testing:

* The differential axle repeatedly came loose during sharper turns.
* The LEGO gear train had noticeable mechanical slack.
* Mechanical slack absorbed some of the servo's movement before it reached the wheels.
* The LEGO chassis limited how much of the MG95S servo movement could be converted into steering angle.
* The resulting minimum turning radius was too large.

### Engineering Decision

Rather than continuing to modify the LEGO system around these limitations, we moved to a more robust differential that could better handle turning loads and provide a more consistent connection between the motor and wheels.

---

# 1.7 Motor Iteration 2 — WLToys 144001 Differential

We replaced the LEGO differential with a **WLToys 144001 differential**.

The differential provides a **2:1 reduction**, meaning two rotations at the differential input produce one rotation at the output.

The new differential provided:

* Smoother movement
* More consistent turning
* Improved mechanical reliability
* Elimination of the axle-loosening problem seen in Iteration 1

---

# 1.8 Gearbox Ratio Selection

We tested four gearbox reduction ratios to balance torque and speed.

| Gear Ratio | Result                                                                           |
| ---------- | -------------------------------------------------------------------------------- |
| 4.4:1      | Higher speed, but insufficient torque for reliable acceleration and loaded turns |
| 9.6:1      | Best balance between torque and speed                                            |
| 46.8:1     | Large torque margin, but vehicle speed became too low                            |
| 100:1      | Very high torque, but significantly too slow for the course                      |

### Final Selection: 9.6:1

The **9.6:1 gearbox** was selected because it provided enough torque for acceleration and loaded turns while keeping the robot fast enough to remain competitive within the fixed three-minute WRO round.

The 46.8:1 and 100:1 options provided more torque, but were too slow for the course.

The motor provides approximately **0.9 kg·cm of stall torque at 1.8 A** in this configuration.

The drivetrain therefore consists of:

**9.6:1 gearbox → 2:1 differential → wheels**

The two reductions were considered together as one drivetrain system rather than independently.

---

# 1.9 Motor Speed

The motor's tachometer-measured speed under load was considered more useful than relying only on the datasheet value.

The motor is rated at approximately **620 RPM at no load**, while testing on the actual drivetrain gave approximately **400 RPM under load**.

This difference of approximately **35–40%** comes from real-world losses including:

* Gearbox friction
* Differential losses
* Tire friction
* Chassis and vehicle mass

For speed and runtime planning, we therefore use the loaded measurement rather than relying on the no-load specification.

### Risk & Mitigation

The approximately 35–40% gap between no-load and loaded motor speed is a known risk.

We address this by using measured loaded RPM for planning instead of assuming that the datasheet's no-load RPM represents actual driving performance.

---

# 1.10 Ackermann Steering

We use **Ackermann steering** together with the differential.

This was chosen not only for mechanical steering performance, but also to reduce mechanical disturbances that could affect IMU heading measurements.

A fixed axle would force both wheels to rotate at the same speed during turns, increasing tire scrub.

Non-Ackermann steering would also increase tire scrub because the front wheels would not follow the correct turning geometry.

Both effects can introduce mechanical disturbances that affect the heading data used by the navigation system.

Therefore, the mechanical design and sensing system were developed together.

---

## Steering System

The MG95S servo was retained, but the original LEGO steering geometry was replaced with a custom Ackermann linkage.

The new linkage makes better use of the servo's usable operating range.

### Servo Range

| Position      | Angle |
| ------------- | ----: |
| Full Left     |   50° |
| Full Straight |   90° |
| Full Right    |  130° |

The larger usable steering range allows tighter turns while maintaining a more predictable steering response.

Ackermann steering is important because:

* The inside wheel travels through a smaller turning circle.
* The outside wheel travels through a larger turning circle.
* The wheels therefore need different steering angles.

This allows the vehicle to corner without unnecessarily dragging the tires across the ground.

### Servo Range Risk

The 50°–130° range was discovered after an early servo bug limited movement to approximately 0°–90°.

The problem was traced to an incorrect PWM conversion.

After correcting the PWM conversion, we verified the 50°–130° range against the physical Ackermann linkage.

This range became the hard limit used by the steering formulas in the software.

---

# 1.11 Steering Signal Flow

The control path is:

```text
Camera
   ↓
Raspberry Pi 5
   ↓
Raspberry Pi Pico 2W
   ↓
PWM
   ↓
MG95S Servo
   ↓
Ackermann Linkage
   ↓
Front Wheels
```

The Raspberry Pi 5 processes camera input and navigation logic.

It sends steering commands to the Pico.

The Pico converts the commands into PWM signals for the MG95S servo.

The servo physically drives the Ackermann steering linkage.

---

# 1.12 Component Placement

| Component  | Layer / Position           | Reason                                                               |
| ---------- | -------------------------- | -------------------------------------------------------------------- |
| Battery    | Layer 1, centre, flat      | Low and central placement improves stability and weight distribution |
| DC Motor   | Layer 1                    | Drives rear axle through differential                                |
| Camera     | Layer 1, front, high       | Provides a long sight line for track and pillar detection            |
| IMU        | Layer 2, away from battery | Reduces magnetic interference and vibration                          |
| Pico 2W    | Layer 2                    | Keeps controller away from high-current wiring                       |
| TOF Sensor | Layer 1, rear              | Provides direct line of sight to the rear edge during parking        |

### Battery

The battery is mounted low and centrally.

Because friction follows:

**F = μN**

battery position affects how normal force is distributed across the wheels.

Our initial rear-mounted battery caused the front wheels to lose traction during turns, so it was moved toward the centre.

### Camera

The camera is mounted at the front and elevated.

It is tilted downward to capture both side regions of interest and the track ahead.

The higher position provides a longer sight line, allowing the robot to detect pillar colours earlier and make smoother steering decisions.

### IMU

The BNO055 is mounted on the second layer away from the battery, motor, and high-current wiring.

This reduces exposure to magnetic interference and vibration.

### TOF Sensor

The TOF sensor is mounted at the rear of the first layer.

It is used for close-range detection during parking and reversing.

It requires a direct line of sight to the rear of the robot. Mounting it on the elevated second layer would shift its sensing axis away from the actual rear edge and could introduce measurement error during precision parking.

---

# 2. Power & Sensor Architecture

# 2.1 Two-Board Architecture

The robot uses two computing boards:

### Raspberry Pi 5

The Pi 5 handles high-level processing:

* Camera processing
* Navigation
* Lane following
* Obstacle avoidance
* State-machine logic
* Decision making

### Raspberry Pi Pico 2W

The Pico acts as the real-time controller.

It:

* Reads the BNO055 IMU
* Generates PWM signals
* Controls the MG95S servo
* Controls the L298N motor driver
* Receives commands from the Raspberry Pi 5
* Sends status information back to the Pi 5

The Pico runs its control program directly from flash rather than running a full operating system.

---

# 2.2 Why Two Boards?

The two-board architecture divides the workload according to each board's strengths.

| Raspberry Pi 5    | Raspberry Pi Pico 2W |
| ----------------- | -------------------- |
| Camera processing | IMU reading          |
| Decision logic    | PWM generation       |
| State machine     | Servo control        |
| Navigation        | Motor commands       |

This separation prevents heavy camera processing from interfering with timing-sensitive steering and motor control.

---

# 2.3 Power Distribution

An external battery supplies the L298N and motor because the Pico cannot safely provide the current required by the drive motor.

The electronics use a regulated supply.

The motor receives power through the motor driver.

All components share a **common ground**, giving every control signal the same electrical reference.

---

# 2.4 Current Draw & Battery Sizing

Battery sizing was calculated from the expected current draw of the individual components.

| Component            | Approx. Current | Supply |
| -------------------- | --------------: | -----: |
| Raspberry Pi Pico 2W |         0.150 A |    5 V |
| BNO055 IMU           |         0.010 A |    5 V |
| L298N logic          |         0.036 A |    5 V |
| MG95S servo          |         0.700 A |    5 V |
| Camera               |         0.250 A |    5 V |

### 5 V Rail

Total current:

**0.150 + 0.010 + 0.036 + 0.700 + 0.250 = 1.146 A**

Power:

**1.146 A × 5 V = 5.73 W**

---

## Buck Converter Efficiency

Assuming approximately **85% efficiency**:

**5.73 W ÷ 0.85 = 6.74 W**

Therefore, approximately **6.74 W** is required from the battery for the 5 V electronics.

---

## Motor Power

The DC gear motor draws approximately **0.600 A at 12 V** through the L298N.

Motor power:

**0.600 A × 12 V = 7.2 W**

---

## Total System Power

**6.74 W + 7.2 W = 13.94 W**

At a nominal 12 V:

**13.94 W ÷ 12 V ≈ 1.16 A**

---

## Runtime

The battery capacity is approximately **8.8 Ah**.

Estimated runtime:

**8.8 Ah ÷ 1.16 A ≈ 7.6 hours**

This is a theoretical continuous runtime at the assumed load.

A WRO competition run is much shorter than this, leaving substantial margin for:

* Voltage sag
* Motor load
* Stall current
* Steering current spikes

Because the robot does not normally operate every component at maximum load simultaneously, actual runtime may be higher.

### Battery Risk

Battery voltage sag near full discharge can reduce motor torque through the L298N.

Our large theoretical runtime margin helps mitigate this.

A future improvement is voltage-compensated PWM scaling.

---

# 2.5 Microcontroller Selection & Development

We chose the **Raspberry Pi Pico 2W** as the real-time controller alongside the Raspberry Pi 5.

The reason was to separate heavy vision processing from fast, predictable low-level control.

---

## Iteration 1 — Electronics Bring-Up

Before building the chassis, we tested communication between:

* Computer
* Controller
* Motor
* Servo

Our first setup used a micro:bit connected to a Raspberry Pi.

The connection remained unreliable after multiple attempts, so we moved to a Raspberry Pi Pico.

Early Pico tests also produced communication errors.

After testing different tools, including Thonny, we identified a major issue: the Pico was sharing power with other components and losing power when current demand increased.

### Solution

The Pico was given a dedicated power source.

This created the first working version of the two-way Pi-to-Pico communication system used in the final architecture.

---

## Servo Debugging

During early testing, the MG95S servo did not move through the expected range.

The issue was traced to an incorrect PWM pulse-width conversion.

After correcting the conversion, the expected steering range was restored and later verified against the physical Ackermann linkage.

---

## Iteration 2 — Layered Electronics

The electronics were divided into two physical layers.

### Layer 1

* Battery
* Motor
* Camera mount

### Layer 2

* IMU
* Pico

Moving the IMU and Pico away from the motor and battery wiring reduced exposure to electrical noise and vibration.

---

## Iteration 3 — Physical Fit & Wiring

After 3D printing, we found that the electronics did not yet have sufficiently defined mounting positions.

We therefore finalized the first-layer component layout with dedicated positions for the electrical components.

We also replaced earlier twisted-wire connections with terminal blocks.

This created more secure connections that are less likely to short or disconnect during a collision.

---

# 2.6 Sensor Selection

| Sensor           | Interface       | Primary Purpose                               |
| ---------------- | --------------- | --------------------------------------------- |
| BNO055 IMU       | I²C             | Heading hold, drift correction, parking turns |
| TOF Sensor       | I²C, address 29 | Close-range wall and obstacle detection       |
| Pi Camera 3 Wide | CSI             | Lane, wall, and pillar detection              |

---

# 2.7 Camera + IMU Fusion

Our initial lane-following system relied only on the camera.

It compared black-pixel counts between left and right regions of interest.

This worked when the track was clearly visible, but short camera blind spots could allow the robot's heading to drift before the camera recovered.

We therefore added the BNO055 IMU.

The camera remains the primary visual reference because it directly observes the track.

The IMU provides heading feedback and helps maintain orientation between camera updates.

The two measurements are blended using a **70/30 weighting**.

The final approach uses:

**70% IMU + 30% camera**

This allows the IMU to maintain heading while the camera continuously corrects the robot's position relative to the walls.

---

# 2.8 TOF Sensor

The TOF sensor provides close-range distance information for situations where visual information alone is less reliable.

It is particularly useful around walls and during parking.

---

# 2.9 Motor Driver

We selected the **L298N** for its simplicity and reliable operation.

Its approximately **1.5–2 V voltage drop** is a known efficiency trade-off.

With a nominal 12 V battery, the motor receives approximately **10–10.5 V** through the driver.

This affects the motor's available torque and is therefore considered in the drivetrain design.

---

# 2.10 Sensor Placement & Calibration

The sensor placement follows the two-layer architecture.

### Layer 1

* Battery
* Motor
* Camera

### Layer 2

* IMU
* Pico

The IMU is positioned away from the motor and high-current wiring to reduce electrical interference and vibration.

This produces more stable heading measurements.

### IMU Calibration

Before each run:

1. The robot is placed stationary at the starting line.
2. The IMU is calibrated.
3. The initial heading reference is established.
4. The state machine begins.

This reduces the effect of gyro drift at the beginning of the run.

---

# 2.11 Wiring Architecture

The low-level control path is:

```text
Raspberry Pi 5
      ↓
Raspberry Pi Pico 2W
      ↓
Actuators
```

The Pi 5 sends movement and steering commands to the Pico.

The Pico generates:

* PWM for the MG95S servo
* Control signals for the L298N

The L298N sits between the Pico and DC motor.

This allows the Pico to send low-current control signals while the motor driver handles the higher motor current.

The motor receives power from the external battery rather than directly from the Pico.

All components share a common ground.

A small onboard status screen was also used during bench testing for live debugging.

---

# 2.12 Circuit Diagram

The complete pin-level circuit diagram is maintained in the team's circuit-diagram.org project.

> **TODO:** Add the circuit diagram link here.

The repository should also contain the exported diagram at:

```text
/wiring/circuit_diagram.png
```

---

# 3. Safety & Reliability

## Main Power Switch

A single main switch powers the robot on and off between runs.

This follows WRO Rule 9.10.

---

## Battery Voltage Floor

The robot uses a 3S Li-ion battery pack with a nominal voltage of approximately 12 V.

The system does not allow the pack to fall below approximately **9 V**.

This helps protect the battery and maintain stable electronics operation.

---

## Secure Wiring

Terminal blocks replaced loose twisted-wire joints.

This reduces the chance of:

* Short circuits
* Disconnected wires
* Wiring failures after bumps or collisions

---

## Front Bumper

A front bumper protects:

* Camera
* Steering linkage

if the robot contacts a pillar or wall.

---

# 4. Bill of Materials

| Component                                         | Purpose                                    |
| ------------------------------------------------- | ------------------------------------------ |
| Raspberry Pi 5                                    | Main decision and vision computer          |
| Raspberry Pi Pico 2W                              | Real-time motor, servo, and IMU controller |
| Sainsmart Wideangle 5MP Camera                    | 160° FoV vision                            |
| MG95S Micro Servo                                 | Ackermann steering actuator                |
| L298N Motor Driver                                | Motor power stage                          |
| JGA25-371 620 RPM 12 V DC Gear Motor with Encoder | Drive motor                                |
| WLToys 144001 Differential                        | Rear-axle differential, 2:1                |
| BNO055 IMU                                        | Heading correction and parking precision   |
| 12 V → 5 V Buck Converter                         | Electronics power regulation               |
| 12 V 3S1P Li-ion Battery Pack, 8800 mAh           | Main power source                          |
| Terminal Blocks                                   | Secure wiring distribution                 |
| Main Power Switch                                 | Main robot power control                   |
| 3D-Printed Chassis                                | Structural frame                           |
| Pico-LCD-1.44 Display                             | Bench-testing status display               |
| M3 Fasteners                                      | Chassis assembly                           |
| LEGO Wheels / Peg Parts                           | Mechanical components                      |
| Misc. Wiring / Connectors                         | Electrical connections                     |

### Known Costs

| Component                            |       Cost |
| ------------------------------------ | ---------: |
| JGA25-371 620 RPM 12 V DC Gear Motor | $12.64 CAD |
| Sainsmart Wideangle 5MP Camera       | $16.99 CAD |
| 12 V → 5 V Buck Converter            |  $5.28 CAD |

> Some costs and purchase links in the original documentation were placeholders and have intentionally not been invented here.

---

# 5. Software Architecture

The software is organized under the `src` directory.

```text
src/
├── openchallenge.py
├── obstaclechallenge.py
├── pico/
│   └── ...
└── lib/
    ├── frames.py
    ├── BNO055 configuration
    └── TOF tester
```

## `openchallenge.py`

Main program for the Open Challenge.

Contains:

* Navigation logic
* Challenge-specific logic
* Wall following
* Turn detection
* Steering control

---

## `obstaclechallenge.py`

Main program for the Obstacle Challenge.

Contains:

* Obstacle detection
* Navigation
* Obstacle avoidance
* Challenge-specific logic

---

## `pico/`

Contains the Raspberry Pi Pico code.

It was used to test and develop communication between the Raspberry Pi 5 and Pico.

---

## `lib/`

Contains reusable sensor and computer-vision code.

### `frames.py`

Contains the `Frame` class and camera-processing functions used for tasks such as:

* Color detection
* Contour detection

### BNO055 Configuration

Contains the code required to configure and communicate with the IMU.

### TOF Tester

Used to test the TOF sensor and its distance readings.

This organization separates:

* Challenge logic
* Microcontroller code
* Sensor code
* Computer vision

This makes individual systems easier to test and modify.

---

# 6. Open Challenge

The Open Challenge uses a combination of camera-based wall following and IMU heading control.

---

## 6.1 Sensor Input

The Raspberry Pi captures a **320 × 240** camera feed and reads the robot's heading from the BNO055 IMU.

The camera uses separate regions of interest to detect:

* Left black wall
* Right black wall
* Blue turn markers
* Orange turn markers

---

## 6.2 Steering

The camera calculates a steering correction from the difference in wall area between the left and right sides.

The IMU calculates a correction from the difference between:

* Current heading
* Desired heading

These corrections are combined using:

**70% IMU + 30% camera**

to produce the final steering angle.

---

## 6.3 IMU Drift

### Risk

Over a long run, IMU drift could gradually reduce the accuracy of wall following and parking.

### Planned Mitigation

We plan to re-zero the IMU during the run when the robot reaches known straight sections of the track.

This provides an additional reference instead of relying only on calibration at the starting line.

---

# 6.4 Turn Detection

Blue markers indicate a **counter-clockwise turn**.

Orange markers indicate a **clockwise turn**.

The first detected colour locks the direction for the run.

Before executing a turn, the robot checks that there is enough space beside the corresponding wall.

The desired heading is then changed by **90°**.

---

# 6.5 Communication & Stopping

The Raspberry Pi calculates steering and speed values and sends them to the Raspberry Pi Pico.

The Pico controls the motor and servo.

After **12 detected line crossings**, the robot enters its stopping sequence.

---

# 7. Software Iterations

## Initial Approach

The first wall-following system used only the camera.

The robot could follow walls but had difficulty maintaining a straight heading over longer distances.

---

## IMU Integration

The BNO055 IMU was added to provide heading feedback.

This allowed the robot to correct its orientation rather than relying entirely on visual information.

---

## Weighting Experiments

We tested different weightings between the camera and IMU steering outputs.

The final result was:

**70% gyro + 30% camera**

This provided the most stable and consistent navigation in testing.

### Why Not 50/50?

The camera provides direct spatial information about the track edges and was considered more trustworthy for moment-to-moment positional correction.

However, the IMU provides valuable stability during short camera blind spots.

Therefore:

* **IMU:** Maintains heading and keeps the robot straight.
* **Camera:** Provides positional correction based on the walls.

---

# 8. Obstacle Management

## 8.1 State Machine

The main loop reads the camera and gyro each frame and dispatches control to the current state.

After **12 line crossings**, the system force-switches to `IN_PARKING`.

The primary states are:

```text
OUT_PARKING
      ↓
WALL_FOLLOW
      ↓
AVOIDING_OBSTACLE
      ↓
TURNING
      ↓
REVERSING
      ↓
IN_PARKING
```

---

## 8.2 `OUT_PARKING`

The robot compares the left and right wall pixel areas.

It steers away from whichever wall is closer for at least one second.

If a matching-colour obstacle is located in the side being turned into, the robot holds its movement.

It then transitions to:

```text
WALL_FOLLOW
```

---

# 8.3 `WALL_FOLLOW`

The robot uses `navigate_wall()` for steering.

The base speed is **87**.

The state:

* Follows the wall
* Watches for obstacles
* Watches for line crossings
* Detects blue/orange turn markers

The first colour detected locks the turn direction for the entire run.

The opposite colour is then ignored for direction selection.

---

# 8.4 `AVOIDING_OBSTACLE`

The robot tracks the nearest red or green obstacle contour.

As the obstacle gets closer, the robot reduces speed.

The tested mapping is approximately:

```text
Obstacle area:
400 px² → speed 87
4000 px² → speed 72
```

The robot steers toward a midpoint between:

* The obstacle's far corner
* The nearest wall pixel

### Passing Direction

* **GREEN → pass left**
* **RED → pass right**

If the obstacle is not on the wrong side, is more than 75% down the frame, and has an area greater than 4000 px², the robot transitions to:

```text
REVERSING
```

If there is no obstacle in view, the robot returns to:

```text
WALL_FOLLOW
```

---

## 8.5 Colour Detection Risk

HSV colour thresholds for pillar detection may not remain reliable under different competition lighting.

### Planned Mitigation

A pre-run colour calibration step is planned so that HSV thresholds can be adjusted to the actual competition lighting environment.

---

# 8.6 `TURNING`

The turning state runs whenever a turn is pending.

A matching-colour obstacle can veto the turn:

* **RED** for counter-clockwise turns
* **GREEN** for clockwise turns

In this situation, the robot crawls straight until the obstacle clears.

An opposite-colour obstacle does not block the turn.

The turn executes once the wall-clearance gate `SAFE_TURN_AREA` allows it.

Control can then transition to `AVOIDING_OBSTACLE` to continue avoiding the obstacle on the new heading.

The system gives up waiting after **1.5 seconds** regardless.

---

# 8.7 `REVERSING`

The robot:

1. Reverses straight for exactly **1 second**.
2. Transitions to `AVOIDING_OBSTACLE`.
3. Reassesses the obstacle.

It does not transition directly back to `WALL_FOLLOW`.

---

# 8.8 `IN_PARKING`

The parking sequence uses `gyro_only_steer()`.

The camera is intentionally not used as the primary steering input during this sequence.

The sequence is:

```text
Reach back wall
      ↓
Initial turn
      ↓
Cleared check
      ↓
Forced 90° turn
      ↓
Final reverse
      ↓
Final turn
      ↓
Hold
```

The current configuration uses:

```text
STOP_AFTER_PARK_CLEARED = True
```

---

# 8.9 Software → Mechanical Design

The parking strategy was designed around the mechanical limitations of the robot.

During the reverse turn, primary control shifts from camera-based positioning to the IMU.

The reason is that close-range parking requires more mechanical precision than camera-based positioning can reliably provide.

Instead of continuously correcting the steering with the camera, we use a forced, pre-tuned turn.

This sacrifices some adaptability in exchange for repeatability.

The approach was selected because small steering hesitations during tight parking caused more error than a committed, pre-tested steering movement.

---

# 9. Steering Methods

| Method                   | Used By                                            | Basis                                                    |
| ------------------------ | -------------------------------------------------- | -------------------------------------------------------- |
| `navigate_wall()`        | `WALL_FOLLOW`, `TURNING`, base `AVOIDING_OBSTACLE` | `0.7 × gyro-P + 0.5 × camera-P`, clamped to 50°–130°     |
| `gyro_only_steer()`      | `IN_PARKING`                                       | Full gyro PD, no camera                                  |
| Direct obstacle steering | `AVOIDING_OBSTACLE`, opposite-colour `TURNING`     | Proportional steering based on pixel error to pass point |

---

## Mechanical → Software Connection

The 50°–130° steering clamp is directly connected to the physical Ackermann linkage.

It was not originally chosen as an arbitrary software limit.

An early servo PWM bug limited movement to approximately 0°–90°.

After correcting the bug, we verified the actual physical steering range and used that measured range throughout the software.

An early electronics bug therefore ended up defining a hard mechanical/software interface used by the navigation system.

---

# 10. Repository Structure

The repository is organized so that a team without prior context can use it to reproduce the robot.

```text
/
├── README.md
│
├── software/
│   ├── README.md
│   ├── pi5/
│   └── pico/
│
├── cad/
│   ├── README.md
│   ├── chassis_iteration1/
│   ├── chassis_iteration2/
│   └── *.stl
│   └── *.step
│
├── wiring/
│   ├── README.md
│   ├── circuit_diagram.png
│   └── pin_map.md
│
└── docs/
    ├── team_photos/
    ├── performance_videos/
    └── dev_log.md
```

---

# 11. Reproducibility

Our goal is for another team to be able to rebuild the robot using the repository without requiring prior knowledge of our development process.

The repository therefore contains:

* Software
* CAD files
* Wiring diagrams
* Pin maps
* Documentation
* Development logs
* Performance videos
* Team photos

---

# 12. Tools Required

To reproduce the robot, the following tools are required:

* 3D printer capable of printing the chassis layers
* Small Phillips/hex screwdriver set
* Soldering iron and solder
* Multimeter
* Folding rule / measuring cube

The measuring tool is useful for checking competition dimension compliance.

---

# 13. Fasteners & Connectivity

Required components include:

* M3 screws and nuts
* Nylon cable ties / zip ties
* Electrical tape
* Jumper wires
* JST / terminal block connectors

The electrical connections include the battery, fuse, switch, and power rail.

---

# 14. Build From Scratch

## Step 1 — Print the Chassis

Print the current corrected chassis version from:

```text
/cad/chassis_iteration2
```

The earlier iteration should be kept for reference but should not be used as the primary build version.

---

## Step 2 — Assemble the Differential

Assemble the WLToys 144001 differential.

---

## Step 3 — Mount the Motor & Steering

Install:

* DC motor
* Ackermann steering linkage
* Printed motor mounts
* Servo

---

## Step 4 — Wire the Power System

Follow the wiring diagram in:

```text
/wiring
```

Pay particular attention to:

* Common ground
* Fuse placement
* Main power switch
* Battery connections
* Motor driver connections

---

## Step 5 — Install Components

Mount:

### Layer 1

* Battery
* Motor
* Camera
* TOF sensor
* Motor driver

### Layer 2

* Raspberry Pi Pico 2W
* BNO055 IMU
* Raspberry Pi 5

The battery should be mounted centrally and flat.

The IMU should remain separated from high-current wiring and the motor.

---

## Step 6 — Flash the Pico

Flash the Raspberry Pi Pico using:

```text
/software/pico
```

---

## Step 7 — Configure the Raspberry Pi 5

Set up the Raspberry Pi 5 using:

```text
/software/pi5
```

Refer to the software folder's README for exact setup commands.

---

## Step 8 — Bench Test

Run the bench test before attempting a full track run.

This allows communication, sensors, steering, and motor operation to be verified before competition testing.

---

# 15. Running the Program

When running the program without a connected display, make sure:

```python
SHOW_VID = False
```

If `SHOW_VID` remains `True`, the program may produce errors without an attached screen because it will attempt to display the camera output.

---

# 16. Running Automatically on Raspberry Pi Startup

The robot can automatically start the challenge program when the Raspberry Pi boots using `crontab`.

Open the root crontab:

```bash
sudo crontab -e
```

Add:

```bash
@reboot /usr/bin/python3 </path/to>/obs_ch.py >/dev/null 2>&1 &
```

Replace:

```text
</path/to>
```

with the actual path to the challenge program.

This starts the Python program automatically whenever the Raspberry Pi boots.

---

## Debugging Startup

By default:

```bash
>/dev/null 2>&1
```

redirects standard output and error messages to `/dev/null`.

For debugging, create a log file:

```bash
sudo touch /dev/cronlog
```

Then modify the crontab command to redirect output to that file.

This allows startup errors and debug messages to be reviewed.

---

# 17. CAD & Wiring Files

All STL files used for the robot's CAD and 3D-printed components are included in the repository.

The wiring diagrams are also provided so that the robot's physical structure and electrical connections can be reproduced alongside the software.

---

# 18. Possible Improvements

## Dynamic Colour Calibration

Our HSV ranges are currently fixed for an entire run.

A pre-run calibration step using the actual competition lighting would reduce pillar-colour misclassification.

---

## Mid-Run IMU Re-Zeroing

The IMU currently uses calibration at the start of the run.

A future improvement is to re-zero the IMU at known straight sections of the track to reduce accumulated drift.

---

## Splitting Pico Workload

The Pico currently handles both IMU reading and motor/servo control.

As the system becomes more complex, a second microcontroller could share this workload.

---

## Voltage-Compensated PWM

Motor PWM could be dynamically adjusted as battery voltage decreases.

This would help maintain more consistent driving performance throughout the battery discharge cycle.

---

## Lateral Distance Sensors

Side-mounted distance sensors could supplement the existing camera/IMU parking approach.

This could reduce reliance on a single pre-tuned parking angle.

---

## Confirmed Motor RPM

The current documentation uses an approximately **400 RPM loaded-speed figure**.

A future improvement is to replace this estimated value with a tachometer-measured value under representative competition load.

---

# 19. Development Log

## Iteration 1 — Electronics Bring-Up

Attempted:

```text
micro:bit ↔ Raspberry Pi
```

The connection was unsuccessful and was abandoned.

We then moved to:

```text
Raspberry Pi Pico ↔ Raspberry Pi
```

Repeated connection errors were initially traced to software issues and eventually to a power-delivery problem.

The Pico was not receiving enough power through the shared rail.

### Solution

A dedicated power source was added for the Pico.

Separately, the steering servo was found to be rotating through only approximately 0°–90° instead of the expected range.

This was traced to a PWM pulse-width mapping bug and corrected.

---

## June 19, 2026

Built the hardware architecture and circuit diagram for:

```text
Pi 5
  ↓
Pico
  ↓
Servo / Motor
```

---

## August 6, 2026

Debugged the Obstacle Challenge.

Problems included:

* Robot not stopping correctly
* Incorrect red-pillar avoidance
* Random **−22° steering offset**

The steering offset was traced and the red-pillar avoidance behavior was fixed.

---

## August 7, 2026

Resolved the remaining bug from August 6.

Then shifted focus toward implementing and testing the parking logic.

---

# 20. WRO Documentation Timeline

Meaningful dated commits are maintained throughout development.

The intended development timeline includes:

1. **First meaningful commit:** at least two months before the competition, containing at least one-fifth of the final code.
2. **Second meaningful commit:** at least one month before the competition.
3. **Third meaningful commit:** at least two weeks before the competition.

Commit messages should describe actual engineering changes.

### Good Examples

```text
fix red pillar avoidance midpoint offset bug
```

```text
add IMU-based parking controller
```

```text
update Ackermann steering geometry
```

### Avoid

```text
update code
```

```text
final
```

```text
stuff
```

---

# 21. Performance Videos

> **TODO:** Add links to:
>
> * Summary video
> * Open Challenge video
> * Obstacle Challenge video
> * Parking demonstration

Recommended location:

```text
/docs/performance_videos/
```

---

# 22. Team Photos

> **TODO:** Add team and robot photos.

Recommended location:

```text
/docs/team_photos/
```

---

# 23. Engineering Summary

Batmobile evolved through multiple hardware and software iterations.

The major design progression was:

```text
LEGO Chassis
      ↓
3D-Printed Chassis
      ↓
Improved Weight Distribution
      ↓
WLToys Differential
      ↓
9.6:1 Gearbox
      ↓
Ackermann Steering
      ↓
Pi 5 + Pico 2W Architecture
      ↓
Camera + IMU Fusion
      ↓
Obstacle State Machine
      ↓
IMU-Based Parking
```

The final system combines:

* Raspberry Pi 5 high-level processing
* Raspberry Pi Pico 2W real-time control
* OpenCV-based computer vision
* BNO055 IMU heading control
* TOF close-range sensing
* Ackermann steering
* WLToys differential
* 9.6:1 motor reduction
* 2:1 differential reduction
* State-machine navigation
* Camera + IMU sensor fusion

The design decisions were made iteratively based on physical testing, software behavior, and the constraints of the WRO Future Engineers course.

---

# 24. Repository Links

> **Circuit Diagram:** TODO
> **Summary Video:** TODO
> **Open Challenge Video:** TODO
> **Obstacle Challenge Video:** TODO
> **CAD Files:** [`/cad`](./cad)
> **Software:** [`/software`](./software)
> **Wiring:** [`/wiring`](./wiring)
> **Development Log:** [`/docs/dev_log.md`](./docs/dev_log.md)

---

# License

This project was developed for the **World Robot Olympiad Future Engineers 2026** competition.

Add the appropriate license here if the team decides to make the repository officially open source.
