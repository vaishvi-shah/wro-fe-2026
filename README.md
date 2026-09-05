# Hello! We are Team Electrocute

<table>
  <tr>
    <td align="center"><strong>Team Picture</strong><br><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Team%20Picture.jpg" width="500
                                                          00"></td>
  </tr>
</table>

We are a team of three high school students participating in the **World Robot Olympiad (WRO) Future Engineers 2026** category.

| Member           | Role                 |
| ---------------- | -------------------- |
| Vaishvi Shah     | Software & Logic     |
| Avani Devalankar | 3d Design & Hardware |
| Jaitra Bhatt     | Documentation        |

# Overview

The primary objective was to build an autonomous robot capable of navigating a closed course with obstacles and parallel parking itself at the end.

We chose this project because we wanted a challenging experience during high school where we could apply our Python skills while developing new skills in electronics, CAD, and 3D printing. We also drew inspiration from top teams that participated in WRO Future Engineers over the past two years by studying their [GitHub repositories](https://github.com/World-Robot-Olympiad-Association).

Our main objective was to create a dependable and effective system while showcasing our engineering, problem-solving, and teamwork skills. The project also gave us an opportunity to expand our knowledge and develop new skills throughout the process of building our robot.

We approached the development through several stages, beginning with idea generation and research, followed by designing, building, testing, and refining our system. We also kept thorough records of our work to make it easier to share information, track changes, and maintain a consistent workflow throughout the project.

### Summary Video
Add summary video here.

### Obstacle Video
Add obstacle video here.

### Open Challenge Videos
Add open challenge videos here.


## Table of Content
Add table of Content 

## Meet The Robot

- Insert robot images

Meet Batmobile, our autonomous car designed and 3D-printed for WRO Future Engineers 2026. The brain of the robot is a Raspberry Pi 5, which processes the camera feed using OpenCV and handles navigation, obstacle detection, and sensor logic. A Raspberry Pi Pico communicates with the Pi through a USB serial connection and controls the DC motor and steering servo.

Batmobile uses a camera, IMU, and encoder sensor to navigate the field. The camera provides visual information for navigation and obstacle detection, the IMU helps maintain a straight heading and execute accurate turns, and the encoder sensor assists with precise parking. Together, these systems allow Batmobile to independently perceive its surroundings, make decisions, and navigate the competition field.

### Preliminary Work

We found the public WRO repositories especially useful when getting started. Being able to look at documentation from previous teams gave us a reference for how to approach the competition without having to figure everything out from scratch. We’ve tried to do the same with our documentation by making it practical and detailed enough that a future team can use it as a starting point for their own robot.


# Introduction

## Meet The Robot

- Insert robot images

Meet Batmobile, our autonomous car designed and 3D-printed for WRO Future Engineers 2026. The brain of the robot is a Raspberry Pi 5, which processes the camera feed using OpenCV and handles navigation, obstacle detection, and sensor logic. A Raspberry Pi Pico communicates with the Pi through a USB serial port and controls the DC motor and the steering servo.

Batmobile uses a camera, IMU, and encoder sensor to navigate the field. The camera provides visual information for navigation and obstacle detection, while the IMU helps maintain a straight heading. Finally, the encoder assists with precise parking. Together, these systems allow Batmobile to independently perceive its surroundings, make decisions, and navigate the competition field.

## Preliminary Work

We found publicly available WRO repositories from previous teams especially useful during the early stages of our project. Their documentation gave us insight into how other teams approached robot design, programming, electronics, and problem solving. This allowed us to learn from their experiences and use their work as a starting point rather than having to develop every idea from scratch.

As our project progressed, we wanted to contribute to the same community that helped us. We therefore focused on creating documentation that is practical, detailed, and easy to follow. By documenting both our successes and failures, we hope our work can provide a useful starting point for teams developing their own robots.

# Mobility & Mechanical Design

The chassis is built in a layered structure so that mounting space could be divided deliberately, with the first layer holding the heavier components and a second, elevated layer used to mount the electronics. This final version did not happen immediately; we went through many iterations to make it work.

## First Iteration  LEGO Chassis

**Starting Design**

Our first chassis was constructed using LEGO Technic pieces to create a basic structure for initial testing. A LEGO differential was incorporated into the drivetrain system to transfer power to the wheels while allowing them to rotate at different speeds during turns.

**Issues Identified**

During testing, we encountered several mechanical problems. The wheels frequently came off the chassis during movement and turning. The steering system also had a limited Ackermann steering angle, which made it difficult for the robot to perform tight turns. This was especially problematic during parking, where greater steering angles were needed to maneuver within the available space. Additionally, the differential would come off the axle during operation, affecting the reliability of the drivetrain.

<table>
  <tr>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%201/Top%20View.jpg"  width="200" height="400" /></td>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%201/Bottom%20View.jpg" width="200" height="400" /></td>
  </tr>
  <tr>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%201/Right%20View.jpg" width="200" height="400" /></td>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%201/Left%20View.jpg" " width="200" height="400" /></td>
  </tr>
    <tr>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%201/Front%20View.jpg" width="200" height="400" /></td>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%201/Back%20View.jpg" " width="200" height="400" /></td>
  </tr>
</table>

## Second Iteration  3D-Printed Chassis

### Chassis Design

We replaced the original LEGO chassis with a custom 3D-printed chassis designed specifically around our robot’s components. The chassis was divided into two separate layers to organize the mechanical and electronic components:

- Layer 1: Battery, motor driver, servo, and Ackermann steering system.
- Layer 2: Raspberry Pi 5, Raspberry Pi Pico 2, and IMU.

This two-layer structure provided dedicated mounting areas, improved component organization, and made the overall system easier to assemble and maintain.

### Issues Identified

During testing, we found that placing the battery toward the rear of the chassis caused an uneven weight distribution. This shifted the centre of mass toward the rear, reducing the normal force acting on the front wheels.

With less weight on the front wheels, the available front-wheel traction decreased, particularly during faster movement and sharp turns. At higher speeds, the front wheels would lose traction and lift slightly, which reduced steering effectiveness and made the robot less stable and predictable while turning.

<table>
  <tr>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%202/Top%20View.jpg"  width="200" height="400" /></td>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%202/Bottom%20View.jpg" width="200" height="400" /></td>
  </tr>
  <tr>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%202/Front%20View.jpg" width="200" height="400" /></td>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%202/Bottom%20View.jpg" " width="200" height="400" /></td>
  </tr>
    <tr>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%202/Right%20View.jpg" width="200" height="400" /></td>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%202/Left%20View.jpg" " width="200" height="400" /></td>
  </tr>
</table>

## Third Iteration  Physical Fit

### Chassis & Motor Fit

The chassis was extended after measuring the physical DC motor and discovering that it was longer than the original CAD model. An access hole was added to allow easier access to the motor mounting screws during assembly and maintenance. Finally, the motor clamp geometry was adjusted to improve the fit, including reducing the clamp diameter by 0.25 mm for a tighter connection.

### Chassis & Weight Distribution

The front of the chassis was extended to create more space for parking and make the robot easier to maneuver. We also moved the battery closer to the centre of the chassis to balance the weight more evenly and improve stability.

Additional space was made inside the chassis to fit the updated DC motor, which had different dimensions from the previous motor. We also replaced the previous motor driver with a new motor driver setup, so the layout of the internal components had to be adjusted to fit everything properly.

### Mounting & Fastening

The LEGO peg holes were increased by 0.1 mm after testing showed that the original holes were slightly too small. We also added dedicated mounting holes for the servo so it could be securely attached to the chassis.

Before finalizing the design, we test-fitted each clamp with its corresponding physical component. This helped us check that the dimensions were correct and catch any fitting issues before final assembly.

<table>
  <tr>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%203/Top%20View.jpg"  width="200" height="400" /></td>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%203/Bottom%20View.jpg" width="200" height="400" /></td>
  </tr>
  <tr>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%203/Right%20View.jpg" width="200" height="400" /></td>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%203/Left%20View.jpg" " width="200" height="400" /></td>
  </tr>
    <tr>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%203/Front%20View.jpg" width="200" height="400" /></td>
    <td><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Batmobile%20Iteration%203/Back%20View.jpg" " width="200" height="400" /></td>
  </tr>
</table>

### Accessibility & Wiring

The chassis was designed to keep the wiring and electrical components accessible. This allowed us to easily reach the connections during testing, troubleshooting, and repairs without having to completely disassemble the robot.

### Battery Placement

Initially, the battery was positioned toward the rear of the robot, but this caused uneven weight distribution and shifted the centre of mass backward.

To solve this, we moved the battery toward the centre of the chassis, which distributed the weight more evenly and improved stability and traction during turns.

### Rear Wheel Bearings

We added bearings to the rear wheel axles to reduce friction between the axles and the chassis. This allowed the rear wheels to rotate more smoothly and freely, reducing mechanical resistance and improving the robot's overall movement and consistency.

### Screw Usage

We switched to M3 screws throughout the assembly, replacing the larger fasteners used in earlier versions. This helped reduce the overall weight of the robot while still providing enough strength to securely hold the chassis.

Using one standard screw size also made assembly and part sourcing simpler by reducing the number of different fasteners required.## Drive System

### Motor Selection

### Motor 1: JGA25-371 620 RPM DC Motor

<table>
  <tr>
    <td align="center"><strong></strong><br><img src="https://github.com/vaishvi-shah/wro-fe-2026/blob/main/photos/Robot%20Electronic%20Parts/JGA25-371%20620%20RPM%20DC%20Motor.png" width="300"></td>
  </tr>
</table> 

### Specifications

| Specification | Value |
|---------------|-------|
| Reduction Ratio | 9.6:1 |
| Rated Voltage | 12V |
| Speed | 620 RPM |
| Current | 60 mA |
| Torque | 0.1 kg·cm |
| Speed | 450 RPM |
| Current | 0.45 A |
| Torque | 0.35 kg·cm |
| Current | 1.3 A |

The WRO track is flat, so we did not need the extra stall torque provided by the lower-RPM motors to handle slopes. Instead, we chose the 620 RPM motor because its higher top speed allows the robot to move faster during the open sections of the track, helping us achieve faster lap times.

## Drive System

### Motor 2: Lower-RPM JGA25-371 Variant

Add pictures

| Specification | Lower-RPM JGA25-371 (~126–280 RPM) |
|---|---|
| Operating voltage | 6–24V (12V nominal) |
| Free-run speed at 12V | ~126–280 RPM depending on ratio |
| Stall torque at 12V | ~2.65–4.2 kg·cm depending on ratio |
| Free-run current at 12V | ~46 mA |
| Encoder | Integrated, ~12 counts/rev |
| Main advantage | Higher torque margin for acceleration and obstacle-course maneuvering |
| Main disadvantage | Lower top speed, resulting in slower lap times on the open/obstacle track |

At a 1:34 ratio, the motor runs at approximately 126 RPM at 12V with roughly 4.2 kg·cm of stall torque, while a 201 RPM version provides approximately 2.65 kg·cm of stall torque.

The higher gear reduction gives the robot more torque at the wheels, which improves acceleration and helps it overcome resistance. However, this also reduces the robot’s maximum speed. Since the WRO track is flat and has open sections where speed is important, we decided that the additional torque was not as useful as having a higher top speed.

### Final Decision

We chose the JGA25-371 620 RPM DC Motor because it provides a good balance of speed, torque, and size for our robot. The motor also has an integrated encoder, which allows us to measure its rotation and use that feedback for more accurate speed control.

We paired it with the WLToys 144001 differential, which transfers power to the rear wheels while allowing the wheels to rotate at different speeds when turning. Overall, this setup gave us the speed and control we needed for the WRO track.

| Risk | Mitigation |
|---|---|
| The stall torque for the specific 620 RPM gear ratio is not published by the vendor, so the value is estimated by scaling down from documented lower-RPM variants in the same JGA25-371 family. | Encoder feedback allows the motor's actual RPM to be monitored under load. A significant RPM drop without a corresponding increase in the drive command can indicate that the motor is approaching its available torque limit. This allows a potential torque shortfall to be identified during testing before competition. |

### Steering, Servo & Differential Design

### WLToys 144001

Our first drivetrain used a LEGO differential to drive the rear wheels, allowing the wheels to rotate at different speeds during turns and reducing wheel scrub. However, the differential axle repeatedly came loose during sharper turns.

To improve reliability and performance, we replaced the LEGO differential with a WLToys 144001 differential, which has a 2.5:1 reduction ratio. This provided a stronger connection, smoother movement, and more consistent turning while eliminating the axle-loosening issue.

Add pictures of differential

### Ackermann Steering

Our initial LEGO robot used Ackermann steering, but when we switched to the custom chassis, the original steering geometry caused the robot to slip whenever it moved. We adjusted the Ackermann steering angle to better suit the new chassis geometry, reducing wheel slip and improving steering stability.

The new linkage also made better use of the servo’s approximately 50°–130° operating range, rather than having its movement restricted by the previous LEGO chassis geometry.

| Position | Angle |
|---|---|
| Center | 82° |
| Full Left | Center - 45° |
| Full Right | Center + 45° |

This was necessary because the inside wheel travels along a smaller circle, while the outside wheel travels along a larger circle. By turning the wheels at different angles, the robot can follow the correct path through a corner and turn more smoothly without the tires dragging across the ground.

| Risk | Mitigation |
|---|---|
| The steering range was originally limited to 0°–90° because of a servo bug discovered during Iteration 1. | We traced and corrected the bug, then tested the servo against the physical Ackermann linkage to determine its actual mechanical limits. |
| Using a steering range beyond the mechanical limits could cause the linkage to bind or damage the components. | We verified a safe 50°–130° range through physical testing and permanently adopted it as the steering limit. Every steering formula in our software clamps its output to this range. |

### Servo: MG90S Servo

Add picture of servo

### Specifications

| Specification | Value |
|---|---|
| Rated Torque | 1.1 kgf/cm |
| Speed | 0.15 sec/60° |
| Voltage | 5V |
| Gearing | Metal |
| Type | Digital |

We chose the servo because its compact size and PWM control made it well suited for steering. Its torque was enough to move the front wheels reliably and respond quickly to steering commands. It is also commonly used in robotics, so finding documentation and compatible mounting hardware was straightforward.

After considering several steering options, we selected Ackermann steering geometry because it reduces tire slip and improves turning accuracy. The inside and outside front wheels turn at different angles, allowing each wheel to follow its own turning path. Our steering linkage is arranged so that the projected lines of the front wheels meet near the rear axle, creating the desired Ackermann geometry.

# Power & Sensor Architecture

## Component Placement

Show 2 layers - Add pictures from Canva

## Microcontroller Selection & Development

The microcontroller handles the robot's low-level control, including the motor, steering servo, and IMU, while communicating with the Raspberry Pi 5 for higher-level processing and decision-making.

### Iteration 1: micro:bit

Add pics

The micro:bit is a compact board based on the Nordic nRF52833. It includes an accelerometer, magnetometer, Bluetooth, LED matrix, and two buttons.

- Advantage: Easy to program and quick to test.
- Disadvantage: Limited motor and servo control.
- Testing: The Raspberry Pi connection was unreliable.

### Raspberry Pi Pico 2 W

Add pics

The Raspberry Pi Pico 2 W uses the RP2350 and has Wi-Fi, Bluetooth, and 26 GPIO pins. It does not have built-in sensors, but it can connect to many sensors and devices.

- Main advantage: It has many GPIO pins and supports PWM, I²C, SPI, and UART.
- Main disadvantage: It required more setup and testing to get all of our hardware communicating correctly.
- Final solution: After testing and configuring the Pico 2 W, we were able to reliably control the motor and servo while communicating with the Raspberry Pi 5.

## Raspberry Pi 5

No second board was tested against the Raspberry Pi 5. It was selected at the beginning of the design because the robot's navigation system requires real-time processing of camera data for lane, wall, and pillar detection.

| Specification | Raspberry Pi 5 (4GB) |
|---|---|
| CPU | Broadcom BCM2712, quad-core Cortex-A76 @ 2.4 GHz |
| RAM | 4GB LPDDR4X |
| Camera interface | MIPI CSI-2 |
| Connectivity | Gigabit Ethernet, Wi-Fi, Bluetooth, USB 3.0 |
| Cooling | Active cooling required under sustained load; heatsink and fan used |
| BOM reference cost | $153.95 CAD |
| Main advantage | Sufficient CPU headroom for real-time computer vision |
| Main disadvantage | Highest power draw and cooling requirement of the computing boards |

The Raspberry Pi 5 was chosen because the robot needs to process live camera video to detect lanes, walls, and pillars.

- Main advantage: It has enough processing power to handle real-time camera and computer vision tasks.
- Main disadvantage: The Raspberry Pi 5 has higher power consumption, which puts more demand on the robot's battery.
- Cooling: Active cooling was added to prevent overheating during long periods of use.













<!--
HOW IMAGES WORK IN THIS FILE:
Every place a picture belongs, you'll see a comment like this one (invisible on GitHub, only visible when editing the raw file) telling you exactly what filename/path to save the image under. Once you commit that image file to that exact path in your repo, the ![...](...) line right below the comment will automatically display it — no other changes needed. Comments like this one never render on GitHub.
-->

# Introduction

Hello! We are a team of three high school students participating in the World Robot Olympiad
(WRO) Future Engineers 2026 category. Our team combines software, hardware, design, and
documentation to develop an autonomous robot capable of navigating the competition field and
completing its challenges.

Our team members each have a specific area of responsibility:

| Member | Role |
|---|---|
| Vaishvi Shah | Software & Logic |
| Avani Devalankar | 3d Design & Hardware |
| Jaitra Bhatt | Documentation |

Although we have individual roles, our development is a collaborative process. We work
together to test ideas, identify problems, refine our design, and make decisions about the robot's
performance.

Through WRO Future Engineers, our goal is not only to compete, but also to strengthen our
skills in robotics, programming, engineering, problem-solving, and teamwork. This
documentation presents our journey from our initial concepts and prototypes to the final robot,
including the challenges we encountered, the solutions we developed, and the improvements
we made along the way.

## Summary Video

+ obstacle/open videos

<!--
IMAGE: Summary video — this isn't a static picture, it's a video link. On GitHub you can't embed a playable video directly in a README, so the standard approach is either:
1. A clickable thumbnail: ![Watch the video](video/img/summary_thumbnail.png) wrapped in a link to the YouTube URL, e.g.
   [![Watch the video](video/img/summary_thumbnail.png)](https://youtube.com/your-video-link)
2. Or just a plain link: [Watch our summary video](https://youtube.com/your-video-link)
Save a thumbnail image (if using option 1) to: video/img/summary_thumbnail.png
-->

## Meet The Robot

<!--
IMAGE: Robot glamour shots ("Insert robot images"). Save these to your v-photos folder, e.g.:
v-photos/img/front.jpg, v-photos/img/back.jpg, v-photos/img/left.jpg, v-photos/img/right.jpg, v-photos/img/top.jpg, v-photos/img/under.jpg
Then embed each with:
-->
![Robot front view](v-photos/img/front.jpg)
![Robot back view](v-photos/img/back.jpg)

Meet Batmobile, our autonomous car designed and 3D-printed for WRO Future Engineers 2026.
At the heart of the robot is a Raspberry Pi 5, which processes the camera feed using OpenCV
and handles navigation, obstacle detection, and sensor logic. A Raspberry Pi Pico
communicates with the Pi through USB-C and controls the motors and steering servo.

Batmobile uses a camera, IMU, and TOF sensor to navigate the field. The camera provides
visual information for navigation and obstacle detection, the IMU helps maintain a straight
heading and execute accurate turns, and the TOF sensor assists with precise parking. Together,
these systems allow Batmobile to independently perceive its surroundings, make decisions, and
navigate the competition field.

## Preliminary Work

We found the public WRO repositories especially useful when getting started. Being able to look
at documentation from previous teams gave us a reference for how to approach the competition
without having to figure everything out from scratch. We've tried to do the same with our
documentation by making it practical and detailed enough that a future team can use it as a
starting point for their own robot.

---

# Mobility & Mechanical Design

## Chassis Overview

The chassis is built in a layered structure so that mounting space could be divided deliberately
between the first layer (heavier, noise-sensitive components) and a second, elevated layer
(sensitive sensing components). This split wasn't part of the original plan, it came out of a real
space conflict during assembly, described below.

## Chassis

### First Iteration — LEGO Chassis

**Starting Design**
- Built our first chassis using LEGO components.
- Used a LEGO-based drivetrain and differential to test the basic movement and steering system.

**Issues Identified**
- Wheels would frequently come off the chassis during testing.
- Steering was difficult because the LEGO design limited the available Ackermann steering angle.
- The limited Ackermann geometry made parking difficult, as the robot could not make sufficiently tight turns.
- The differential would come off the axle, especially during turning and movement.

**Conclusion**
- The LEGO chassis was useful for our initial testing, but its mechanical limitations made it unsuitable for reliable steering, parking, and drivetrain operation.

### Second Iteration — 3D-Printed Chassis

**Chassis Design**
- Replaced the LEGO chassis with a custom 3D-printed chassis.
- Designed the chassis with two separate layers:
  - Layer 1: Battery, motor driver, servo, and Ackermann steering system
  - Layer 2: Raspberry Pi Pico and Raspberry Pi 5
- The two-layer design provided dedicated mounting locations for the electronics and drivetrain components.

**Issues Identified**
- The battery was positioned toward the rear of the chassis, creating an uneven weight distribution.
- This shifted the centre of mass toward the rear and reduced the normal force on the front wheels.
- The reduced front-wheel normal force decreased the available traction:

  **F_f = μN_f**, where *F_f* is the maximum frictional force at the front wheels, *μ* is the coefficient of friction, and *N_f* is the normal force on the front wheels.

- At higher speeds and during turns, the front wheels would lose contact/traction and lift slightly, reducing steering control.
- This made the robot less stable and predictable during high-speed driving and turning.

<!--
IMAGE: "Hardware Architecture" block diagram (Pi Camera 3 Wide / MG95S Micro Servo / DC Motor / Raspberry Pi 5 / Raspberry Pi Pico / L298N Motor Driver, with arrows showing signal flow).
Best-match existing filename in your repo tree: elec/Assembly_Instructions/img/System_operation_process.png
Save the diagram there (or rename to match if you already have a different file), then embed with:
-->
![Hardware architecture diagram](elec/Assembly_Instructions/img/System_operation_process.png)

### Iteration 3 (Final) — Physical Fit

After 3D printing the parts, we identified several physical mismatches that were not apparent in the
previous iteration. Each issue was addressed and validated against the real hardware:

**Chassis & Motor Fit**
- Motor wire opening: Reoriented the opening after it was found to bind the wiring against the chassis.
- Motor length: Extended the chassis after confirming the physical DC motor was longer than the CAD model.
- Motor mounting access: Added an access hole for the DC motor mounting screws.
- Motor clamp: Tightened the fit by adjusting the clamp geometry and reducing its diameter by 0.25 mm.

**1. Chassis & Weight Distribution**
- Extended the front of the chassis to create additional space for parking and maneuvering.
  - Constraints & Trade-Offs: We chose to extend the chassis to match our real, physically-longer DC motor rather than resize our CAD assumption to fit the motor, since the actual part in hand is the more reliable design reference than the original CAD estimate.
- Relocated the battery to the center of the chassis to improve overall weight distribution and balance.
- Added dedicated space for the new DC motor to accommodate its updated dimensions.
- Replaced the previous motor driver with the new driver configuration and adjusted the chassis layout accordingly.

**Mounting & Fastening**
- LEGO peg holes: Increased hole diameter by 0.1 mm after finding the original holes slightly undersized.
- Servo mounting: Added dedicated holes to secure the servo to the base.
- Clamp validation: Test-fitted every clamp against the physical components before finalizing the design, addressing the fit verification gap from the previous iteration.

**Mechanical Connections**
- Motor → drive gear: Designed and finalized a custom connector to mechanically couple the DC motor to the drive gear.
- Differential → wheel axle: Designed and finalized a custom connector to couple the differential to the wheel axle.

**Result:** With the motor-to-gear and differential-to-axle connections mechanically finalized, the
wiring harness could follow a fixed, repeatable routing rather than being improvised for each
build.

## Accessibility & Wiring

The chassis was designed to keep the wiring and electrical components accessible. This allowed us to
easily reach the connections during testing, troubleshooting, and repairs without having to
completely disassemble the robot.

### Battery Placement

Initially, the battery was positioned toward the rear of the robot, but this caused uneven weight
distribution and shifted the centre of mass backward. This reduced the normal force on the front
wheels, causing the robot to lose traction and slip or skid whenever it turned. To solve this, we
moved the battery toward the centre of the chassis, which distributed the weight more evenly and
improved stability and traction during turns.

### Rear Wheel Bearings

We added bearings to the rear wheel axles to reduce friction between the axles and the chassis. This
allowed the rear wheels to rotate more smoothly and freely, reducing mechanical resistance and
improving the robot's overall movement and consistency.

### Screw Usage

- Switched to M3 (metric, 3 mm) screws throughout the assembly, replacing the larger fasteners used in earlier iterations.
- This reduced the overall weight of the robot, since M3 screws are smaller and lighter than the previous screw size while still providing sufficient holding strength for the chassis, motor mounts, and servo brackets.
- Standardizing on a single screw size also simplified assembly and part sourcing, reducing the number of different fasteners needed to build the robot.

## Motor Iterations

### Motor Iteration 1 — LEGO Differential

**Drivetrain Configuration**
- Our first drivetrain used a LEGO-based differential to drive the rear wheels rather than a fixed axle. This allowed the inside wheel to rotate at a different speed during turns, reducing wheel scrub and drag.

**Issues Identified**
- During sharper turns, the differential axle repeatedly came loose under turning stress.
- The LEGO gear train also had noticeable mechanical slack, which absorbed part of the servo's input before it reached the wheels. This made the robot's turns less sharp and less precise than commanded.
- The LEGO chassis geometry also limited how much of the MG95S servo's movement could be converted into steering angle, limiting the robot's minimum turning radius.

**Engineering Decision**
- Rather than continuing to adjust the LEGO system around these limitations, we moved to a more robust differential that could handle turning loads while providing a more consistent connection between the motor and wheels.

### Motor Iteration 2 — WLToys 144001 Differential

**Differential Upgrade**
- We replaced the LEGO differential with a WLToys 144001 differential, which uses a 2:1 reduction: two rotations at the differential input produce one rotation at the output.
- The new differential provided smoother movement and more consistent turning, while also eliminating the axle-loosening issue seen in Iteration 1.

**Gearbox Ratio Selection**

We tested four gearbox reduction ratios to find a balance between torque and speed:

| Gear Ratio | Testing Result |
|---|---|
| 4.4:1 | Higher speed, but insufficient torque for reliable acceleration and maintaining speed through loaded turns. |
| 9.6:1 | Best balance of torque and speed for the vehicle |
| 46.8:1 | Large torque margin, but vehicle speed became too low |
| 100:1 | Very high torque, but significantly too slow for the course |

**Final Selection: 9.6:1**

Constraint & Trade-Offs: This was a deliberate trade-off: the 46.8:1 and 100:1 options provided
more torque, but they were simply too slow for the WRO course.

- The 9.6:1 gearbox was selected because it provided enough torque for acceleration and loaded turns while keeping the robot fast enough to stay competitive within the fixed 3-minute round.
- The motor provides approximately 0.9 kg·cm of stall torque at 1.8 A at this configuration.
- The 2:1 differential reduction then adds another stage of reduction after the motor gearbox, increasing wheel torque while reducing output speed.
- Therefore, the 9.6:1 gearbox and 2:1 differential were selected as one drivetrain system, rather than treating the two ratios independently.

**Measured Motor Speed**
- We used the motor's tachometer-measured speed under load rather than relying only on the datasheet value.
- The motor is rated at approximately 620 RPM at no load, but testing on the actual drivetrain gave approximately 400 RPM under load.
- This difference of roughly 35–40% comes from real-world losses including:
  - Gearbox friction
  - Differential losses
  - Tire friction
  - Chassis and vehicle mass
- Using the loaded measurement gave us a more realistic value for estimating the robot's actual driving speed.

**Risk and Mitigation:** This ~35–40% gap between no-load and loaded specs is a known risk we
address directly by using the measured loaded-RPM figure for all speed and runtime planning,
rather than trusting the datasheet's no-load number.

<!--
IMAGE: DC gear motor dimension diagram (31mm length, 24.4mm width, 17.5-27mm shaft, 4mm shaft diameter, mounting plate with 2-M3 holes).
Exact match in your repo tree: elec/Motor/img/DC_Gear_Motor.png
Save the image there, then embed with:
-->
![DC gear motor dimensions](elec/Motor/img/DC_Gear_Motor.png)

## Ackerman Steering Geometry

- We chose the differential and Ackermann steering to help maintain clean IMU heading data. A fixed axle would drag the inside wheel through turns, while non-Ackermann steering would cause tire scrub. Both effects can introduce mechanical disturbances that affect the heading data our software relies on for wall-following and parking. Therefore, the mechanical design was developed alongside the sensing system rather than independently of it.
- The MG95S servo was retained, but the steering system was changed from the LEGO geometry to an Ackermann steering linkage.
- The new linkage allowed us to make much better use of the servo's approximately 50°–130° operating range, rather than having its movement limited by the previous LEGO chassis geometry.

**Servo Range**

| Position | Angle |
|---|---|
| Full Left | 50° |
| Full Straight | 90° |
| Full Right | 130° |

- This produced a larger usable steering angle, allowing the robot to make tighter turns while maintaining more predictable steering response.

This is needed because:
- The inside wheel travels a smaller circle.
- The outside wheel travels a larger circle.

By turning the wheels at different angles, the vehicle can corner smoothly without the tires
dragging across the ground.

**Risks and Mitigations:** This bounded range was originally discovered through the
Iteration 1 servo bug, not by original design — the bug limited the servo to 0°–90° before
we traced and corrected it. We verified the corrected 50°–130° range against the physical
Ackermann linkage's real mechanical limits before adopting it permanently, and this
became the exact range every steering formula in our software clamps against.

## Logic

- The Raspberry Pi 5 processes the camera input and driving logic to determine when and how much the robot should turn.
- The Pi sends the steering command to the Raspberry Pi Pico, which converts the command into a PWM signal for the MG95S servo.
- The servo then physically drives the Ackermann steering linkage.

**Signal Flow:**

`Camera → Raspberry Pi 5 → Raspberry Pi Pico → PWM → MG95S Servo → Ackermann Linkage → Front Wheels`

**Iteration 2 Result**
- The combination of the WLToys differential, 9.6:1 motor gearbox, 2:1 differential reduction, and Ackermann steering gave us a drivetrain with a more controlled balance of:
  - Torque for acceleration and loaded turns
  - Speed for completing the course within the 3-minute limit
  - Steering range for tighter turns
  - Mechanical consistency for repeatable driving

## Placement of Components

| Component | Chassis Layer | Why Placed There |
|---|---|---|
| Battery | 1st: Center, Flat | Mounted low and centrally for stability and even weight distribution. Since friction follows F = μN, the battery's position affects how the normal force N, and therefore friction, is distributed across the wheels. Mounting the battery near the back caused the front wheels to slip when turning. |
| DC Motor | 1st | Drives the rear axle through a differential. |
| Camera Mount | 1st: Front, high | Tilted down to capture both side ROIs and the track ahead. The higher position provides a longer sight line, allowing the robot to detect pillar colours earlier and make smoother steering decisions before getting too close. |
| IMU | 2nd: Away from battery | Contains a magnetometer, accelerometer, and gyroscope. Since magnetic field strength decreases with distance according to the Biot–Savart law, placing the IMU on the second layer keeps it farther from the battery and motor, reducing magnetic interference and vibration for cleaner heading data. |
| Pico | 2nd | Grouped with the IMU and kept away from high-current wiring to reduce electrical noise and interference. |
| TOF Sensor | 1st; chassis rear | Mounted at the back of the chassis, on the first layer with the other high-current, physically bulky components. Since the TOF sensor is used for close-range detection during parking and reversing, it needs a direct, unobstructed line of sight to the rear of the robot — placing it on the elevated second layer would offset its sensing axis from the actual rear edge of the chassis, introducing measurement error exactly when precision matters most. |

---

# Power & Sensor Architecture

## Power System Architecture

### Two-Board Architecture
- The Raspberry Pi 5 acts as the robot's main decision-making computer. It processes camera input and runs the higher-level logic, including lane following, obstacle avoidance, and the state machine, in Python.
- The Pi 5 sends movement commands to the Raspberry Pi Pico 2W, which acts as the real-time controller.
- The Pico runs a single program directly from flash rather than a full operating system, allowing it to handle low-latency control tasks reliably.
- The Pico reads the BNO055 IMU directly and converts commands from the Pi 5 into low-level signals for the MG95S servo and L298N motor driver.
- The Pico can also send status information back to the Pi 5.

### Why Two Boards?
- The split is based on the strengths of each board:
  - Pi 5 → High-level processing: camera processing, decision logic, and state machine.
  - Pico 2W → Real-time control: IMU reading, PWM generation, servo control, and motor commands.
- This prevents heavy camera processing from interfering with the timing-sensitive steering and motor control.

### Power Distribution
- An external power source feeds the L298N and drives the motor directly because the Pico cannot safely provide the current required by the motor.
- The electronics use the appropriate regulated supply, while the motor receives power directly through the motor driver.
- All components share a common ground, giving every control signal the same electrical reference.

### Current Draw & Battery Sizing

Rather than estimating battery requirements, we calculated the expected current draw of each
component and used the total to size the power system.

| Component | Approx. Current | Supply |
|---|---|---|
| Pico 2W | 0.150 A | 5 V |
| BNO055 IMU | 0.010 A | 5 V |
| L298N logic | 0.036 A | 5 V |
| MG95S servo | 0.700 A | 5 V |
| Camera | 0.250 A | 5 V |
| **5 V rail total** | **1.146 A** | **5 V** |

**5 V Power Requirement**
- The 5 V rail requires approximately 5.73 W.
- Accounting for approximately 85% buck-converter efficiency, the battery must supply approximately 6.74 W for the 5 V electronics.

**Motor Power**
- The drive motor draws approximately 0.600 A at 12 V through the L298N.
- This adds approximately 7.2 W to the battery load.

**Total System Requirement**
- Combined estimated battery draw: approximately 13.94 W.
- At a nominal 12 V, this corresponds to approximately 1.16 A.
- With an 8.8 Ah battery pack, the theoretical continuous runtime is approximately 7.6 hours.

**Why This Matters**
- A competition run is much shorter than the calculated runtime, leaving a large margin for voltage sag, motor load, stall current, and steering current spikes.
- The battery was therefore selected from a bottom-up power calculation rather than an estimate.

**Risk:** Battery voltage sag near full discharge also reduces motor torque available through the
L298N.

**Mitigation:** Our conservative runtime margin (~7.6 hrs theoretical vs. a run that takes minutes)
helps offset this, and voltage-compensated PWM scaling is documented as a planned future
improvement.

## Microcontroller Selection & Development

We chose the Raspberry Pi Pico 2W as the real-time controller alongside the Raspberry Pi 5
because heavy vision processing and fast, predictable control are better handled as separate
tasks.

### Electronics Bring-Up
- Before building the chassis, we first tested whether the computer, controller, motor, and servo could communicate reliably.
- Our first setup used a micro:bit connected to a Raspberry Pi, but the connection remained unreliable after multiple attempts, so we moved to a Raspberry Pi Pico.
- Early Pico tests produced communication errors. We corrected several software issues, but the connection still failed intermittently.
- After testing different tools, including Thonny, we identified the main issue: the Pico was sharing power with other components and was losing power when current demand increased.
- We solved this by giving the Pico a dedicated power source, creating the first version of the two-way Pi-to-Pico communication system that remains in the final architecture.
- We also tested the MG95S servo and found that its expected 0°–90° movement did not match the actual motion. We traced this to an incorrect PWM pulse-width conversion and corrected the signal range.
- At this stage, the system used breadboard and jumper-wire connections. The shared power rail and loose wiring contributed to the brownouts that we later addressed in the final design.

### Iteration 2 — Layered Electronics
- We divided the electronics into two physical layers:
  - Layer 1: battery, motor, and camera mount
  - Layer 2: IMU and Pico
- Moving the IMU and Pico away from the motor and battery wiring reduced exposure to electrical noise and vibration, giving the IMU a cleaner environment for heading measurements.

### Iteration 3 — Physical Fit & Wiring
- After 3D printing, we identified that the electronics did not yet have defined mounting positions.
- We therefore finalized the first-layer component layout with dedicated positions for the electrical components rather than placing them wherever space was available.
- We also replaced the earlier twisted-wire connections with terminal blocks, creating more secure connections that are less likely to short or disconnect during a collision.

### Development Result

The electronics evolved from a basic communication test into a defined architecture where the
Pi 5 handles decisions, the Pico handles real-time control, and the physical layout
protects the reliability of both systems.

## Sensor Selection & Justification

| Sensor | Interface | Primary Purpose |
|---|---|---|
| BNO055 IMU | I²C | Heading hold, drift correction, parking turns |
| TOF Sensor | I²C, address 29 | Close-range wall and obstacle detection |
| Pi Camera 3 Wide | CSI | Lane, wall, and pillar detection |

### Why We Moved to Camera + IMU Fusion
- Our initial lane-following system relied only on the camera, comparing black-pixel counts between left and right regions of interest (ROIs).
- This worked well when the track was clearly visible, but short camera blind spots could allow the robot's heading to drift before the camera recovered.
- We therefore added the BNO055 IMU specifically to handle this gap.
- The camera remains the primary reference because it directly observes the track, while the IMU helps maintain the robot's heading between camera updates.
- The two measurements are blended using a 70/30 weighting, rather than treating them as two independent steering systems that could give conflicting commands.

### TOF Sensor
- The TOF sensor provides close-range distance information for situations where visual information alone is less reliable, particularly around walls and obstacles.

### Motor Driver
- We selected the L298N for its simplicity and reliable operation.
- Its approximately 1.5–2 V voltage drop is a known efficiency trade-off that we accepted in exchange for keeping the motor-control system straightforward.
- The voltage drop means the motor only sees roughly 10–10.5V from a 12V-nominal battery. This matters directly for our torque calculations and is documented as a known risk (see Safety & Reliability Measures / risk section) rather than something we discovered and ignored.

## Sensor Placement & Calibration

### Physical Placement
- Sensor placement follows the two-layer architecture:
  - Layer 1: battery, motor, and camera
  - Layer 2: IMU and Pico
- The IMU is positioned away from the motor and high-current wiring to reduce electrical interference and vibration affecting its measurements.
- This separation helps produce more stable heading data during operation.
- This placement is a direct power-to-sensor constraint:
  - We couldn't position the IMU purely for convenience, we had to place it specifically to protect signal quality from electromagnetic interference generated by the battery + motor wiring.

### IMU Calibration
- Before each run, the robot is held stationary at the starting line while the IMU is calibrated.
- This establishes a stable initial reference and reduces the effect of gyro drift before the state machine begins.

## Wiring Architecture

### Control Path

The Pico acts as the central low-level controller:

`Raspberry Pi 5 → Raspberry Pi Pico 2W → Actuators`

- The Pi 5 sends movement and steering commands to the Pico.
- The Pico generates the PWM signal for the MG95S servo and control signals for the L298N.
- The servo directly drives the steering linkage.
- The L298N sits between the Pico and DC motor, allowing the Pico to send low-current control signals while the driver supplies the higher current required by the motor.
- The motor receives power from the external battery supply, rather than directly from the Pico.
- A small onboard status screen was also used during bench testing for live debugging.

### Common Ground
- All components share a common ground, ensuring that control signals have the same reference voltage throughout the system.

<!--
IMAGE: Pin-level circuit diagram (Raspberry Pi Pico pinout with Servo, Driver L298N, and Pico-LCD-1.44 screen wiring shown).
Exact match in your repo tree: elec/Wiring_Diagram/circuit_diagram.png
Save the image there, then embed with:
-->
![Pico wiring circuit diagram](elec/Wiring_Diagram/circuit_diagram.png)

The complete pin-level circuit diagram is maintained in our circuit-diagram.org project.

[https://www.circuit-diagram.org/editor/c/d64ecc1be7fa4d368e2e6eff0a0027d1](https://www.circuit-diagram.org/editor/c/d64ecc1be7fa4d368e2e6eff0a0027d1)

## Safety & Reliability Measures

**Main Power Switch**
- A single main switch powers the robot on and off between runs, following WRO Rule 9.10.

<!--
IMAGE: Main power switch photo.
Exact match in your repo tree: elec/Fool-Proof-Design/img/Main_Power_Switch.png
Save the image there, then embed with:
-->
![Main power switch](elec/Fool-Proof-Design/img/Main_Power_Switch.png)

**Battery Voltage Floor**
- The 3S Li-ion battery pack has a nominal voltage of approximately 12 V.
- The system does not allow the pack to fall below approximately 9 V, helping protect the battery and maintain stable electronics operation.

**Secure Wiring**
- Terminal blocks replaced loose twisted-wire joints, reducing the chance of a short or disconnected wire if the robot experiences a bump or collision.

**Front Bumper**
- A front bumper protects the camera and steering linkage if the robot contacts a pillar or wall.

<!--
IMAGE: Front bumper photo.
Exact match in your repo tree: elec/Fool-Proof-Design/img/Front_Bumper.png
Save the image there, then embed with:
-->
![Front bumper](elec/Fool-Proof-Design/img/Front_Bumper.png)

**Circuit Diagram**
- The complete pin-level circuit diagram is maintained in our circuit-diagram.org project.

## 2.7 Total Cost of Materials

| Component | Est. Unit Cost (CAD) | Qty | Est. Subtotal | Source / Link |
|---|---|---|---|---|
| Raspberry Pi 5 | [Placeholder] | 1 | [Placeholder] | [Insert link] |
| Raspberry Pi Pico 2 W | [Placeholder] | 1 | [Placeholder] | [Insert link] |
| Pi Camera 3 Wide | [Placeholder] | 1 | [Placeholder] | [Insert link] |
| JGA25-371 620 RPM 12V DC Gear Motor w/ Encoder | $12.64 | 1 | $12.64 | AliExpress |
| MG95S Micro Servo | [Placeholder] | 1 | [Placeholder] | [Insert link] |
| L298N Motor Driver | [Placeholder] | 1 | [Placeholder] | [Insert link] |
| BNO055 IMU | [Placeholder] | 1 | [Placeholder] | [Insert link] |
| Rear ToF Sensor | [Placeholder] | 1 | [Placeholder] | [Insert link] |
| WLToys 144001 Differential | [Placeholder] | 1 | [Placeholder] | [Insert link] |
| 12V 3s1p Li-ion Battery Pack (8800 mAh) | [Placeholder] | 1 | [Placeholder] | [Insert link] |
| Buck Converter (12V→5V) | [Placeholder] | 1 | [Placeholder] | [Insert link] |
| Pico-LCD-1.44 Display | [Placeholder] | 1 | [Placeholder] | [Insert link] |
| Terminal Blocks (set) | [Placeholder] | 1 | [Placeholder] | [Insert link] |
| M3 Fasteners (assorted) | [Placeholder] | 1 | [Placeholder] | [Insert link] |
| 3D Printing Filament (all 4 chassis iterations) | [Placeholder] | 1 | [Placeholder] | — |
| LEGO Wheels / Peg Parts | [Placeholder] | 1 | [Placeholder] | [Insert link] |
| Misc. Wiring / Connectors | [Placeholder] | 1 | [Placeholder] | — |
| **Total Estimated Build Cost** | | | **[Placeholder]** | |

## 2.75 Battery Runtime Calculation

| Component | Voltage Requirement | Current Requirement |
|---|---|---|
| Raspberry Pi Pico 2W | 5V (via buck converter) | ~150 mA (WiFi active) |
| IMU | 3.3–5V | ~10 mA |
| L298N Motor Driver (logic) | 5V logic | ~36 mA |
| DC Gear Motor (drive) | 12V (via L298N) | ~600 mA (typical running load) |
| Servo Motor (steering) | 5V | ~700 mA (under load) |
| Camera Module | 5V (via buck converter) | ~250 mA |
| Buck Converter (12V→5V) | 12V in, 5V out | ~85% efficiency (typical) |
| 12V 3s1p Li-ion Pack | 12V nominal (11.1V) | 8800 mAh (8.8 Ah) capacity |

Marked component current values are typical estimates for these part classes — swap in your
own motor/camera datasheet numbers if you have measured them, and redo the math below.

**Step 1 — 5V side consumption**
- Pico 2W: 0.150 A
- IMU: 0.010 A
- L298N logic: 0.036 A
- Servo: 0.700 A
- Camera: 0.250 A
- Total 5V current: 1.146 A → Power at 5V = 1.146 A × 5V = 5.73 W

**Step 2 — Account for buck converter efficiency (85%)**
- Battery power needed for the 5V rail = 5.73 W / 0.85 = 6.74 W

**Step 3 — Add the drive motor (12V side, direct off battery through the L298N)**
- DC Gear Motor: 0.600 A × 12V = 7.2 W

**Step 4 — Total battery power & current**
- Total power from battery = 6.74 W (5V rail) + 7.2 W (motor) = 13.94 W
- Current from battery = 13.94 W / 12V = 1.16 A

**Step 5 — Runtime at full load**
- Battery capacity: 8.8 Ah
- Runtime = 8.8 Ah / 1.16 A = ≈7.6 hours at continuous max load

**Conclusion:** At full continuous load, our 12V 3S1P 8800 mAh battery provides approximately
7.6 hours of runtime, far beyond a WRO competition run. This leaves a strong safety margin for
voltage sag, motor stall spikes, and rapid servo corrections. Since the robot rarely operates
every component at maximum load simultaneously, actual runtime should be even higher.

---

# Software Architecture & Obstacle Strategy

Our software is organized under the src directory, with the main challenge programs separated
from the supporting hardware and computer-vision code.

```
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

- **openchallenge.py**
  - Main program for the Open Challenge.
  - Contains the navigation and challenge-specific logic.
- **obstaclechallenge.py**
  - Main program for the Obstacle Challenge.
  - Contains the obstacle detection, navigation, and challenge-specific logic.
- **pico/**
  - Contains the Raspberry Pi Pico code.
  - Used to test and develop communication between the Raspberry Pi 5 and Pico.
- **lib/**
  - Contains supporting and reusable code for the robot's sensors and computer vision.
  - frames.py contains the Frame class and camera-processing functions used for tasks such as color and contour detection.
  - BNO055 configuration contains the code required to configure and communicate with the IMU.
  - TOF tester is used to test the TOF sensor and its distance readings.

This organization keeps the challenge logic, microcontroller code, and
sensor/computer-vision code separated, making individual systems easier to test and modify.

## Open Challenge Logic

The Open Challenge uses a combination of camera-based wall following and IMU heading
control to navigate the field.

<!--
IMAGE: Open Challenge steering flowchart (Left ROI / Right ROI / IMU heading → cam_steer / gyro_steer → steering_value → clamp → final steering).
Exact match in your repo tree: code/State_Machine_Pseudo_Code/img/Open_Challenge_Steering_Flowchart.png
Save the image there, then embed with:
-->
![Open Challenge steering flowchart](code/State_Machine_Pseudo_Code/img/Open_Challenge_Steering_Flowchart.png)

1. **Sensor Input**
   - The Raspberry Pi captures a 320 × 240 camera feed and reads the robot's heading from the BNO055 IMU.
   - The camera uses separate ROIs to detect the left/right black walls and blue/orange turn markers.
2. **Steering**
   - The camera calculates a steering correction based on the difference in wall area between the left and right sides.
   - The IMU calculates a correction based on the difference between the current and desired heading.
   - These are combined using 70% IMU + 30% camera to produce the final steering angle.
     - Risk: Over a long run, IMU drift could slowly reduce the accuracy of wall-following and parking.
     - Mitigation: To reduce this, we plan to re-zero the IMU during the run when the robot reaches known straight sections of the track, instead of calibrating only at the start.
3. **Turn Detection**
   - Blue markers indicate a counter-clockwise turn and orange markers indicate a clockwise turn.
   - The first detected colour locks the direction for the run.
   - Before executing a turn, the robot checks that there is enough space beside the corresponding wall.
   - The desired heading is then changed by 90°.
4. **Communication & Stopping**
   - The Raspberry Pi sends the calculated steering and speed values to the Raspberry Pi Pico, which controls the motor and servo.
   - After 12 detected line crossings, the robot enters its stopping sequence.

## Software Iterations

- Initial approach: Used only the camera for wall following and steering.
- Issue: The robot was able to follow the walls, but had difficulty maintaining a straight heading, especially over longer distances.
- Change: Added the BNO055 IMU to provide heading feedback and correct the robot's orientation.
- Testing: Experimented with different weightings between the camera and gyro steering outputs.
- Final result: 70% gyro + 30% camera produced the most stable and consistent navigation.
- Constraints & Trade-Offs: We chose this instead of a 50/50 split because the camera's direct spatial reading of track edges was more trustworthy moment-to-moment than the gyro's drift-prone heading estimate, but the gyro still adds meaningful stability during blind spots.
  - Gyro: Maintains the robot's heading and keeps it straight.
  - Camera: Provides positional correction based on the walls.
- Final approach: Both sensors contribute to steering, with the IMU having greater influence while the camera provides continuous positional correction.

## Obstacle Management

## State machine & obstacle logic

Each frame, the main loop reads the camera and gyro, dispatches to one state's logic, and
force-switches to IN_PARKING once LINE_COUNT (12) line crossings are reached.

<!--
IMAGE: Full state machine diagram (OUT_PARKING → WALL_FOLLOW → AVOIDING_OBSTACLE → TURNING → REVERSING → IN_PARKING, with all transition labels).
Exact match in your repo tree: code/State_Machine_Pseudo_Code/img/Full_State_Machine_Diagram.png
Save the image there, then embed with:
-->
![Full state machine diagram](code/State_Machine_Pseudo_Code/img/Full_State_Machine_Diagram.png)

### State details

**OUT_PARKING** — compares left/right wall pixel area, steers max-away from whichever is closer
for ≥1s, holds if a matching-colour obstacle sits in the side just turned into, then →
`WALL_FOLLOW`.

**WALL_FOLLOW** — steering from `navigate_wall()`, speed 87. Watches for obstacles (→
`AVOIDING_OBSTACLE`) and line crossings. First colour seen (blue/orange) locks `direction`
(`CCL`/`CWR`) for the whole run; the other colour is ignored from then on.

**AVOIDING_OBSTACLE** — tracks the nearest red/green contour, slows as it gets closer (area
400→4000px² maps to speed 87→72), and steers toward the midpoint between the obstacle's
far corner and the nearest wall pixel (GREEN passes left, RED passes right). If the obstacle isn't
"wrong side", is past 75% down the frame, and area > 4000px², → `REVERSING`. No obstacle in
view → `WALL_FOLLOW`.

- Risk & Mitigation: HSV colour thresholds for pillar detection may not hold up under competition lighting different from our testing environment. This is documented explicitly, with a pre-run calibration step planned as a future improvement.

**TURNING** — runs whenever a turn is pending. A matching-colour obstacle (RED for CCL,
GREEN for CWR) vetoes the turn entirely (crawl straight until it clears); an opposite-colour
obstacle doesn't block it — the turn fires as soon as the wall-clearance gate
(`SAFE_TURN_AREA`) allows, and control hands to `AVOIDING_OBSTACLE` to keep dodging on
the new heading. Gives up waiting after 1.5s regardless.

**REVERSING** — backs straight for exactly 1s, then → `AVOIDING_OBSTACLE` to reassess (not
straight back to `WALL_FOLLOW`).

**IN_PARKING** — gated sequence held by `gyro_only_steer()` (heading-only, no camera):
reach back wall → initial turn → cleared check → (sequence currently stops here —
`STOP_AFTER_PARK_CLEARED` is `True`) → forced 90° turn → final reverse → final turn → hold.

**Software → Mechanical:** Our parking logic deliberately shifts primary control from
camera to IMU during the reverse turn, because the mechanical precision parking
requires exceeds what camera-based positioning reliably provides at close range. The
software strategy was built around a known mechanical/sensing limitation, not the other
way around. We chose this forced, pre-tuned turn over continuous live camera correction
— small steering hesitations in a tight parking maneuver caused more error than a
committed, pre-tested turn angle, trading some adaptability for repeatability.

### Steering methods

| Method | Used By | Basis |
|---|---|---|
| `navigate_wall()` | WALL_FOLLOW, TURNING (default), AVOIDING_OBSTACLE (base) | 0.7×gyro-P + 0.5×camera-P, clamped [50,130] |
| `gyro_only_steer()` | IN_PARKING only | Full gyro PD, no camera |
| Direct obstacle steering | AVOIDING_OBSTACLE, TURNING (opposite-colour case) | Proportional on pixel error to the pass point, overrides the above |

**Mechanical Iteration → Software Behavior:** The 50°–130° range our steering formula clamps
against wasn't an arbitrary software choice — it traces back to the Ackermann linkage's real
mechanical range, which itself was only discovered after an early servo PWM bug limited rotation
to 0°–90°. A single early electronics bug ended up defining a hard limit used throughout the entire
software stack.

---

# Reproducibility & GitHub Quality

This section is written so that a team with no prior context could rebuild an identical robot from
this repository alone.

## Repository Structure

```
/README.md (this file — full engineering writeup)
/software
  /README.md (setup + how to run instructions)
  /pi5 (vision + decision-layer Python code)
  /pico (real-time control code)
/cad
  /README.md (what each file is, iteration notes — see Section 1.2)
  chassis_iteration1/
  chassis_iteration2/
  *.stl, *.step
/wiring
  /README.md
  circuit_diagram.png
  pin_map.md
/docs
  team_photos/
  performance_videos/ (links per Section 7 of the WRO GitHub requirements)
  dev_log.md
```

At least three meaningful, dated commits are maintained per WRO's documentation timeline
requirement (first commit ≥2 months before competition with ≥1/5 of final code, second ≥1
month before, third ≥2 weeks before), with descriptive messages (e.g. "fix red pillar avoidance
midpoint offset bug") rather than generic ones like "update code."

## Full Parts List (Bill of Materials)

| Part | Purpose | Cost/Link |
|---|---|---|
| Raspberry Pi 5 | Main decision/vision computer | |
| Raspberry Pi Pico 2W | Real-time motor/servo/IMU controller | |
| Sainsmart Wideangle 5MP Camera 160 degree FoV | Lane, wall, and pillar vision | $16.99 - amazon.ca |
| MG95S micro servo | Steering actuator (Ackermann linkage) | |
| L298N motor driver | Drive motor power stage | |
| DC gear motor | Drive motor | |
| WLToys 144001 differential (2:1) | Rear-axle differential | |
| IMU (gyroscope + accelerometer) | Heading correction, parking precision | |
| 12V to 5V Step-down converter | Powers Pi 5, Pico, camera, servo logic | $5.28 - aliexpress.com |
| 12V 3s1p Li-ion battery pack (8800 mAh) | Main power source | |
| Terminal blocks | Safe wiring distribution | |
| Main power switch | Single on/off per WRO rule 9.10 | |
| 3D-printed chassis (base/mid/top layers) | Structural frame — see /cad | |

## Tools Required

- 3D printer capable of printing the chassis layers in /cad
- Screwdriver set (small Phillips/hex, matching the fastener sizes below)
- Soldering iron + solder (for permanent wiring joints)
- Multimeter (for power system verification and battery voltage checks)
- Folding rule/measuring cube (matches what judges use at check time per WRO rule — useful to self-check dimension compliance before competition)

## Fasteners & Connectivity Components

- M3 screws and nuts (chassis layer assembly, servo/motor mounts)
- Nylon cable ties / zip ties (wire management, per WRO rule 11.20 which explicitly permits these)
- Electrical tape (insulation at solder joints)
- Jumper wires (Pico ↔ IMU, Pico ↔ display)
- JST/terminal block connectors (battery ↔ fuse ↔ switch ↔ power rail)

## Build-From-Scratch Guide

1. Print all chassis layers from /cad/chassis_iteration2 (the current, corrected version — see Section 1.2 for why Iteration 1 is kept only for reference, not for rebuilding).
2. Assemble the differential per Section 1.3, using the WLToys 144001 unit.
3. Mount the DC motor and Ackermann steering linkage per Section 1.4, using the printed mounts.
4. Wire the power system per the diagram in /wiring, following the ground-sharing and fuse placement described in Section 2.2 and 2.6.
5. Mount the battery on the first layer (center, flat) and the IMU on the second layer, per the placement reasoning in Section 1.5.
6. Flash the Pico with the code in /software/pico, and set up the Raspberry Pi 5 with the code in /software/pi5 (see that folder's own README for exact setup commands).
7. Run the bench test in Section 5.2, Step 1, before attempting a full track run.

## Possible Improvements

- **Dynamic color calibration:** our HSV ranges are currently fixed for an entire run. A pre-run calibration step against the actual competition lighting would reduce misclassification risk (Section 2.4).
- **Mid-run IMU re-zeroing:** re-zero the IMU against known straight track sections rather than only at the start, to counter drift accumulation over a long run (Section 4.3).
- **Splitting Pico workload:** the Pico currently handles both IMU reading and motor/servo control on one small board. A second board could share that load as complexity grows.
- **Voltage-compensated PWM scaling:** adjust motor PWM output as battery voltage drops, so drive performance stays consistent through the full discharge curve rather than weakening near the L298N's voltage-drop threshold (Section 2.5).
- **Lateral distance sensors for parking:** supplement the camera/IMU parking approach with side-mounted distance sensors to reduce reliance on a single pre-tuned turn angle.
- **Confirmed motor RPM under load** — replace the estimated ~400 rpm figure with a tachometer-measured value.

## Development Log

**[Iteration 1]** Electronics bring-up. Attempted micro:bit ↔ Raspberry Pi connection —
unsuccessful, abandoned. Pivoted to Raspberry Pi Pico ↔ Raspberry Pi. Repeated connection
errors traced to code bugs first, then a power delivery problem — Pico was not receiving
enough power over the shared rail. Fixed by adding a second, dedicated power source.
Separately debugged the steering servo, which was only rotating through 0°–90° of the
expected 180°; traced to a PWM pulse-width mapping bug and corrected it (see Section 1.2,
Section 1.5).

**June 19, 2026** — Built the hardware architecture and circuit diagram for Pi5 ↔ Pico ↔
servo/motor communication (Section 2.1–2.2).

**August 6, 2026** — Debugged the obstacle challenge: the robot wasn't stopping correctly, wasn't
dodging red pillars properly, and had a random −22° offset appearing in steering calculations.
Traced the offset bug and fixed red pillar dodging (Section 3.6).

**August 7, 2026** — Resolved the remaining bug from Aug 6, then shifted focus to implementing
and testing parking logic (Section 3.5).

## Running the Program

To run the program without a connected display, ensure that the `SHOW_VID` variable is set to
`False`. If `SHOW_VID` remains set to `True`, the program may produce errors when running
without an actual screen, as it will attempt to display the camera output.

## Running the Program on Startup

The robot can be configured to automatically start the challenge program when the Raspberry
Pi boots using crontab.

Open the root crontab using:

```bash
sudo crontab -e
```

Add the following line to the end of the crontab, replacing `<path/to>` with the path to the
challenge file:

```bash
@reboot /usr/bin/python3 </path/to>/obs_ch.py >/dev/null 2>&1 &
```

This command automatically runs the Python program whenever the Raspberry Pi starts.

By default, the `>/dev/null 2>&1` portion redirects all standard output and error messages to
`/dev/null`, meaning they will not be saved or displayed. For debugging purposes, a log file
can instead be created using:

```bash
sudo touch /dev/cronlog
```

The crontab command can then be modified to redirect output to this file, allowing debug
messages and errors to be reviewed.

## CAD and Wiring Files

All STL files used for the robot's CAD and 3D-printed components are included in the repository.
All wiring diagrams are also provided, allowing the robot's physical structure and electronic
connections to be reproduced alongside the software and challenge implementations.
