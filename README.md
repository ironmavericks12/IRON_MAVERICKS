# Iron Mavericks

## WRO Future Engineers 2026

Welcome to the official engineering repository of **Iron Mavericks**, a team participating in the **WRO Future Engineers 2026** competition.

We are a student robotics team from **India**, working together to design, build, program and test an autonomous self-driving robot. Our goal is to combine mechanical design, electronics, programming and computer vision into one reliable autonomous system.

This repository presents our team's robot, engineering approach, competition software, CAD model, photographs and performance material.

---

## 👥 Our Team

**Team Name:** Iron Mavericks  
**Country:** India  
**School / Organization:** ROBOFUN LABS  
**Coach:** `Mohit Solanki`  
**Team Members:** `Ahan Singhal, Sonakshi Sanyal, Udaiveer Singh`

Iron Mavericks is built around teamwork and practical engineering. Each stage of the robot's development involves designing, building, programming, testing and improving the system.

We believe that a successful competition robot is not only about the final result. It is also about understanding the problem, identifying failures, testing solutions and continuously improving the design.

---

## 🤖 Our Robot

Our WRO Future Engineers robot is an autonomous self-driving vehicle controlled by a **Raspberry Pi 5 (8GB)**.

The robot uses a **Raspberry Pi Camera Module 3 Wide Angle** as its primary perception system. Camera images are processed using Python and OpenCV to understand the track, identify colored markers and detect obstacles.

The robot then makes driving decisions and controls its drive motor and steering servo automatically.

---

## 🧠 Our Approach

The main focus of our robot is **vision-based autonomous driving**.

Instead of relying on a fixed sequence of movements, the robot continuously observes the track using the camera and changes its steering according to what it sees.

Our software combines:

- Computer vision
- LAB color processing
- HSV color processing
- Track detection
- Color-marker detection
- Direction detection
- Proportional steering control
- Lap counting
- Obstacle detection
- Obstacle avoidance

This allows the robot to make decisions while it is moving on the track.

---

## 🏁 Open Round

For the Open Round, the robot uses the camera to detect the black track and colored direction markers.

The first colored marker determines the driving direction:

```text
Blue first   → Anticlockwise
Orange first → Clockwise
```

The robot then follows the track using camera-based detection and proportional steering control.

The system also counts the relevant marker crossings and stops after completing the configured number of laps.

The Open Round program is available here:

```text
src/Open_Round.py
```

---

## 🚧 Obstacle Round

For the Obstacle Round, we extended the same autonomous driving system with obstacle detection and avoidance.

The robot detects red and green obstacles using a combination of **HSV and LAB color information**.

Our supplied avoidance strategy is:

```text
RED obstacle   → Pass on the RIGHT
GREEN obstacle → Pass on the LEFT
```

The obstacle system does not completely replace normal line following. Instead, it uses the normal steering calculation as the base and adds an obstacle-avoidance correction when a sufficiently close obstacle is detected.

The Obstacle Round program is available here:

```text
src/Obstacle_Round.py
```

---

## 🔧 Engineering and Development

Our robot was developed as an integrated mechanical, electrical and software system.

### Mechanical

The vehicle design and final robot model are provided in:

```text
models/
```

The repository contains our final STEP model:

```text
models/Iron_Mavericks_Robot.STEP
```

### Electronics

The robot uses a Raspberry Pi 5 as the central controller. Motor control is handled by the TB6612FNG and steering is controlled using an MG90S servo.

The power system uses a 3-cell 18650 Li-ion battery pack with regulated power supplies for the Raspberry Pi, servo and auxiliary electronics.

### Software

The autonomous software is written in Python and uses:

- OpenCV
- NumPy
- Picamera2
- Raspberry Pi GPIO

The source code is available in:

```text
src/
```

---

## 📷 Our Robot

We have included six views of the vehicle:

- Front
- Rear
- Left
- Right
- Top
- Bottom

All vehicle photographs are available in:

```text
v-photos/
```

---

## 🎥 Performance

Our Open Round performance video is included in:

```text
video/Open_Round.mp4
```

The `video/` directory contains the performance-video documentation for the repository.

---

## 📁 Repository Contents

The repository follows the engineering-materials structure used for WRO Future Engineers documentation.

| Folder | Contents |
|---|---|
| `t-photos` | Team photographs |
| `v-photos` | Vehicle photographs |
| `video` | Performance videos and video documentation |
| `schemes` | Electrical and electromechanical schematics |
| `src` | Competition source code |
| `models` | CAD and manufacturing models |

---

## 🔬 Engineering Philosophy

Our development process is based on:

**Design → Build → Program → Test → Identify Problems → Improve → Test Again**

We treat failures during testing as part of the engineering process. Vision thresholds, steering parameters, obstacle behavior and mechanical components are tested and adjusted based on the behavior of the actual robot.

Our objective is to develop a robot that is not only capable of completing the track, but also demonstrates a clear relationship between its mechanical design, electronics, software and autonomous decision-making.

---

## 🌟 Why Iron Mavericks?

The name **Iron Mavericks** represents our approach to robotics:

- **Iron** — strong engineering and a robust physical robot.
- **Mavericks** — independent thinking, experimentation and creative problem solving.

As a team, we aim to learn from every test, improve our design and work together to solve the challenges presented by autonomous robotics.

---

## 🏆 WRO Future Engineers 2026

This repository contains the engineering materials for our participation in **WRO Future Engineers 2026**.

We have included our competition source code, vehicle photographs, CAD model and available performance material so that the development of our autonomous robot can be understood as a complete engineering project.


---

**Iron Mavericks — WRO Future Engineers 2026**  
**Design. Build. Program. Test. Improve.**
