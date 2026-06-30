# Autonomous Frontier Explorer

Autonomous exploration and occupancy-grid mapping for a Pioneer 3-DX robot in Webots. The robot builds a probabilistic map from ultrasonic sensors, selects frontier targets, navigates around obstacles, and recovers from loops or physical stalls without predefined waypoints.

![Pioneer 3-DX exploring the complex arena](assets/images/complex_environment.jpg)

## Highlights

- Bayesian log-odds occupancy-grid mapping
- Dynamic frontier detection and target selection
- Hybrid Bug navigation with PD wall following
- Loop, bumper, timeout, and stalled-motion recovery
- Automatic completion after high map coverage
- Simple and complex Webots demonstration worlds

## Demo

[Watch the complex-arena run](assets/demo/autonomous_exploration.mp4)

The recorded run reaches approximately 97.6% observed map coverage. A detailed design discussion is available in the [technical report](docs/technical_report.pdf), with a readable [Markdown version](docs/TECHNICAL_REPORT.md).

## Requirements

- [Webots R2025a](https://cyberbotics.com/) or a compatible newer release
- Python 3, provided by Webots

No additional Python packages are required.

## Run the simulation

1. Clone the repository.
2. Open `worlds/PA1_complex.wbt` in Webots.
3. Press **Run**.
4. Observe the robot and the live occupancy-grid display.

For a smaller environment, open `worlds/PA1_simple.wbt`.

## How it works

The controller begins with a 360-degree scan, updates a Bayesian occupancy grid from eight ultrasonic sensors, and searches for boundaries between known free space and unknown space. A hybrid Bug controller drives toward each selected frontier and switches to wall following when the direct path is blocked. Recovery logic abandons unreachable goals and frees the robot after collisions or stalled motion.

## Project structure

```text
controllers/autonomous_explorer/  Python controller, mapping, navigation, sensors
worlds/                            Webots simulation worlds
assets/demo/                       Recorded autonomous run
assets/images/                     Project preview images
docs/                              Technical report
```

## Technical report

The report covers the sensor model, log-odds mapping, frontier scoring, navigation state machine, recovery behaviours, performance optimisation, and observed results:

- [Technical report (PDF)](docs/technical_report.pdf)
- [Technical report (Markdown)](docs/TECHNICAL_REPORT.md)

## Acknowledgements

Built with [Webots](https://cyberbotics.com/) and the Pioneer 3-DX simulation model.
