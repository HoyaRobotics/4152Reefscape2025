# FRC 4152 Bruce - 2025 Reefscape

### With AdvantageKit and MapleSim Simulation

![Robot Image](/assets/2025BruceCAD.png)

---

## Highlights
- 4 coral L4 autonomous on both sides
- full coral simulation and loading staion intaking

## Using Simulation
When running the simulation, you can see the simulation results through AdvantageScope

- Add `/assets/Robot_2025 V3` to AdvantageScope assets folder
- Included AdvantageScope Layout `/assets/AdvantageScope Layout.json`
- Run code in simulation.

Please drag the following fields:

- Open field `AdvantageKit/RealOutputs/FieldSimulation/RobotPosition` make it `Robot/Robot 2025 V3`
- Open field `AdvantageKit/RealOutputs/Elevator/ElevatorPose` make it component 1 of `Robot/Robot 2025 V3`
- Open field `AdvantageKit/RealOutputs/Elevator/CarriagePose` make it component 2 of `Robot/Robot 2025 V3`
- Open field `AdvantageKit/RealOutputs/Arm/ArmPose` make it component 3 of `Robot/Robot 2025 V3`
- Open field `AdvantageKit/RealOutputs/Climber/ClimberPose` make it component 3 of `Robot/Robot 2025 V3`
- Open field `AdvantageKit/RealOutputs/FieldSimulation/Algae` and make it `Algae`
- Open field `AdvantageKit/RealOutputs/FieldSimulation/Coral` and make it `Coral`
- Open field `AdvantageKit/Intake/CoralInIntake` and make it `Coral`
- Open field `AdvantageKit/RealOutputs/Odometry/Robot` and make it `Green Ghost/Robot 2025 V3`
- Open field `AdvantageKit/RealOutputs/DriveToPose/targetPose` and make it `Blue Ghost/Robot 2025 V3`

## Controls
![Controller Image](/assets/XboxDiagram.png)
### Driver Controller
- left Stick: swerve XY translation.
- Right Stick: swerve rotation.
- Left Trigger: place In processor (auto drive).
- Right Trigger: intake from loading station.
- Right Trigger + Left Trigger: intake from loading station (auto drive to closest loading station position).
- Left Bumper: place in net (auto drive to closest alliance barge position).
- Right Bumper: grab algae out of reef (auto drive to closest reef face and knows which level to grab from).
- left Stick Button (Our controller has back paddles mapped to the stick buttons): automaticaly align to left reef of closest reef face and place pre selected level.
- Right Stick Button (Our controller has back paddles mapped to the stick buttons): automaticaly align to right reef of closest reef face and place pre selected level.
- B: move to trough position and score on second click.
- A: Pre select L2 position and remembers position until changed.
- X: Pre select L3 position and remembers position until changed.
- Y: Pre select L4 position and remembers position until changed.
- Start: reset odometry.
- Back: reset elevator encoder.
- POV Up: prepare climber for climbing.
- POV Down: climb.
- POV Left: lower climber and reset other subsystems to defult positions
- POV Right: grab lollipop algae (auto drive to closest lollipop algae).
