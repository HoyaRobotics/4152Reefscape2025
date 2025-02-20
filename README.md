# FRC 4152 (To Be Named)

### 2025 Robot With AdvantageKit and MapleSim Simulation

![Robot Image](/assets/2025CAD.png)

---

## Highlights
- TBD

## Using Simulation
When running the simulation, you can see the simulation results through AdvantageScope

- Add `/assets/Robot_2025` to AdvantageScope assets folder
- Included AdvantageScope Layout `/assets/AdvantageScope Layout.json`
- Run code in simulation.

Please drag the following fields:

- Open field `AdvantageKit/RealOutputs/FieldSimulation/RobotPosition` make it `Robot/Robot 2025`
- Open field `AdvantageKit/RealOutputs/Elevator/ElevatorPose` make it component 1 of `Robot/Robot 2025`
- Open field `AdvantageKit/RealOutputs/Elevator/CarriagePose` make it component 2 of `Robot/Robot 2025`
- Open field `AdvantageKit/RealOutputs/Arm/ArmPose` make it component 3 of `Robot/Robot 2025`
- Open field `AdvantageKit/RealOutputs/FieldSimulation/Algae` and make it `Algae`
- Open field `AdvantageKit/RealOutputs/FieldSimulation/Coral` and make it `Coral`
- Open field `AdvantageKit/RealOutputs/Odometry/Robot` and make it `Green Ghost/Robot 2025`

## Controls
![Controller Image](/assets/XboxDiagram.png)
### Driver Controller
- left Stick: Swerve XY Translation.
- Right Stick: Swerve Rotation.
- Left Trigger: Place on commanded level.
- Right Trigger: Intake from loading station.
- left Stick Button (Our controller has back paddles mapped to the stick buttons): Automaticaly align to left reef of closest reef face.
- Right Stick Button (Our controller has back paddles mapped to the stick buttons): Automaticaly align to right reef of closest reef face.
- B: Move to Trough position.
- A: Move to L2 position.
- X: Move to L3 position.
- Y: Move to L4 position.
- Start: reset odometry.
