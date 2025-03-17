package frc.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public class ArmConstants {
    public static final Angle positionError = Degrees.of(1.5);
    public static final Angle startingAngle = Degrees.of(60);
    public static final Angle baseAngle = Degrees.of(-65);
    public static final Angle safeRetractAngle = Degrees.of(150);

    public static final Angle zeroAngle = Degrees.of(120);
}
