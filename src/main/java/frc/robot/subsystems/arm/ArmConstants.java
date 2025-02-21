package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public class ArmConstants {
    public static final Angle positionError = Degrees.of(3);

    public class l_Angles {
        public static final Angle Base = Degrees.of(-25), // 51.5 horizontal, -35 safe
                Trough = Degrees.of(147.5),
                L2 = Degrees.of(142.5), // 165
                L3 = Degrees.of(142.5), // 165
                L4 = Degrees.of(170),
                Loading = Degrees.of(-21.5),
                Processing = Radians.of(0.0),
                Barge = Degrees.of(40),
                Hold = Degrees.of(40),
                L2Algae = Degrees.of(0.0),
                l3Algae = Degrees.of(0.0),
                Starting = Degrees.of(60);
    }
}
