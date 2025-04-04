// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final IntakeVersion intakeVersion = IntakeVersion.V2;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static enum IntakeVersion {
        /** V1 Coral Intake */
        V1,

        /** V2 Coral And Algae Intake */
        V2
    }

    public static final boolean useVariableIntakeHeight = true;
    // DO NOT USE PROFILING RIGHT NOW
    public static final boolean TeleopMotionProfiling = false;
    public static final boolean AutoMotionProfiling = false;
    public static final boolean FuseDriverInputs = true;
}

// align to trough and coral station (closest spot along coral station)
// commands that extend drive to pose
