// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeMotor = new TalonFX(4, "rio");
    private final LaserCan lasercan = new LaserCan(60);
    double intakeRatio = 60.0 / 14; // Meters

    private VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    public IntakeIOReal() {
        configureMotors();
        configureLaserCan();
    }

    public boolean hasCoral() {
        Measurement measurement = lasercan.getMeasurement();

        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            double measureDistance = measurement.distance_mm;
            double coralDistance = Inches.of(5).in(Millimeters);

            return measureDistance <= coralDistance;
        } else {
            return false;
        }
    }

    private void configureLaserCan() {
        try {
            lasercan.setRangingMode(RangingMode.SHORT);
            lasercan.setRegionOfInterest(new RegionOfInterest(4, 4, 8, 8));
            lasercan.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    private void configureMotors() {
        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
        intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeMotorConfig.Feedback.SensorToMechanismRatio = intakeRatio;
        intakeMotorConfig.Voltage.PeakForwardVoltage = 11.0;
        intakeMotorConfig.Voltage.PeakReverseVoltage = 11.0;
        intakeMotorConfig.Slot0.kV = 0.0;
    }

    @Override
    public void setSpeed(AngularVelocity targetSpeed) {
        intakeMotor.setControl(velocityRequest.withVelocity(targetSpeed));
    }

    @Override
    public void stop() {
        setSpeed(RotationsPerSecond.of(0));
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        // check sensors for game piece
        inputs.speed = RotationsPerSecond.of(intakeMotor.getVelocity().getValueAsDouble());
        inputs.hasGamePiece = hasCoral();
    }
}
