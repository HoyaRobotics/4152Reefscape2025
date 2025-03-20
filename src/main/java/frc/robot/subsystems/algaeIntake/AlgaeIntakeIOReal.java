// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class AlgaeIntakeIOReal implements AlgaeIntakeIO {
    private final TalonFX algaeIntakeMotor = new TalonFX(37, "rio");
    private final LaserCan laserCan = new LaserCan(38);
    double intakeRatio = (30.0 / 15.0) * (44.0 / 8.0);

    private VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private VoltageOut voltageRequest = new VoltageOut(0.0);

    LinearFilter filter = LinearFilter.movingAverage(1);

    public AlgaeIntakeIOReal() {
        configureMotors();
        configureLaserCan();
    }

    public boolean hasAlgaeCurrent() {
        double current = filter.calculate(algaeIntakeMotor.getStatorCurrent().getValueAsDouble());
        return current >= 5.0;
    }

    public boolean hasAlgaeSensed() {
        Measurement measurement = laserCan.getMeasurement();

        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            double measureDistance = measurement.distance_mm;
            double algaeDistance = Inches.of(4.0).in(Millimeters);

            return measureDistance <= algaeDistance;
        } else {
            return false;
        }
    }

    private void configureLaserCan() {
        try {
            laserCan.setRangingMode(RangingMode.SHORT);
            laserCan.setRegionOfInterest(new RegionOfInterest(4, 4, 8, 8));
            laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    private void configureMotors() {
        TalonFXConfiguration algaeIntakeMotorConfig = new TalonFXConfiguration();
        algaeIntakeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        algaeIntakeMotorConfig.CurrentLimits.StatorCurrentLimit = 15;
        algaeIntakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        algaeIntakeMotorConfig.Feedback.SensorToMechanismRatio = intakeRatio;
        algaeIntakeMotorConfig.Voltage.PeakForwardVoltage = 12.0;
        algaeIntakeMotorConfig.Voltage.PeakReverseVoltage = -12.0;
        algaeIntakeMotorConfig.Slot0.kV =
                1.19; // 0.295 for a 30/11 gear ratio , 0.472 48 / 11, 0.649 48 / 8, 1.19 11/1s
        algaeIntakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        algaeIntakeMotor.getConfigurator().apply(algaeIntakeMotorConfig);
    }

    @Override
    public void setSpeed(AngularVelocity targetSpeed) {
        algaeIntakeMotor.setControl(velocityRequest.withVelocity(targetSpeed).withSlot(0));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        algaeIntakeMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void stop() {
        algaeIntakeMotor.stopMotor();
    }

    @Override
    public void setCurrentLimit(Current currentLimit) {
        CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = currentLimit.in(Amp);
        currentLimitConfigs.StatorCurrentLimitEnable = true;

        algaeIntakeMotor.getConfigurator().apply(currentLimitConfigs);
    }

    @Override
    public void updateInputs(AlgaeIntakeInputs inputs) {
        // check sensors for game piece
        inputs.speed = RotationsPerSecond.of(algaeIntakeMotor.getVelocity().getValueAsDouble());
        inputs.hasAlgae = hasAlgaeSensed();
    }
}
