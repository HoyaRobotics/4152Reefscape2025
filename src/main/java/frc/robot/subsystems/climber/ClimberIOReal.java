// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {

    SparkFlex climberMotor = new SparkFlex(0, MotorType.kBrushless);
    private double climberRatio = 5.0 * 5.0 * (68.0 / 18.0) * (32.0 / 12.0);

    public ClimberIOReal() {
        configureMotors();
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        double encoderPosition = climberMotor.getEncoder().getPosition();
        inputs.angle = Rotations.of(encoderPosition);
    }

    @Override
    public void setAngle(Angle targetAngle) {}

    @Override
    public void setVoltage(Voltage volts) {
        climberMotor.setVoltage(volts);
    }

    private void configureMotors() {
        SparkFlexConfig climberMotorConfig = new SparkFlexConfig();

        climberMotorConfig.encoder.positionConversionFactor(1.0 / climberRatio);
        climberMotorConfig.closedLoop.maxMotion.maxAcceleration(0.1);
        climberMotorConfig.closedLoop.maxMotion.maxVelocity(0.1);
        climberMotorConfig.encoder.inverted(false);
        climberMotorConfig.softLimit.forwardSoftLimit(0.3);
        climberMotorConfig.softLimit.reverseSoftLimit(-0.26);
        climberMotorConfig.softLimit.forwardSoftLimitEnabled(true);
        climberMotorConfig.softLimit.reverseSoftLimitEnabled(true);

        climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
