// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {

    SparkFlex climberMotor = new SparkFlex(36, MotorType.kBrushless);
    SparkClosedLoopController closedLoopController;
    RelativeEncoder climberEncoder;
    private double climberRatio = 5.0 * 5.0 * (68.0 / 18.0) * (32.0 / 12.0);

    public ClimberIOReal() {
        configureMotors();
        closedLoopController = climberMotor.getClosedLoopController();
        climberEncoder = climberMotor.getEncoder();
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.climberAngle = Rotations.of(climberEncoder.getPosition());
    }

    @Override
    public void setAngle(Angle targetAngle, boolean fast) {
        if (fast) {
            closedLoopController.setReference(targetAngle.in(Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot0);
        } else {
            closedLoopController.setReference(targetAngle.in(Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot1);
        }
    }

    @Override
    public void setVoltage(Voltage volts) {
        climberMotor.setVoltage(volts);
    }

    private void configureMotors() {
        SparkFlexConfig climberMotorConfig = new SparkFlexConfig();

        climberMotorConfig
                .closedLoop
                .maxMotion
                .maxAcceleration(0.1, ClosedLoopSlot.kSlot0)
                .maxVelocity(0.1, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(0.01, ClosedLoopSlot.kSlot0)
                .maxAcceleration(0.1, ClosedLoopSlot.kSlot1)
                .maxVelocity(0.1, ClosedLoopSlot.kSlot1)
                .allowedClosedLoopError(0.01, ClosedLoopSlot.kSlot1);
        /*
        climberMotorConfig.encoder
            .positionConversionFactor(1.0 / climberRatio)
            .velocityConversionFactor(1.0 / climberRatio)
            .inverted(false);
        */
        /*
        climberMotorConfig.softLimit
            .forwardSoftLimit(0.3);
            .reverseSoftLimit(-0.26);
            .forwardSoftLimitEnabled(true);
            .reverseSoftLimitEnabled(true);
        */
        climberMotorConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.0, ClosedLoopSlot.kSlot0)
                .velocityFF(0.0, ClosedLoopSlot.kSlot0)
                .outputRange(-1, 1);
        climberMotorConfig.idleMode(IdleMode.kBrake);
        climberMotorConfig.smartCurrentLimit(60);
        // climberMotorConfig.voltageCompensation(11);

        climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // climberEncoder.setPosition(ClimberConstants.baseAngle.in(Rotations));
    }
}
