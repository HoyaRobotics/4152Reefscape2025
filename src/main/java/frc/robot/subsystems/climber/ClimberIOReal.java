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
    private double climberRatio = 5.0 * 5.0 * 4.0 * (68.0 / 18.0) * (32.0 / 12.0);

    public ClimberIOReal() {
        configureMotors();
        closedLoopController = climberMotor.getClosedLoopController();
        climberEncoder = climberMotor.getEncoder();
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.climberAngle = Rotations.of(climberEncoder.getPosition() / climberRatio);
    }

    @Override
    public void setAngle(Angle targetAngle, boolean fast) {
        if (fast) {
            closedLoopController.setReference(
                    targetAngle.in(Rotations) * climberRatio, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        } else {
            closedLoopController.setReference(
                    targetAngle.in(Rotations) * climberRatio, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
    }

    @Override
    public void setVoltage(Voltage volts) {
        climberMotor.setVoltage(volts);
    }

    private void configureMotors() {
        SparkFlexConfig climberMotorConfig = new SparkFlexConfig();

        climberMotorConfig
                .softLimit
                .forwardSoftLimit(.3 * climberRatio)
                .reverseSoftLimit(-0.005 * climberRatio)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);

        climberMotorConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(5, ClosedLoopSlot.kSlot0)
                .outputRange(-8 / 12.0, 8 / 12.0, ClosedLoopSlot.kSlot0)
                .p(5, ClosedLoopSlot.kSlot1)
                .outputRange(-12 / 12.0, 12 / 12.0, ClosedLoopSlot.kSlot1);
        // climberMotorConfig.idleMode(IdleMode.kCoast);
        climberMotorConfig.idleMode(IdleMode.kBrake);
        climberMotorConfig.smartCurrentLimit(60);
        climberMotorConfig.inverted(false);

        climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // climberEncoder.setPosition(ClimberConstants.baseAngle.in(Rotations));
    }
}
