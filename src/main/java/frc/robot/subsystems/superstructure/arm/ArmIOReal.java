// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

/** Add your docs here. */
public class ArmIOReal implements ArmIO {
    final TalonFX armMotor = new TalonFX(32, "rio");
    final CANcoder armEncoder = new CANcoder(33, "rio");
    double armRatio = 185.7143;

    private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0.0);

    public ArmIOReal() {
        configureArmMotor();
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.armAngle = armMotor.getPosition(true).getValue();
    }

    @Override
    public void setPosition(Angle targetAngle) {
        armMotor.setControl(magicRequest.withPosition(targetAngle).withSlot(0));
    }

    @Override
    public void stop() {
        armMotor.stopMotor();
    }

    private void configureArmMotor() {
        CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
        armEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.8;
        // armEncoderConfig.MagnetSensor.MagnetOffset = 0.117;
        armEncoderConfig.MagnetSensor.MagnetOffset = 0.075439;
        armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        armEncoder.getConfigurator().apply(armEncoderConfig);
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        armConfig.CurrentLimits.StatorCurrentLimit = 30; // 60
        armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        armConfig.Feedback.FeedbackRemoteSensorID = 33;
        armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        armConfig.Feedback.RotorToSensorRatio = armRatio;
        armConfig.MotionMagic.MotionMagicAcceleration = 2.0; // 0.3
        armConfig.MotionMagic.MotionMagicCruiseVelocity = 0.8;
        armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armConfig.Slot0.kG = 0.01;
        armConfig.Slot0.kS = 0.15;
        armConfig.Slot0.kV = 21.04111;
        armConfig.Slot0.kA = 0.0;
        armConfig.Slot0.kP = 45;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(180.0);
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.12;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(-65.0);
        armConfig.Voltage.PeakForwardVoltage = 10.0;
        armConfig.Voltage.PeakReverseVoltage = 10.0;

        armMotor.getConfigurator().apply(armConfig);
    }
}
