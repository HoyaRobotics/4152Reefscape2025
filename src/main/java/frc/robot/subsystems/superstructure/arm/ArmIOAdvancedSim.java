// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ArmIOAdvancedSim implements ArmIO {
    final TalonFX armMotor = new TalonFX(32, "rio");
    final CANcoder armEncoder = new CANcoder(33, "rio");
    double armRatio = 185.7143;
    private SingleJointedArmSim armSim;

    private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0.0);

    public ArmIOAdvancedSim() {
        configureArmMotor();
        armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                armRatio,
                0.16931712,
                0.28778209,
                ArmConstants.baseAngle.in(Radians),
                Units.degreesToRadians(180.0),
                false,
                ArmConstants.startingAngle.in(Radians));
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        var armEncoderSim = armEncoder.getSimState();
        armEncoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        armEncoderSim.Orientation = ChassisReference.CounterClockwise_Positive;
        var armMotorSim = armMotor.getSimState();
        armMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        armMotorSim.Orientation = ChassisReference.Clockwise_Positive;

        var motorVoltage = armMotorSim.getMotorVoltageMeasure();

        armSim.setInputVoltage(motorVoltage.in(Volts));
        armSim.update(0.02);

        armEncoderSim.setMagnetHealth(MagnetHealthValue.Magnet_Green);
        armEncoderSim.setRawPosition(Radians.of(armSim.getAngleRads()));
        armEncoderSim.setVelocity(RadiansPerSecond.of(armSim.getVelocityRadPerSec()));
        armMotorSim.setRawRotorPosition(Radians.of(armSim.getAngleRads()).times(armRatio));
        armMotorSim.setRotorVelocity(
                RadiansPerSecond.of(armSim.getVelocityRadPerSec()).times(armRatio));

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
        armEncoderConfig.MagnetSensor.MagnetOffset = 0.117;
        armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        armEncoder.getConfigurator().apply(armEncoderConfig);
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        armConfig.CurrentLimits.StatorCurrentLimit = 60;
        armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        armConfig.Feedback.FeedbackRemoteSensorID = 33;
        armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        armConfig.Feedback.RotorToSensorRatio = armRatio;
        armConfig.MotionMagic.MotionMagicAcceleration = 1.0; // 0.4
        armConfig.MotionMagic.MotionMagicCruiseVelocity = 0.6;
        armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armConfig.Slot0.kG = 0.01;
        armConfig.Slot0.kS = 0.15;
        armConfig.Slot0.kV = 21.04111;
        armConfig.Slot0.kA = 0.0;
        armConfig.Slot0.kP = 100;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.7;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.12;
        armConfig.Voltage.PeakForwardVoltage = 10.0;
        armConfig.Voltage.PeakReverseVoltage = 10.0;

        armMotor.getConfigurator().apply(armConfig);
    }
}
