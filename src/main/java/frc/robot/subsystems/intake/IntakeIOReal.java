// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO {
    final TalonFX intakeMotor = new TalonFX(4, "rio");
    double intakeRatio = 60.0 / 14; // Meters

    private VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    public IntakeIOReal() {
        configureMotors();
    }

    private void configureMotors(){
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

    }

    @Override
    public void updateInputs(IntakeInputs inputs){

    }
}
