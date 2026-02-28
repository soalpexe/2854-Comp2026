// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private TalonFX leftMotor, rightMotor;

    public Shooter(int leftMotorID, int rightMotorID) {
        leftMotor = new TalonFX(leftMotorID);
        rightMotor = new TalonFX(rightMotorID);

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        leftMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        rightMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        rightMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leftMotor.getConfigurator().apply(leftMotorConfig);
        rightMotor.getConfigurator().apply(rightMotorConfig);
    }

    public Command setPercentCmd(double percent) {
        return Commands.runOnce(
            () -> {
                leftMotor.setControl(new VoltageOut(percent * 12));
                rightMotor.setControl(new VoltageOut(-percent * 12));
            },
            this
        );
    }

    @Override
    public void periodic() {}
}
