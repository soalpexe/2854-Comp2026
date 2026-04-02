// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase {
    public enum Substate {
        OFF,
        ON,
        UNJAM
    }

    private TalonFX motor;

    private Substate substate;
    private VoltageOut voltageRequest;

    public Transfer(int motorID) {
        motor = new TalonFX(motorID);

        substate = Substate.OFF;
        voltageRequest = new VoltageOut(0);

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(motorConfig);
    }

    public void requestPercent(double percent) {
        motor.setControl(voltageRequest.withOutput(percent * 12));
    }

    public Command setSubstateCmd(Substate substate) {
        return Commands.runOnce(() -> this.substate = substate);
    }

    @Override
    public void periodic() {
        switch (substate) {
            case OFF:
                requestPercent(0);
                break;

            case ON:
                requestPercent(-1);
                break;

            case UNJAM:
                requestPercent(1);
                break;

            default:
                break;
        }
    }
}