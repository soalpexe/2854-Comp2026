// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    public enum Substate {
        STOW,
        OFF,
        ON,
        RAMP,
        UNJAM
    }

    public enum Position {
        STOW(2.6),
        RAMP(7),
        DEPLOY(10.5);

        public final double value;

        Position(double value) {
            this.value = value;
        }
    }

    private TalonFX pivotMotor, topRollerMotor, bottomRollerMotor;

    private Substate substate;
    private MotionMagicVoltage positionRequest;
    private VoltageOut voltageRequest;

    public Intake(int pivotMotorID, int topRollerMotorID, int bottomRollerMotorID) {
        pivotMotor = new TalonFX(pivotMotorID);
        topRollerMotor = new TalonFX(topRollerMotorID);
        bottomRollerMotor = new TalonFX(bottomRollerMotorID);

        substate = Substate.STOW;
        positionRequest = new MotionMagicVoltage(0);
        voltageRequest = new VoltageOut(0);

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        pivotMotorConfig.Slot0.kP = 10;
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 200;
        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = 100;

        pivotMotor.getConfigurator().apply(pivotMotorConfig);
    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public boolean atPosition(Position position) {
        return MathUtil.isNear(position.value, getPosition(), Constants.Intake.tolerance);
    }

    public void requestPosition(Position position) {
        pivotMotor.setControl(positionRequest.withPosition(position.value));
    }

    public void requestPercent(double percent) {
        topRollerMotor.setControl(voltageRequest.withOutput(percent * 12));
        bottomRollerMotor.setControl(voltageRequest.withOutput(percent * 12));
    }
    
    public Command setSubstateCmd(Substate substate) {
        return Commands.runOnce(() -> this.substate = substate);
    }

    @Override
    public void periodic() {
        switch (substate) {
            case STOW:
                requestPosition(Position.STOW);
                requestPercent(0);
                break;
                
            case OFF:
                requestPosition(Position.DEPLOY);
                requestPercent(0);
                break;
                
            case ON:
                requestPosition(Position.DEPLOY);
                requestPercent(1);
                break;

            case RAMP:
                requestPosition(Position.RAMP);
                requestPercent(1);
                break;
                
            case UNJAM:
                requestPosition(Position.DEPLOY);
                requestPercent(-1);
                break;
        
            default:
                break;
        }
    }
}