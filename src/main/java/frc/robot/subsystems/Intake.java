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
        STOW(0.5),
        RAMP(6),
        DEPLOY(11);

        public final double value;

        Position(double value) {
            this.value = value;
        }
    }

    private TalonFX pivotMotor, rollerMotor;

    private Substate substate;
    private MotionMagicVoltage positionRequest;
    private VoltageOut voltageRequest;

    public Intake(int pivotMotorID, int rollerMotorID) {
        pivotMotor = new TalonFX(pivotMotorID);
        rollerMotor = new TalonFX(rollerMotorID);

        TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotMotorConfig.Slot0.kP = 10;
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 40;
        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = 60;
        
        TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        rollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotMotor.getConfigurator().apply(pivotMotorConfig);
        rollerMotor.getConfigurator().apply(rollerMotorConfig);

        substate = Substate.STOW;
        positionRequest = new MotionMagicVoltage(0);
        voltageRequest = new VoltageOut(0);
    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public boolean atPosition(Position position) {
        return MathUtil.isNear(position.value, getPosition(), Constants.Intake.tolerance);
    }

    public void setPosition(Position position) {
        pivotMotor.setControl(positionRequest.withPosition(position.value));
    }

    public void setPercent(double percent) {
        rollerMotor.setControl(voltageRequest.withOutput(percent * 12));
    }
    
    public Command setSubstateCmd(Substate substate) {
        return Commands.runOnce(() -> this.substate = substate);
    }

    @Override
    public void periodic() {
        switch (substate) {
            case STOW:
                setPosition(Position.STOW);
                setPercent(0);
                break;
                
            case OFF:
                setPosition(Position.DEPLOY);
                setPercent(0);
                break;
                
            case ON:
                setPosition(Position.DEPLOY);
                setPercent(1);
                break;

            case RAMP:
                setPosition(Position.RAMP);
                setPercent(1);
                break;
                
            case UNJAM:
                setPosition(Position.DEPLOY);
                setPercent(-1);
                break;
        
            default:
                break;
        }
    }
}