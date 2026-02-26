// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX pivotMotor, rollerMotor;

    public enum Position {
        Stow(0.5),
        Deploy(10);

        public final double value;

        Position(double value) {
            this.value = value;
        }
    }

    public Intake(int pivotMotorID, int rollerMotorID) {
        pivotMotor = new TalonFX(pivotMotorID);
        rollerMotor = new TalonFX(rollerMotorID);

        TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        pivotMotorConfig.Slot0.kP = 10;
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 40;
        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = 60;
        
        TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        rollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotMotor.getConfigurator().apply(pivotMotorConfig);
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public double getVoltage() {
        return pivotMotor.getMotorVoltage().getValueAsDouble();
    }

    public boolean atPosition(Position position) {
        return MathUtil.isNear(position.value, getPosition(), Constants.Intake.tolerance);
    }

    public Command setPositionCmd(Position position) {
        return Commands.run(
            () -> pivotMotor.setControl(new MotionMagicVoltage(position.value)),
            this
        )
        .until(() -> atPosition(position) || getVoltage() > 20);
    }

    public Command setPercentCmd(double percent) {
        return Commands.startEnd(
            () -> rollerMotor.setControl(new VoltageOut(percent * 12)),
            () -> rollerMotor.setControl(new VoltageOut(0)),
            this
        );
    }

    public Command toggleDeployCmd() {
        return Commands.either(
            setPositionCmd(Position.Deploy),
            setPositionCmd(Position.Stow),
            
            () -> atPosition(Position.Stow)
        );
    }

    public Command pulseCmd() {
        return Commands.repeatingSequence(
            setPercentCmd(-1),
            setPositionCmd(Position.Deploy),
            setPositionCmd(Position.Stow),
            Commands.waitSeconds(0.3)
        );
    }
    
    public Command startCmd() {
        return Commands.parallel(
            setPositionCmd(Position.Deploy),
            setPercentCmd(-1)
        );
    }

    @Override
    public void periodic() {}
}