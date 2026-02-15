// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private TalonFX pivotMotor, rollerMotor;

    public enum Position {
        Stow(0.5),
        Pulse(6.5),
        Deploy(9);

        public double value;

        Position(double value) {
            this.value = value;
        }
    }

    public Intake(int pivotMotorID, int rollerMotorID) {
        pivotMotor = new TalonFX(pivotMotorID);
        rollerMotor = new TalonFX(rollerMotorID);

        TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotMotorConfig.Slot0.kP = 10;
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 40;
        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = 60;

        pivotMotor.getConfigurator().apply(pivotMotorConfig);
    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public boolean atPosition(Position position) {
        return MathUtil.isNear(position.value, getPosition(), 0.2);
    }

    public Command setPositionCmd(Position position) {
        return Commands.run(
            () -> pivotMotor.setControl(new MotionMagicExpoVoltage(position.value)),
            this
        )
        .until(() -> atPosition(position));
    }

    public Command setRollerOutCmd(double percent) {
        return Commands.runOnce(
            () -> rollerMotor.setControl(new DutyCycleOut(percent)),
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
        return Commands.sequence(
            setRollerOutCmd(-1),
            Commands.repeatingSequence(
                setPositionCmd(Position.Deploy),
                setPositionCmd(Position.Pulse),
                Commands.waitSeconds(0.3)
            )
        );
    }
    
    public Command startCmd() {
        return Commands.sequence(
            setPositionCmd(Position.Deploy),
            setRollerOutCmd(-1)
        );
    }

    public Command stopCmd() {
        return setRollerOutCmd(0);
    }

    @Override
    public void periodic() {}
}