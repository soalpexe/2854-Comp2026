// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private TalonFX motor;

    public enum Position {
        STOW(0),
        DEPLOY(80);

        public final double value;

        Position(double value) {
            this.value = value;
        }
    }

    public Climber(int motorID) {
        motor = new TalonFX(motorID);
        
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        motorConfig.Slot0.kP = 10;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
        motorConfig.MotionMagic.MotionMagicAcceleration = 120;

        motor.getConfigurator().apply(motorConfig);
    }

    public boolean atPosition(Position position) {
        return MathUtil.isNear(position.value, getPosition(), Constants.Climber.tolerance);
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public Command setPositionCmd(Position position) {
        return Commands.run(
            () -> motor.setControl(new MotionMagicVoltage(position.value)),
            this
        )
        .until(() -> atPosition(position));
    }

    public Command togglePositionCmd() {
        return Commands.either(
            setPositionCmd(Position.DEPLOY),
            setPositionCmd(Position.STOW),
            () -> atPosition(Position.STOW)
        );
    }

    @Override
    public void periodic() {}
}
