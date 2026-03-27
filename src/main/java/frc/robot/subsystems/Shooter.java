// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShotCalculator;
import frc.robot.ShotCalculator.ShotParameters;

public class Shooter extends SubsystemBase {
    public enum Substate {
        IDLE,
        REV,
        UNJAM
    }

    private TalonFX leftFlywheelMotor, rightFlywheelMotor, hoodMotor;

    private Substate substate;
    private VelocityVoltage velocityRequest;
    private PositionVoltage positionRequest;

    public Shooter(int leftFlywheelMotorID, int rightFlywheelMotorID, int hoodMotorID) {
        leftFlywheelMotor = new TalonFX(leftFlywheelMotorID);
        rightFlywheelMotor = new TalonFX(rightFlywheelMotorID);
        hoodMotor = new TalonFX(hoodMotorID);

        substate = Substate.IDLE;
        velocityRequest = new VelocityVoltage(0);
        positionRequest = new PositionVoltage(0);

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration flywheelMotorConfig = new TalonFXConfiguration();
        flywheelMotorConfig.Slot0.kS = 0.13;
        flywheelMotorConfig.Slot0.kV = 0.115;

        TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();
        hoodMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        hoodMotorConfig.Slot0.kP = 20;

        leftFlywheelMotor.getConfigurator().apply(flywheelMotorConfig);
        rightFlywheelMotor.getConfigurator().apply(flywheelMotorConfig);
        hoodMotor.getConfigurator().apply(hoodMotorConfig);
    }

    public Rotation2d positionToAngle(double position) {
        return Rotation2d.fromRotations(position / Constants.Shooter.motorToHoodRatio).plus(Constants.Shooter.minHardstopAngle);
    }

    public double angleToPosition(Rotation2d angle) {
        return angle.minus(Constants.Shooter.minHardstopAngle).getRotations() * Constants.Shooter.motorToHoodRatio;
    }

    public double getPosition() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    public Rotation2d getAngle() {
        return positionToAngle(getPosition());
    }

    public void requestRPS(double rps) {
        leftFlywheelMotor.setControl(velocityRequest.withVelocity(rps));
        rightFlywheelMotor.setControl(velocityRequest.withVelocity(-rps));
    }

    public void requestAngle(Rotation2d angle) {
        hoodMotor.setControl(positionRequest.withPosition(angleToPosition(angle)));
    }
    
    public Command setSubstateCmd(Substate substate) {
        return Commands.runOnce(() -> this.substate = substate);
    }

    @Override
    public void periodic() {
        switch (substate) {
            case IDLE:
                requestAngle(Rotation2d.fromDegrees(20));
                requestRPS(0); // TODO: Convert from raw voltage
                break;

            case REV:
                ShotParameters targetParams = ShotCalculator.getTargetParams();

                requestAngle(targetParams.hoodAngle());
                requestRPS(targetParams.rps());
                break;

            case UNJAM:
                requestAngle(Rotation2d.fromDegrees(20));
                requestRPS(0); // TODO: Convert from raw voltage
                break;
        
            default:
                break;
        }
    }
}