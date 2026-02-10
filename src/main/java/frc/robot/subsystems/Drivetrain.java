// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.ShotCalculator;
import frc.robot.Utilities;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    private SwerveRequest.FieldCentric focRequest;
    private SwerveRequest.RobotCentric rocRequest;

    private PIDController translationPID, headingPID;

    private boolean isAiming;

    public Drivetrain(SwerveDrivetrainConstants drivetrainConfig, SwerveModuleConstants<?, ?, ?>... moduleConfigs) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConfig, moduleConfigs);

        focRequest = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drivetrain.maxSpeed * Constants.deadband)
            .withRotationalDeadband(Constants.Drivetrain.maxAngularSpeed * Constants.deadband);

        rocRequest = new SwerveRequest.RobotCentric();

        translationPID = new PIDController(Constants.Drivetrain.translationP, Constants.Drivetrain.translationI, Constants.Drivetrain.translationD);
        headingPID = new PIDController(Constants.Drivetrain.headingP, Constants.Drivetrain.headingI, Constants.Drivetrain.headingD);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);

        setOdometryFrequency(250);
    }

    private void setOdometryFrequency(double frequency) {
        StatusSignalCollection signals = new StatusSignalCollection();

        for (var module : getModules()) {
            signals.addSignals(
                module.getDriveMotor().getPosition(),
                module.getEncoder().getAbsolutePosition()
            );
        }

        signals.addSignals(getPigeon2().getYaw());
        signals.setUpdateFrequencyForAll(frequency);
    }

    private double calcAimingPID(double targetHeading) {
        double power = headingPID.calculate(getRotation2d().getRadians(), targetHeading);
        return MathUtil.clamp(power, -Constants.Drivetrain.maxAngularSpeed, Constants.Drivetrain.maxAngularSpeed);
    }

    public Rotation2d getRotation2d() {
        return getState().RawHeading;
    }

    public Pose2d getPose2d() {
        return getState().Pose;
    }

    public ChassisSpeeds getSpeeds() {
        return getState().Speeds;
    }

    public void addVisionMeasurement(Pose2d rawEstimate) {
        if (Utilities.isValidPose(rawEstimate)) {
            Pose2d estimate = new Pose2d(rawEstimate.getTranslation(), getRotation2d());
            addVisionMeasurement(estimate, Utils.getCurrentTimeSeconds());
        }
    }

    public void addVisionMeasurements(Pose2d[] estimates) {
        for (Pose2d estimate : estimates) {
            addVisionMeasurement(estimate);
        }
    }

    public void setROCSpeeds(ChassisSpeeds speeds) {
        setControl(rocRequest
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vxMetersPerSecond)
            .withRotationalRate(isAiming ? calcAimingPID(ShotCalculator.targetHeading) : speeds.vxMetersPerSecond)
        );
    }

    public Command setFOCSpeedsCmd(DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier angularSpeed) {
        return run(() -> setControl(focRequest
                .withVelocityX(speedX.getAsDouble())
                .withVelocityY(speedY.getAsDouble())
                .withRotationalRate(isAiming ? calcAimingPID(ShotCalculator.targetHeading) : angularSpeed.getAsDouble())
            )
        );
    }

    public Command setIsAimingCmd(boolean value) {
        return runOnce(() -> isAiming = value);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setOperatorPerspectiveForward(
                Utilities.getAlliance() == Alliance.Red ? Constants.redPerspective : Constants.bluePerspective
            );
        }
    }
}
