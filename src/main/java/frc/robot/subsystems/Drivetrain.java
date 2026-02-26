// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.ShotCalculator;
import frc.robot.Utilities;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    private SwerveRequest.FieldCentric focRequest;
    private SwerveRequest.RobotCentric rocRequest;

    private PIDController translationPID, headingPID;

    private boolean isAiming;

    public Drivetrain(SwerveDrivetrainConstants drivetrainConfig, double odomFrequency, SwerveModuleConstants<?, ?, ?>... moduleConfigs) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConfig, odomFrequency, moduleConfigs);

        focRequest = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drivetrain.maxSpeed * Constants.deadband)
            .withRotationalDeadband(Constants.Drivetrain.maxAngularSpeed * Constants.deadband);

        rocRequest = new SwerveRequest.RobotCentric();

        translationPID = new PIDController(Constants.Drivetrain.translationP, Constants.Drivetrain.translationI, Constants.Drivetrain.translationD);
        headingPID = new PIDController(Constants.Drivetrain.headingP, Constants.Drivetrain.headingI, Constants.Drivetrain.headingD);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    private double calcAimingPID(double targetHeading) {
        double power = headingPID.calculate(getHeading().getRadians(), targetHeading);
        return MathUtil.clamp(power, -Constants.Drivetrain.maxAngularSpeed, Constants.Drivetrain.maxAngularSpeed);
    }
    

    public Pose2d getEstimatedPose() {
        return getState().Pose;
    }

    public Rotation2d getHeading() {
        return getState().RawHeading;
    }

    public ChassisSpeeds getSpeeds() {
        return getState().Speeds;
    }

    public void addVisionMeasurements(Pose2d... rawEstimates) {
        for (Pose2d rawEstimate : rawEstimates) {
            if (Utilities.isValidPose(rawEstimate)) {
                Pose2d estimate = new Pose2d(rawEstimate.getTranslation(), getHeading());
                addVisionMeasurement(estimate, Utils.getCurrentTimeSeconds());
            }
        }
    }

    public void setROCSpeeds(ChassisSpeeds speeds) {
        setControl(rocRequest
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(isAiming ? calcAimingPID(ShotCalculator.targetHeading) : speeds.omegaRadiansPerSecond)
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
        return Commands.runOnce(() -> isAiming = value);
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
