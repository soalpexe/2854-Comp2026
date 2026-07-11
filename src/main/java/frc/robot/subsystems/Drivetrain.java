// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.ShotCalculator;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    public enum Substate {
        DRIVE,
        SNAKE,
        AIM,
        TRENCH_AND_BUMP
    }

    private PIDController headingPID;
    
    private Substate substate;
    private SwerveRequest.FieldCentric fcRequest;
    private SwerveRequest.RobotCentric rcRequest;

    public Drivetrain(SwerveDrivetrainConstants drivetrainConfig, double odomFrequency, SwerveModuleConstants<?, ?, ?>... moduleConfigs) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConfig, odomFrequency, moduleConfigs);

        headingPID = new PIDController(Constants.Drivetrain.headingP, Constants.Drivetrain.headingI, Constants.Drivetrain.headingD);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);

        substate = Substate.DRIVE;

        fcRequest = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drivetrain.maxSpeed * Constants.deadband)
            .withRotationalDeadband(Constants.Drivetrain.maxAngularSpeed * Constants.deadband);
        rcRequest = new SwerveRequest.RobotCentric();

        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            AutoBuilder.configure(
                this::getEstimatedPose,
                this::resetPose,
                this::getSpeeds,
                this::requestSpeeds,
                new PPHolonomicDriveController(
                    new PIDConstants(5, 0, 0),
                    new PIDConstants(5, 0, 0)
                ),
                RobotConfig.fromGUISettings(),
                () -> false,
                this
            );
        }

        catch (IOException | ParseException e) {
            e.printStackTrace();
        }
    }

    public Substate getSubstate() {
        return substate;
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

    public Translation2d getDisplacement() {
        return getEstimatedPose().getTranslation();
    }

    public Translation2d getVelocity() {
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getSpeeds(), getHeading());
        return new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    }

    public void addVisionMeasurements(ArrayList<PoseEstimate> estimates) {
        for (PoseEstimate estimate : estimates) {
            Pose2d pose = new Pose2d(estimate.pose.getTranslation(), getHeading());
            addVisionMeasurement(pose, estimate.timestampSeconds);
        }
    }

    public void requestSpeeds(ChassisSpeeds speeds) {
        fcRequest
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond);

        rcRequest
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond);
    }

    public void requestHeadingPID(Rotation2d targetHeading) {
        double omega = headingPID.calculate(getHeading().getRadians(), targetHeading.getRadians());
        omega = MathUtil.clamp(omega, -Constants.Drivetrain.maxAngularSpeed, Constants.Drivetrain.maxAngularSpeed);

        fcRequest.withRotationalRate(omega);
        rcRequest.withRotationalRate(omega);
    }

    public Command requestSpeedsCmd(DoubleSupplier vxSupplier, DoubleSupplier vySupplier, DoubleSupplier omegaSupplier) {
        return run(() -> requestSpeeds(
            new ChassisSpeeds(
                vxSupplier.getAsDouble(),
                vySupplier.getAsDouble(),
                omegaSupplier.getAsDouble()
            )
        ));
    }

    public Command setSubstateCmd(Substate substate) {
        return Commands.runOnce(() -> this.substate = substate);
    }

    @Override
    public void periodic() {
        Rotation2d targetHeading;

        switch (substate) {
            case SNAKE:
                if (getVelocity().getNorm() > Constants.Drivetrain.minSnakeVel) requestHeadingPID(getVelocity().getAngle());
                break;

            case AIM:
                targetHeading = ShotCalculator.getTargetParams().heading();
                requestHeadingPID(targetHeading.plus(Constants.Shooter.rotationOffset));
                break;

            case TRENCH_AND_BUMP:
                Pose2d pose = getEstimatedPose();
                targetHeading = (pose.getY() < Constants.Field.bumpLeftY && pose.getY() > Constants.Field.bumpRightY) ? new Rotation2d(45) : Rotation2d.kZero;
                requestHeadingPID(targetHeading);
                break;
        
            default:
                break;
        }

        if (DriverStation.isAutonomous()) setControl(rcRequest);
        else setControl(fcRequest);
    }
}