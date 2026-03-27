// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.Utils;
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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    public enum Substate {
        DRIVE,
        SNAKE,
        AIM
    }

    private PIDController headingPID;

    private Translation2d rlast, r;
    
    private Substate substate;
    private SwerveRequest.FieldCentric fcRequest;
    private SwerveRequest.RobotCentric rcRequest;

    public Drivetrain(SwerveDrivetrainConstants drivetrainConfig, double odomFrequency, SwerveModuleConstants<?, ?, ?>... moduleConfigs) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConfig, odomFrequency, moduleConfigs);

        headingPID = new PIDController(Constants.Drivetrain.headingP, Constants.Drivetrain.headingI, Constants.Drivetrain.headingD);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);

        rlast = new Translation2d();
        r = new Translation2d();

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
                () -> Utilities.getAlliance() == Alliance.Red,
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
        Translation2d dr = r.minus(rlast);
        return dr.div(Constants.period);
    }

    public void addVisionMeasurements(Pose2d... rawEstimates) {
        for (Pose2d rawEstimate : rawEstimates) {
            if (Utilities.isValidPose(rawEstimate)) {
                Pose2d estimate = new Pose2d(rawEstimate.getTranslation(), getHeading());
                addVisionMeasurement(estimate, Utils.getCurrentTimeSeconds());
            }
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
        rlast = r;
        r = getDisplacement();

        switch (substate) {
            case SNAKE:
                if (getVelocity().getNorm() > 1) requestHeadingPID(getVelocity().getAngle());
                break;

            case AIM:
                Rotation2d targetHeading = ShotCalculator.getTargetParams().heading();
                requestHeadingPID(targetHeading.plus(Constants.Shooter.rotationOffset));
                break;
        
            default:
                break;
        }

        if (DriverStation.isAutonomous()) setControl(rcRequest);
        else setControl(fcRequest);
    }
}