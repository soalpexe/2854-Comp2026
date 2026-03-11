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

    private boolean isSlowed, isAiming;

    private PIDController aimingPID;

    public Drivetrain(SwerveDrivetrainConstants drivetrainConfig, double odomFrequency, SwerveModuleConstants<?, ?, ?>... moduleConfigs) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConfig, odomFrequency, moduleConfigs);

        focRequest = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drivetrain.maxSpeed * Constants.deadband)
            .withRotationalDeadband(Constants.Drivetrain.maxAngularSpeed * Constants.deadband);

        rocRequest = new SwerveRequest.RobotCentric();

        isSlowed = false;
        isAiming = false;

        aimingPID = new PIDController(Constants.Drivetrain.aimingP, Constants.Drivetrain.aimingI, Constants.Drivetrain.aimingD);
        aimingPID.enableContinuousInput(-Math.PI, Math.PI);

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

    public Pose2d getEstimatedPose() {
        return getState().Pose;
    }

    public Rotation2d getHeading() {
        return getState().RawHeading;
    }

    public ChassisSpeeds getSpeeds() {
        return getState().Speeds;
    }

    public boolean isSlowed() {
        return isSlowed;
    }

    public boolean isAiming() {
        return isAiming;
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
        double percent = isSlowed ? Constants.Drivetrain.slowPercent : 1;

        rocRequest
            .withVelocityX(speeds.vxMetersPerSecond * percent)
            .withVelocityY(speeds.vyMetersPerSecond * percent)
            .withRotationalRate(speeds.omegaRadiansPerSecond * percent);

        focRequest
            .withVelocityX(speeds.vxMetersPerSecond * percent)
            .withVelocityY(speeds.vyMetersPerSecond * percent)
            .withRotationalRate(speeds.omegaRadiansPerSecond * percent);
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

    public Command setSlowedCmd(boolean value) {
        return Commands.runOnce(() -> isSlowed = value);
    }

    public Command setAimingCmd(boolean value) {
        return Commands.runOnce(() -> {
            isAiming = value;
            if (isAiming) aimingPID.reset();
        });
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setOperatorPerspectiveForward(
                Utilities.getAlliance() == Alliance.Red ? Constants.redPerspective : Constants.bluePerspective
            );
        }

        if (isAiming) {
            double omega = aimingPID.calculate(getHeading().getRadians(), ShotCalculator.getTargetHeading());
            omega = MathUtil.clamp(omega, -Constants.Drivetrain.maxAngularSpeed, Constants.Drivetrain.maxAngularSpeed);

            rocRequest.withRotationalRate(omega);
            focRequest.withRotationalRate(omega);
        }

        if (DriverStation.isAutonomous()) setControl(rocRequest);
        else setControl(focRequest);
    }
}
