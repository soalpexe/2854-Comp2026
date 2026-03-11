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

    private PIDController aimingPID;

    private boolean isSlowed, isAiming;

    public Drivetrain(SwerveDrivetrainConstants drivetrainConfig, double odomFrequency, SwerveModuleConstants<?, ?, ?>... moduleConfigs) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConfig, odomFrequency, moduleConfigs);

        focRequest = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drivetrain.maxSpeed * Constants.deadband)
            .withRotationalDeadband(Constants.Drivetrain.maxAngularSpeed * Constants.deadband);

        rocRequest = new SwerveRequest.RobotCentric();

        aimingPID = new PIDController(Constants.Drivetrain.aimingP, Constants.Drivetrain.aimingI, Constants.Drivetrain.aimingD);
        aimingPID.enableContinuousInput(-Math.PI, Math.PI);

        isSlowed = false;
        isAiming = false;

        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            AutoBuilder.configure(
                this::getEstimatedPose,
                this::resetPose,
                this::getSpeeds,
                this::setROCSpeeds,
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

    private double calcAimingPID(double targetHeading) {
        double power = aimingPID.calculate(getHeading().getRadians(), targetHeading);
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
        double percent = isSlowed ? Constants.Drivetrain.slowPercent : 1;

        setControl(rocRequest
            .withVelocityX(speeds.vxMetersPerSecond * percent)
            .withVelocityY(speeds.vyMetersPerSecond * percent)
        );

        if (!isAiming) setControl(focRequest.withRotationalRate(speeds.omegaRadiansPerSecond * percent));
    }

    public Command setFOCSpeedsCmd(DoubleSupplier vxSupplier, DoubleSupplier vySupplier, DoubleSupplier omegaSupplier) {
        return run(() -> {
            double percent = isSlowed ? Constants.Drivetrain.slowPercent : 1;

            setControl(focRequest
                .withVelocityX(vxSupplier.getAsDouble() * percent)
                .withVelocityY(vySupplier.getAsDouble() * percent)
            );

            if (!isAiming) setControl(focRequest.withRotationalRate(omegaSupplier.getAsDouble() * percent));
        });
    }

    public Command setSlowedCmd(boolean value) {
        return Commands.runOnce(() -> isSlowed = value);
    }

    public Command setAimingCmd(boolean value) {
        return Commands.runOnce(() -> isAiming = value);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setOperatorPerspectiveForward(
                Utilities.getAlliance() == Alliance.Red ? Constants.redPerspective : Constants.bluePerspective
            );
        }

        if (isAiming) setControl(focRequest.withRotationalRate(calcAimingPID(ShotCalculator.getTargetHeading())));
    }
}
