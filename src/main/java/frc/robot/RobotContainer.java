// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private CommandXboxController controller;

    public Drivetrain drivetrain;
    public Vision vision;

    public Shooter shooter;
    public Transfer transfer;
    public Spindexer spindexer;
    public Intake intake;

    public RobotContainer() {
        controller = new CommandXboxController(Constants.controllerID);

        drivetrain = new Drivetrain(
            Constants.Drivetrain.drivetrainConfig,
            Constants.Drivetrain.odomFrequency,
            Constants.Drivetrain.frontLeftConfig, Constants.Drivetrain.frontRightConfig, Constants.Drivetrain.backLeftConfig, Constants.Drivetrain.backRightConfig
        );
        vision = new Vision();

        shooter = new Shooter(Constants.Shooter.leftMotorID, Constants.Shooter.rightMotorID);
        transfer = new Transfer(Constants.Transfer.motorID);
        spindexer = new Spindexer(Constants.Spindexer.motorID);
        intake = new Intake(Constants.Intake.pivotMotorID, Constants.Intake.rollerMotorID);

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.setFOCSpeedsCmd(
                () -> -controller.getLeftY() * Constants.Drivetrain.maxSpeed,
                () -> -controller.getLeftX() * Constants.Drivetrain.maxSpeed,
                () -> -controller.getRightX() * Constants.Drivetrain.maxAngularSpeed
            )
        );
        
        controller.a().onChange(intake.toggleDeployCmd());
        controller.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        controller.leftBumper().whileTrue(intake.startCmd());
        controller.rightBumper()
            .onTrue(startOuttakeCmd())
            .onFalse(stopOuttakeCmd());
    }

    public void periodic() {
        drivetrain.addVisionMeasurements(vision.getPoseEstimates());
        ShotCalculator.updateState(drivetrain.getEstimatedPose(), drivetrain.getSpeeds());

        Logger.recordOutput("Estimated Robot Pose", drivetrain.getEstimatedPose());
    }

    public Command startOuttakeCmd() {
        return Commands.parallel(
            drivetrain.setIsAimingCmd(true),
            intake.pulseCmd(),
            Commands.sequence(
                shooter.setPercentCmd(0.3),
                Commands.waitSeconds(0.3),
                transfer.setPercentCmd(-1),
                spindexer.setPercentCmd(-0.8)
            )
        );
    }
    
    public Command stopOuttakeCmd() {
        return Commands.parallel(
            drivetrain.setIsAimingCmd(false),
            intake.setPositionCmd(Intake.Position.Deploy),
            Commands.sequence(
                spindexer.setPercentCmd(0),
                Commands.waitSeconds(0.3),
                transfer.setPercentCmd(0),
                shooter.setPercentCmd(0)
            )
        );
    }
}
