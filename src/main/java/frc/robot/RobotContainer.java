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
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private CommandXboxController controller;

    public Drivetrain drivetrain;
    public Intake intake;
    public Vision vision;

    public RobotContainer() {
        controller = new CommandXboxController(Constants.controllerID);

        drivetrain = new Drivetrain(
            Constants.Drivetrain.drivetrainConfig,
            Constants.Drivetrain.frontLeftConfig, Constants.Drivetrain.frontRightConfig,
            Constants.Drivetrain.backLeftConfig, Constants.Drivetrain.backRightConfig
        );

        intake = new Intake(Constants.Intake.pivotMotorID, Constants.Intake.rollerMotorID);
        vision = new Vision();

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
        
        controller.leftBumper()
            .onTrue(intake.startCmd())
            .onFalse(intake.stopCmd());

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
        return Commands.sequence(
            drivetrain.setIsAimingCmd(true),
            intake.pulseCmd()
        );
    }
    
    public Command stopOuttakeCmd() {
        return Commands.sequence(
            drivetrain.setIsAimingCmd(false),
            intake.setPositionCmd(Intake.Position.Deploy),
            intake.stopCmd()
        );
    }
}
