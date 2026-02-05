// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private CommandScheduler scheduler;
    private CommandXboxController controller;

    public Drivetrain drivetrain;
    public Vision vision;

    public RobotContainer() {
        scheduler = CommandScheduler.getInstance();
        controller = new CommandXboxController(ControllerConstants.controllerID);

        drivetrain = new Drivetrain(
            DrivetrainConstants.drivetrainConfig,
            DrivetrainConstants.frontLeftConfig, DrivetrainConstants.frontRightConfig,
            DrivetrainConstants.backLeftConfig, DrivetrainConstants.backRightConfig
        );

        vision = new Vision();

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.setFOCSpeedsCmd(
                () -> controller.getLeftX(),
                () -> controller.getLeftY(),
                () -> controller.getRightX()
            )
        );
        
        controller.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        controller.rightBumper()
            .onTrue(drivetrain.setIsAimingCmd(true))
            .onFalse(drivetrain.setIsAimingCmd(false));
    }

    public void periodic() {
        drivetrain.addVisionMeasurements(vision.getPoseEstimates());
        ShotCalculator.updateState(drivetrain.getPose2d(), drivetrain.getSpeeds());
    }
}
