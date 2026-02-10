// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private CommandScheduler scheduler;
    private CommandXboxController controller;

    private StructPublisher<Pose2d> robotPosePublisher;

    public Drivetrain drivetrain;
    public Intake intake;
    public Vision vision;

    public RobotContainer() {
        scheduler = CommandScheduler.getInstance();
        controller = new CommandXboxController(Constants.controllerID);

        robotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();

        drivetrain = new Drivetrain(
            Constants.Drivetrain.drivetrainConfig,
            Constants.Drivetrain.frontLeftConfig, Constants.Drivetrain.frontRightConfig,
            Constants.Drivetrain.backLeftConfig, Constants.Drivetrain.backRightConfig
        );

        intake = new Intake(0, 0);
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
        
        controller.rightBumper()
            .onTrue(drivetrain.setIsAimingCmd(true))
            .onFalse(drivetrain.setIsAimingCmd(false));
    }

    public void periodic() {
        drivetrain.addVisionMeasurements(vision.getPoseEstimates());
        ShotCalculator.updateState(drivetrain.getPose2d(), drivetrain.getSpeeds());

        robotPosePublisher.set(drivetrain.getPose2d());
    }
}
