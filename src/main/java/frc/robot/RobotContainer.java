// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.StateMachine.State;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private StateMachine fsm;
    private CommandXboxController controller;

    private Drivetrain drivetrain;
    private Vision vision;

    private Shooter shooter;
    private Transfer transfer;
    private Spindexer spindexer;
    private Intake intake;

    public RobotContainer() {
        fsm = new StateMachine();
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

        configureFSMTriggers();
        configureBindings();
    }

    private void configureFSMTriggers() {
        fsm.getStateTrigger(State.INTAKE)
            .onTrue(
                Commands.sequence(
                    intake.setPositionCmd(Intake.Position.DEPLOY),
                    intake.setPercentCmd(1)
                )
            )
            .onFalse(intake.setPercentCmd(0));

        fsm.getStateTrigger(State.OUTTAKE)
            .onTrue(
                Commands.parallel(
                    drivetrain.setAimingCmd(true),
                    Commands.repeatingSequence(
                        intake.setPercentCmd(1),
                        intake.setPositionCmd(Intake.Position.DEPLOY),
                        intake.setPositionCmd(Intake.Position.STOW),
                        Commands.waitSeconds(0.3)
                    ),

                    Commands.sequence(
                        shooter.setPercentCmd(0.5),
                        Commands.waitSeconds(0.3),
                        transfer.setPercentCmd(-1),
                        spindexer.setPercentCmd(-1)
                    )
                )
            )
            .onFalse(
                Commands.sequence(
                    drivetrain.setAimingCmd(false),
                    intake.setPositionCmd(Intake.Position.DEPLOY),
                    intake.setPercentCmd(0),

                    spindexer.setPercentCmd(0),
                    Commands.waitSeconds(0.3),
                    transfer.setPercentCmd(0),
                    shooter.setPercentCmd(0.3)
                )
            );
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.setFOCSpeedsCmd(
                () -> -controller.getLeftY() * Constants.Drivetrain.maxSpeed,
                () -> -controller.getLeftX() * Constants.Drivetrain.maxSpeed,
                () -> -controller.getRightX() * Constants.Drivetrain.maxAngularSpeed
            )
        );

        controller.leftBumper()
            .onTrue(fsm.setStateCmd(State.INTAKE))
            .onFalse(fsm.setStateCmd(State.IDLE));
            
        controller.rightBumper()
            .onTrue(fsm.setStateCmd(State.OUTTAKE))
            .onFalse(fsm.setStateCmd(State.IDLE));
    }

    public void periodic() {
        drivetrain.addVisionMeasurements(vision.getPoseEstimates());
        ShotCalculator.updateState(drivetrain.getEstimatedPose(), drivetrain.getSpeeds());

        Logger.recordOutput("Estimated Robot Pose", drivetrain.getEstimatedPose());
        Logger.recordOutput("Last Robot State", fsm.lastState.toString());
        Logger.recordOutput("Current Robot State", fsm.currentState.toString());
    }
}
