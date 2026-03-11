// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.StateMachine.State;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private StateMachine fsm;

    private CommandScheduler scheduler;
    private CommandXboxController controller;

    private Drivetrain drivetrain;
    private Vision vision;

    private Shooter shooter;
    private Transfer transfer;
    private Spindexer spindexer;
    private Intake intake;
    private Climber climber;

    public RobotContainer() {
        fsm = new StateMachine();

        scheduler = CommandScheduler.getInstance();
        controller = new CommandXboxController(Constants.controllerID);

        drivetrain = new Drivetrain(
            Constants.Drivetrain.drivetrainConfig,
            Constants.odomFrequency,
            Constants.Drivetrain.frontLeftConfig, Constants.Drivetrain.frontRightConfig, Constants.Drivetrain.backLeftConfig, Constants.Drivetrain.backRightConfig
        );
        vision = new Vision();

        shooter = new Shooter(Constants.Shooter.leftMotorID, Constants.Shooter.rightMotorID);
        transfer = new Transfer(Constants.Transfer.motorID);
        spindexer = new Spindexer(Constants.Spindexer.motorID);
        intake = new Intake(Constants.Intake.pivotMotorID, Constants.Intake.rollerMotorID);
        climber = new Climber(Constants.Climber.motorID);

        ShotCalculator.configure(
            drivetrain::getEstimatedPose,
            drivetrain::getSpeeds
        );

        configureFSMTriggers();
        configureBindings();
        configureNamedCmds();
    }

    private void configureFSMTriggers() {
        fsm.getStateTrigger(State.UNJAM)
            .onTrue(
                Commands.sequence(
                    intake.setPercentCmd(-1),
                    transfer.setPercentCmd(1),
                    spindexer.setPercentCmd(1)
                )
            )
            .onFalse(
                Commands.sequence(
                    intake.setPercentCmd(0),
                    transfer.setPercentCmd(0),
                    spindexer.setPercentCmd(0)
                )
            );

        fsm.getStateTrigger(State.INTAKE)
            .onTrue(
                Commands.sequence(
                    intake.setPositionCmd(Intake.Position.DEPLOY),
                    intake.setPercentCmd(1)
                )
            )
            .onFalse(intake.setPercentCmd(0));

        fsm.getStateTrigger(State.SHOOT)
            .onTrue(
                Commands.parallel(
                    drivetrain.setAimingCmd(true),
                    intake.pulseCmd(),

                    Commands.sequence(
                        transfer.setPercentCmd(1),
                        spindexer.setPercentCmd(1),

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
                    shooter.setPercentCmd(0)
                )
            );
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.requestSpeedsCmd(
                () -> controller.getLeftY() * Constants.Drivetrain.maxSpeed,
                () -> controller.getLeftX() * Constants.Drivetrain.maxSpeed,
                () -> -controller.getRightX() * Constants.Drivetrain.maxAngularSpeed
            )
        );

        controller.x().onTrue(Commands.runOnce(drivetrain::seedFieldCentric));
        controller.a().onTrue(intake.togglePositionCmd());

        controller.b()
            .onTrue(fsm.setStateCmd(State.UNJAM))
            .onFalse(fsm.setStateCmd(State.IDLE));

        controller.leftBumper()
            .onTrue(fsm.setStateCmd(State.INTAKE))
            .onFalse(fsm.setStateCmd(State.IDLE));
            
        controller.rightBumper()
            .onTrue(fsm.setStateCmd(State.SHOOT))
            .onFalse(fsm.setStateCmd(State.IDLE));

        controller.rightTrigger()
            .onTrue(drivetrain.setSlowedCmd(true))
            .onFalse(drivetrain.setSlowedCmd(false));
    }

    private void configureNamedCmds() {
        NamedCommands.registerCommand("DeployIntake", intake.setPositionCmd(Intake.Position.DEPLOY));

        NamedCommands.registerCommand("SetStateIdle", fsm.setStateCmd(State.IDLE));
        NamedCommands.registerCommand("SetStateUnjam", fsm.setStateCmd(State.UNJAM));
        NamedCommands.registerCommand("SetStateIntake", fsm.setStateCmd(State.INTAKE));
        NamedCommands.registerCommand("SetStateShoot", fsm.setStateCmd(State.SHOOT));
    }

    public void reset() {
        scheduler.cancelAll();
        scheduler.schedule(fsm.setStateCmd(State.IDLE));
    }

    public void periodic() {
        drivetrain.addVisionMeasurements(vision.getPoseEstimates());

        Logger.recordOutput("Estimated Robot Pose", drivetrain.getEstimatedPose());

        Logger.recordOutput("Is Slowed", drivetrain.isSlowed());
        Logger.recordOutput("Is Aiming", drivetrain.isAiming());
        
        Logger.recordOutput("Last Robot State", fsm.getLastState().toString());
        Logger.recordOutput("Current Robot State", fsm.getCurrentState().toString());
    }
}
