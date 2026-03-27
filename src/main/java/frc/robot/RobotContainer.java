// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

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
            Constants.odomFrequency,
            Constants.Drivetrain.frontLeftConfig, Constants.Drivetrain.frontRightConfig, Constants.Drivetrain.backLeftConfig, Constants.Drivetrain.backRightConfig
        );
        vision = new Vision();

        shooter = new Shooter(Constants.Shooter.leftFlywheelMotorID, Constants.Shooter.rightFlywheelMotorID, Constants.Shooter.hoodMotorID);
        transfer = new Transfer(Constants.Transfer.motorID);
        spindexer = new Spindexer(Constants.Spindexer.motorID);
        intake = new Intake(Constants.Intake.pivotMotorID, Constants.Intake.topRollerMotorID, Constants.Intake.bottomRollerMotorID);

        ShotCalculator.configure(
            drivetrain::getEstimatedPose,
            drivetrain::getVelocity
        );

        configureFSM();
        configureBindings();
    }

    private void configureFSM() {
        fsm.configureState(
            State.IDLE,
            () -> Commands.sequence(
                drivetrain.setSubstateCmd(Drivetrain.Substate.DRIVE),
                shooter.setSubstateCmd(Shooter.Substate.IDLE),
                transfer.setSubstateCmd(Transfer.Substate.OFF),
                spindexer.setSubstateCmd(Spindexer.Substate.OFF),
                intake.setSubstateCmd(Intake.Substate.OFF)
            ),
            Commands::none,
            Commands::none
        );
        
        fsm.configureState(
            State.STOW,
            () -> intake.setSubstateCmd(Intake.Substate.STOW),
            Commands::none,
            () -> intake.setSubstateCmd(Intake.Substate.OFF)
        );
        
        fsm.configureState(
            State.INTAKE,
            () -> intake.setSubstateCmd(Intake.Substate.ON),
            Commands::none,
            () -> intake.setSubstateCmd(Intake.Substate.OFF)
        );

        fsm.configureState(
            State.INTAKE_AND_SNAKE,
            () -> Commands.sequence(
                drivetrain.setSubstateCmd(Drivetrain.Substate.SNAKE),
                intake.setSubstateCmd(Intake.Substate.ON)
            ),
            Commands::none,
            () -> Commands.sequence(
                drivetrain.setSubstateCmd(Drivetrain.Substate.DRIVE),
                intake.setSubstateCmd(Intake.Substate.OFF)
            )
        );
        
        fsm.configureState(
            State.SHOOT,
            () -> Commands.sequence(
                drivetrain.setSubstateCmd(Drivetrain.Substate.AIM),
                intake.setSubstateCmd(Intake.Substate.RAMP),

                shooter.setSubstateCmd(Shooter.Substate.REV),
                transfer.setSubstateCmd(Transfer.Substate.UNJAM),
                spindexer.setSubstateCmd(Spindexer.Substate.UNJAM),
                Commands.waitSeconds(0.3),

                transfer.setSubstateCmd(Transfer.Substate.ON),
                spindexer.setSubstateCmd(Spindexer.Substate.ON)
            ),
            Commands::none,
            () -> Commands.sequence(
                drivetrain.setSubstateCmd(Drivetrain.Substate.DRIVE),
                intake.setSubstateCmd(Intake.Substate.OFF),

                spindexer.setSubstateCmd(Spindexer.Substate.OFF),
                Commands.waitSeconds(0.3),

                shooter.setSubstateCmd(Shooter.Substate.IDLE),
                transfer.setSubstateCmd(Transfer.Substate.OFF)
            )
        );

        fsm.configureState(
            State.SHOOT_AND_INTAKE,
            () -> Commands.sequence(
                drivetrain.setSubstateCmd(Drivetrain.Substate.AIM),
                intake.setSubstateCmd(Intake.Substate.ON),

                shooter.setSubstateCmd(Shooter.Substate.REV),
                transfer.setSubstateCmd(Transfer.Substate.UNJAM),
                spindexer.setSubstateCmd(Spindexer.Substate.UNJAM),
                Commands.waitSeconds(0.3),

                transfer.setSubstateCmd(Transfer.Substate.ON),
                spindexer.setSubstateCmd(Spindexer.Substate.ON)
            ),
            Commands::none,
            () -> Commands.sequence(
                drivetrain.setSubstateCmd(Drivetrain.Substate.DRIVE),
                intake.setSubstateCmd(Intake.Substate.OFF),

                spindexer.setSubstateCmd(Spindexer.Substate.OFF),
                Commands.waitSeconds(0.3),

                shooter.setSubstateCmd(Shooter.Substate.IDLE),
                transfer.setSubstateCmd(Transfer.Substate.OFF)
            )
        );

        fsm.addTransition(State.SHOOT, State.SHOOT_AND_INTAKE, () -> intake.setSubstateCmd(Intake.Substate.ON));
        fsm.addTransition(State.SHOOT_AND_INTAKE, State.SHOOT, () -> intake.setSubstateCmd(Intake.Substate.RAMP));

        fsm.configureState(
            State.UNJAM,
            () -> Commands.sequence(
                shooter.setSubstateCmd(Shooter.Substate.UNJAM),
                transfer.setSubstateCmd(Transfer.Substate.UNJAM),
                spindexer.setSubstateCmd(Spindexer.Substate.UNJAM),
                intake.setSubstateCmd(Intake.Substate.UNJAM)
            ),
            Commands::none,
            () -> Commands.sequence(
                shooter.setSubstateCmd(Shooter.Substate.IDLE),
                transfer.setSubstateCmd(Transfer.Substate.OFF),
                spindexer.setSubstateCmd(Spindexer.Substate.OFF),
                intake.setSubstateCmd(Intake.Substate.OFF)
            )
        );
        
        fsm.initialize(State.STOW);
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.requestSpeedsCmd(
                () -> -controller.getLeftY() * Constants.Drivetrain.maxSpeed,
                () -> -controller.getLeftX() * Constants.Drivetrain.maxSpeed,
                () -> -controller.getRightX() * Constants.Drivetrain.maxAngularSpeed
            )
        );

        controller.x().onTrue(Commands.runOnce(drivetrain::seedFieldCentric));

        controller.a()
            .onTrue(
                Commands.either(
                    fsm.requestStateCmd(State.STOW),
                    fsm.requestStateCmd(State.IDLE),
                    () -> fsm.getState() != State.STOW
                )
            );

        controller.b()
            .onTrue(fsm.requestStateCmd(State.UNJAM))
            .onFalse(fsm.requestStateCmd(State.IDLE));

        controller.leftBumper().or(controller.rightBumper())
            .whileTrue(
                Commands.defer(
                    () -> {
                        boolean lbPressed = controller.leftBumper().getAsBoolean(), rbPressed = controller.rightBumper().getAsBoolean();

                        if (lbPressed && !rbPressed) return fsm.requestStateCmd(State.INTAKE);
                        if (!lbPressed && rbPressed) return fsm.requestStateCmd(State.SHOOT);
                        return fsm.requestStateCmd(State.SHOOT_AND_INTAKE);
                    },
                    Set.of()
                ).repeatedly()
            )
            .onFalse(fsm.requestStateCmd(State.IDLE));

        controller.rightTrigger()
            .onTrue(fsm.requestStateCmd(State.INTAKE_AND_SNAKE))
            .onFalse(fsm.requestStateCmd(State.IDLE));
    }

    public void periodic() {
        drivetrain.addVisionMeasurements(vision.getPoseEstimates());

        Logger.recordOutput("Robot Pose", drivetrain.getEstimatedPose());
        Logger.recordOutput("Target Pose", ShotCalculator.getTargetPose());
        Logger.recordOutput("Robot State", fsm.getState().toString());
    }
}