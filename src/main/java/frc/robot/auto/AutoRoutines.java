// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.StateMachine;
import frc.robot.StateMachine.State;
import frc.robot.subsystems.Drivetrain;

public class AutoRoutines {
    private static final double shootPreloadSec = 3.0;
    private static final double shootCycleSec = 3.5;

    private static final double leaveSpeedMps = 1.5;
    private static final double leaveTimeSec = 2.0;

    private final StateMachine fsm;
    private final Drivetrain drivetrain;

    public AutoRoutines(StateMachine fsm, Drivetrain drivetrain) {
        this.fsm = fsm;
        this.drivetrain = drivetrain;

        configureNamedCommands();
    }

    // Building blocks for autos authored in the PathPlanner GUI
    private void configureNamedCommands() {
        NamedCommands.registerCommand("Shoot", autoShoot(shootCycleSec));
        NamedCommands.registerCommand("Intake", startIntaking());
        NamedCommands.registerCommand("Idle", fsm.requestStateCmd(State.IDLE));
    }

    private Command autoShoot(double seconds) {
        return Commands.sequence(
            drivetrain.stopCmd(),
            fsm.requestStateCmd(State.SHOOT),
            fsm.waitForState(State.SHOOT),
            Commands.waitSeconds(seconds)
        );
    }

    private Command startIntaking() {
        return Commands.sequence(
            fsm.requestStateCmd(State.INTAKE),
            fsm.waitForState(State.INTAKE)
        );
    }

    private Command timedLeave() {
        return drivetrain.requestSpeedsCmd(() -> -leaveSpeedMps, () -> 0.0, () -> 0.0)
            .withTimeout(leaveTimeSec);
    }

    public SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser;

        if (AutoBuilder.isConfigured()) chooser = AutoBuilder.buildAutoChooser();

        else {
            chooser = new SendableChooser<>();
            DriverStation.reportWarning("AutoBuilder not configured (missing pathplanner/settings.json?), GUI autos unavailable", false);
        }

        chooser.setDefaultOption(
            "Shoot Preload Only",
            Commands.sequence(
                autoShoot(shootPreloadSec),
                fsm.requestStateCmd(State.IDLE)));

        chooser.addOption(
            "Shoot Preload + Leave",
            Commands.sequence(
                autoShoot(shootPreloadSec),
                fsm.requestStateCmd(State.IDLE),
                fsm.waitForState(State.IDLE),
                timedLeave()));

        return chooser;
    }
}
