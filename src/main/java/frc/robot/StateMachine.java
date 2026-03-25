// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StateMachine {
    public enum State {
        IDLE,
        STOW,
        INTAKE,
        SHOOT,
        SHOOT_AND_INTAKE,
        UNJAM
    }

    private record Transition(State startState, State endState) {}

    private State activeState, requestedState;

    private HashMap<State, Supplier<Command>> enterSuppliers, executeSuppliers, exitSuppliers;
    private HashMap<Transition, Supplier<Command>> transitionSuppliers;

    public StateMachine() {
        enterSuppliers = new HashMap<>();
        executeSuppliers = new HashMap<>();
        exitSuppliers = new HashMap<>();
        
        transitionSuppliers = new HashMap<>();

        new Trigger(() -> activeState == requestedState)
            .whileTrue(Commands.defer(() -> getExecuteCmd(activeState).repeatedly(), Set.of()))
            .onFalse(
                Commands.defer(
                    () -> {
                        Transition transition = new Transition(activeState, requestedState);
                        Command transitionCmd = transitionSuppliers.containsKey(transition) ?
                            getTransitionCmd(transition) :
                            Commands.sequence(
                                getExitCmd(activeState),
                                getEnterCmd(requestedState)
                            );

                        return Commands.sequence(
                            transitionCmd,
                            Commands.runOnce(() -> activeState = requestedState)
                        );
                    },
                    Set.of()
                )
            );

        for (State state : State.values()) {
            configureState(
                state,
                Commands::none,
                Commands::none,
                Commands::none
            );
        }

        initialize(State.STOW);
    }

    private Command getEnterCmd(State state) {
        return enterSuppliers.get(state).get();
    }
    
    private Command getExecuteCmd(State state) {
        return executeSuppliers.get(state).get();
    }
    
    private Command getExitCmd(State state) {
        return exitSuppliers.get(state).get();
    }
    
    private Command getTransitionCmd(Transition transition) {
        return transitionSuppliers.get(transition).get();
    }

    public State getState() {
        return activeState;
    }

    private void initialize(State state) {
        activeState = state;
        requestedState = state;

        CommandScheduler.getInstance().schedule(getEnterCmd(state));
    }

    public void configureState(State state, Supplier<Command> enterSupplier, Supplier<Command> executeSupplier, Supplier<Command> exitSupplier) {
        enterSuppliers.put(state, enterSupplier);
        executeSuppliers.put(state, executeSupplier);
        exitSuppliers.put(state, exitSupplier);
    }

    public void addTransition(State startState, State endState, Supplier<Command> transitionSupplier) {
        transitionSuppliers.put(new Transition(startState, endState), transitionSupplier);
    }

    public Command requestStateCmd(State state) {
        return Commands.runOnce(() -> {
            if (activeState == requestedState) requestedState = state;
        });
    }
}