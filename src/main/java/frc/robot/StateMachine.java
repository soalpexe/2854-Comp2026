// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StateMachine {
    public enum State {
        IDLE,
        UNJAM,
        INTAKE,
        SHOOT
    }

    private class Transition {
        public final State startState, endState;

        public Transition(State startState, State endState) {
            this.startState = startState;
            this.endState = endState;
        }

        @Override
        public boolean equals(Object obj) {
            if (obj == null || getClass() != obj.getClass()) return false;
            if (this == obj) return true;

            Transition other = (Transition) obj;
            return startState == other.startState && endState == other.endState;
        }

        @Override
        public int hashCode() {
            return startState.hashCode() * 31 + endState.hashCode();
        }
    }

    private State activeState, requestedState;

    private HashMap<State, Trigger> stateTriggers;
    private HashMap<Transition, Command> transitions;

    public StateMachine() {
        activeState = State.IDLE;
        requestedState = State.IDLE;

        stateTriggers = new HashMap<>();
        transitions = new HashMap<>();

        for (State state : State.values()) {
            stateTriggers.put(state, new Trigger(() -> activeState == state && requestedState == state));

            configureState(
                state,
                Commands.none(),
                Commands.none(),
                Commands.none()
            );
        }
    }

    public State getActiveState() {
        return activeState;
    }
    
    public State getRequestedState() {
        return requestedState;
    }

    public void configureState(State state, Command enter, Command execute, Command exit) {
        stateTriggers.get(state)
            .onTrue(
                Commands.sequence(
                    enter,
                    Commands.repeatingSequence(execute)
                )
            )
            .onFalse(
                Commands.sequence(
                    exit,
                    Commands.defer(
                        () -> {
                            Transition transition = new Transition(activeState, requestedState);

                            if (transitions.containsKey(transition)) return transitions.get(transition);
                            return Commands.none();
                        },
                        Set.of()
                    ),
                    Commands.runOnce(() -> activeState = requestedState)
                )
            );
    }

    public void addTransition(State startState, State endState, Command cmd) {
        transitions.put(new Transition(startState, endState), cmd);
    }

    public Command requestStateCmd(State state) {
        return Commands.runOnce(() -> requestedState = state);
    }
}
