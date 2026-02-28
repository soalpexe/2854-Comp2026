// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StateMachine {
    private HashMap<State, Trigger> stateTriggers;
    private HashMap<Transition, Trigger> transitionTriggers;

    public State lastState, currentState;

    public enum State {
        IDLE,
        INTAKE,
        OUTTAKE
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

    public StateMachine() {
        lastState = State.IDLE;
        currentState = State.IDLE;

        stateTriggers = new HashMap<>();
        transitionTriggers = new HashMap<>();

        for (State state : State.values()) {
            stateTriggers.put(state, new Trigger(() -> currentState == state));
        }
    }

    public Trigger getStateTrigger(State state) {
        return stateTriggers.get(state);
    }
    
    public Trigger getTransitionTrigger(State startState, State endState) {
        return transitionTriggers.get(new Transition(startState, endState));
    }

    public void addTransitionTrigger(State startState, State endState) {
        transitionTriggers.put(
            new Transition(startState, endState),
            new Trigger(() -> lastState == startState && currentState == endState)
        );
    }

    public Command setStateCmd(State state) {
        return Commands.runOnce(() -> {
                lastState = currentState;
                currentState = state;
                
                CommandScheduler.getInstance().cancelAll();
            }
        );
    }
}
