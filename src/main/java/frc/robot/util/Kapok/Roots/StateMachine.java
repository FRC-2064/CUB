package frc.robot.util.Kapok.Roots;

import org.littletonrobotics.junction.Logger;

public abstract class StateMachine<W extends Enum<W>, C extends Enum<C>> {
    protected W wantedState;
    protected C currentState;
    protected C previousState;
    
    protected final String logKey;
    
    public StateMachine(String logKey, W initialWantedState, C initialCurrentState) {
        this.logKey = logKey;
        this.wantedState = initialWantedState;
        this.currentState = initialCurrentState;
        this.previousState = initialCurrentState;
    }

    public void update() {
        previousState = currentState;
        currentState = handleStateTransitions();
        applyState();
        
        Logger.recordOutput(logKey + "/WantedState", wantedState.toString());
        Logger.recordOutput(logKey + "/CurrentState", currentState.toString());
        Logger.recordOutput(logKey + "/PreviousState", previousState.toString());
    }
    
    protected abstract C handleStateTransitions();
    
    protected abstract void applyState();
    
    protected boolean hasStateChanged() {
        return currentState != previousState;
    }
    
    protected boolean didEnterState(C state) {
        return currentState == state && previousState != state;
    }
    
    protected boolean didExitState(C state) {
        return previousState == state && currentState != state;
    }
    
    public void setWantedState(W state) {
        this.wantedState = state;
    }
    
    public C getCurrentState() {
        return currentState;
    }
    
    public W getWantedState() {
        return wantedState;
    }
}