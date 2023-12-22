package org.pinkhawks.ftc.actions;


import java.util.ArrayList;
import java.util.List;


public class ParallelAction implements Action {
    private final ArrayList<Action> mActions;

    public ParallelAction(List<Action> actions){
        mActions = new ArrayList<>(actions);
    }

    @Override
    public void init(){
        mActions.forEach(Action :: init);
    }

    @Override
    public void update(){
        mActions.forEach(Action :: update);
    }

    @Override
    public boolean isFinished(){
        for(Action action : mActions){
            if(!action.isFinished()){
                return false;
            }
        }
        return true;
    }

    @Override
    public void done(){
        mActions.forEach(Action :: done);
    }
}
