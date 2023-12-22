package org.pinkhawks.ftc.actions;

public interface Action {

    void init();

    void update();

    boolean isFinished();

    void done();
}
