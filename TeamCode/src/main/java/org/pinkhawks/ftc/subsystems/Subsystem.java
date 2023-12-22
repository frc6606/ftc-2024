package org.pinkhawks.ftc.subsystems;

public interface Subsystem {

    void init();

    void log();

    void update();

    boolean check();
}