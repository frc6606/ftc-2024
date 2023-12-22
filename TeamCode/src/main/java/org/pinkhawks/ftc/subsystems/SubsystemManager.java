package org.pinkhawks.ftc.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.LinkedList;

public class SubsystemManager {
    private final List<Subsystem> subsystems = new LinkedList<Subsystem>();
    private Telemetry telemetry = null;

    public void register(Subsystem subsystem) {
        this.subsystems.add(subsystem);
    }

    public void unregister(Subsystem subsystem){
        this.subsystems.remove(subsystem);
    }

    public boolean checkSubsystems() {
        boolean isHealthy = true;

        for(Subsystem s : subsystems) {
            isHealthy &= s.check();
        }

        return isHealthy;
    }

    public void log(){
        for (Subsystem s : subsystems) {
            s.log();
        }

        telemetry.update();
    }

    public void init(Telemetry telemetry){
        for (Subsystem s : subsystems){
            s.init();
        }

        this.telemetry = telemetry;
    }

    public void update(){
        for (Subsystem s : subsystems){
            s.update();
        }

        log();
    }

    public void clear() {
        subsystems.clear();
    }
}