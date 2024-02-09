package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.pinkhawks.ftc.subsystems.Subsystem;

public class Drone implements Subsystem {

    private Servo droneServo;
    private HardwareMap hardwareMap;

    public Drone ( HardwareMap hardwareMap){
        this.hardwareMap =  hardwareMap;
    }

    public void launchDrone(){
        droneServo.setPosition(0.6);
    }
    public void restDronePos(){
        droneServo.setPosition(0.8);
    }

    @Override
    public void init() {
        droneServo = hardwareMap.get(Servo.class, "droneServo");
    }

    @Override
    public void log() {

    }

    @Override
    public void update() {

    }

    @Override
    public boolean check() {
        return false;
    }
}
