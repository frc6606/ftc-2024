package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.pinkhawks.ftc.subsystems.Subsystem;

public class Claw implements Subsystem {
    private Servo jointServo, clawServo;
    private HardwareMap hardwareMap;

    public Claw (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void groundPos(){
        jointServo.setPosition(1);
    }

    public void backdropPos(){
        jointServo.setPosition(0.9);
    }

    public void takePixel(){
        clawServo.setPosition(1);
    }

    public void dropPixel(){
        clawServo.setPosition(0);
    }

    public void restPosition(){
        jointServo.setPosition(0.5);
        clawServo.setPosition(0.5);
    }

    @Override
    public void init() {
        jointServo = hardwareMap.get(Servo.class,"jointServo");
        clawServo = hardwareMap.get(Servo.class,"clawServo");
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
