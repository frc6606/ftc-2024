package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.pinkhawks.ftc.subsystems.Subsystem;

@Config
public class Arm implements Subsystem {
    private PIDController controller;
    public static double p = 0.005, i = 0, d = 0.00009;
    public static double f = 0.2;
    private final double ticks_in_degree = 103.6 / 180.0;
    public static int backdropTarget = 460;
    public static int groundTarget = 100;
    public static int hangingTarget = 500;
    private DcMotorEx armMotor;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Arm(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public void goToBackdrop() {
        controller.setPID(p, i, d);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, backdropTarget);
        double ff = Math.cos(Math.toRadians(backdropTarget / ticks_in_degree)) * f;

        double power  = pid + ff;

        armMotor.setPower(power * 0.3);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", backdropTarget);
        telemetry.update();
    }

    public void goToGround() {
        controller.setPID(p, i, d);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, groundTarget);
        double ff = Math.cos(Math.toRadians(groundTarget / ticks_in_degree)) * f;

        double power  = pid + ff;

        armMotor.setPower(power *  0.3);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", groundTarget);
        telemetry.update();
    }

    public void hang() {
        controller.setPID(p, i, d);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, hangingTarget);
        double ff = Math.cos(Math.toRadians(hangingTarget / ticks_in_degree)) * f;

        double power  = pid + ff;
        armMotor.setPower(power * 0.3);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", hangingTarget);
        telemetry.update();
    }

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        armMotor = hardwareMap.get(DcMotorEx.class,"armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public int armPos(){
        return armMotor.getCurrentPosition();
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
